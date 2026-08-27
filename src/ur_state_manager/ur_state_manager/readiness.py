"""Is the UR5 in service, and if not: retry or give up?

Three decisions that ``state_manager`` makes over and over, lifted out of the node so that they can be walked through
without an arm.  Their input is always the same triple -- robot mode, safety mode, ExternalControl status -- and each
of the three signals may be ``None``: the dashboard did not answer, or the latched topic never reached this late
joiner.  ``None`` is not a fourth value to guess at, it means "not known", and the decisions below all fall to the
safe side on it.

The modes arrive as NAMES (``"RUNNING"``, ``"PROTECTIVE_STOP"``), not as the ``ur_dashboard_msgs`` integers.  Two
reasons: this module then needs no ROS at all -- ``ur_dashboard_msgs`` is installed on the robot and in none of the
offboard images -- and the constants keep exactly one source, the ``ROBOT_MODE_NAMES``/``SAFETY_MODE_NAMES`` tables
in ``state_manager``, which are built from the message package itself.  A mode the tables do not know maps to
``"UNKNOWN(n)"`` and is therefore in none of the tuples below, which is the correct answer for a value nobody
recognises.
"""

from __future__ import annotations

from typing import NamedTuple

#: Robot modes in which the arm carries current, so ExternalControl COULD run.  ``POWER_OFF``/``DISCONNECTED``/
#: ``BOOTING`` are deliberately absent (arm off, or not up yet), and so is ``BACKDRIVE``: that is hand guiding, and
#: somebody has their hands on the arm.
POWERED_MODES = ("POWER_ON", "IDLE", "RUNNING")

#: Safety modes in which the arm may move.  ``REDUCED`` counts: the arm works, just more slowly.
GOOD_SAFETY = ("NORMAL", "REDUCED")

#: Safety modes that no software releases -- somebody has to turn the button.  Retrying against them only burns the
#: bring-up attempts.
TERMINAL_SAFETY = ("SYSTEM_EMERGENCY_STOP", "ROBOT_EMERGENCY_STOP")

#: Safety modes that a fresh bring-up heals.  The CB3 throws these out of its own brake release, before ROS streams
#: anything -- see the module docstring of ``state_manager``.
RETRYABLE_SAFETY = ("PROTECTIVE_STOP", "VIOLATION", "FAULT")


class ReadyVerdict(NamedTuple):
    """Result of ``classify_ready``.

    ``retryable`` and ``settled`` answer two different questions, and conflating them costs real time on the arm:
    ``retryable`` says whether a fresh BRING-UP can help, ``settled`` whether POLLING can.  A protective stop is both
    (retry it, but stop watching), an arm still coming up is neither-yet (do not give up, keep watching).
    """

    ok: bool
    detail: str
    #: A repeated bring-up can heal this.  ``False`` only for an e-stop, which no software releases.
    retryable: bool
    #: The state will not change on its own -- stop polling and act now.  ``False`` means "not there YET".
    settled: bool


def needs_recover(robot_mode: str | None, program_running: bool | None) -> bool:
    """Is the arm powered while ExternalControl is NOT running? -> the auto-recovery trigger.

    Exactly the state after a late power-up or a "Paused" on the teach pendant: current is on, but the ROS program
    is not.  Anything else is left alone -- an arm deliberately switched off, one still booting, and above all
    ``BACKDRIVE``, where a restart would happen under somebody's hands.

    ``program_running is None`` means the topic said nothing and the dashboard fallback failed too; that is not a
    licence to act on the real arm, so the answer is ``False``.
    """
    if program_running is not False:
        return False
    return robot_mode in POWERED_MODES


def is_ready(robot_mode: str | None, safety: str | None, program_running: bool | None) -> bool:
    """Is the arm in service: RUNNING, safety fine, ExternalControl running?

    Used twice: as the idempotence check of ``prepare`` (already in service -> no ``robot_state_helper`` needed) and
    as the success condition of the bring-up verification.  One predicate, so the two can not drift apart.
    """
    return robot_mode == "RUNNING" and safety in GOOD_SAFETY and program_running is True


def describe_state(robot_mode: str | None, safety: str | None, program_running: bool | None) -> str:
    """The one-line state for log output and error messages.  ``None`` reads as ``unknown``, not as ``None``."""
    return (
        f"robot_mode={robot_mode if robot_mode is not None else 'unknown'} "
        f"safety={safety if safety is not None else 'unknown'} "
        f"program_running={program_running}"
    )


def classify_ready(robot_mode: str | None, safety: str | None, program_running: bool | None) -> ReadyVerdict:
    """One poll of the bring-up verification: ready, still waiting, or beyond saving?

    ``robot_state_helper`` reports success as soon as RUNNING is reached and play/resend has gone out -- a protective
    stop falling DURING the bring-up slips through that gap.  Hence the check of our own, and hence the two flags:
    ``retryable`` says whether repeating the bring-up can help -- it can not against an e-stop, and the caller must
    stop instead of spending its remaining attempts on it -- while ``settled`` says whether there is any point in
    looking again.

    A state that is merely not ready yet is ``retryable`` and NOT ``settled``: the caller polls on until its own
    deadline, which is exactly the window a bring-up in flight needs.
    """
    if is_ready(robot_mode, safety, program_running):
        return ReadyVerdict(True, "", True, True)
    detail = describe_state(robot_mode, safety, program_running)
    if safety in TERMINAL_SAFETY:
        return ReadyVerdict(False, f"{detail} (E-stop: can only be released manually)", False, True)
    if safety in RETRYABLE_SAFETY:
        return ReadyVerdict(False, detail, True, True)
    return ReadyVerdict(False, detail, True, False)
