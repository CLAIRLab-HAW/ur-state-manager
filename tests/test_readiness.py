"""The readiness decisions of ``ur_state_manager``, without an arm.

These are the cases nobody wants to provoke on the a200-0553: emergency stop, protective stop, safety violation, an
arm powered on but with ExternalControl paused.  A unit test is not a formality here, it is the only practical way to
walk through them at all -- and every one of them decides whether the node reaches for the real UR5.

The functions take mode NAMES, not the ``ur_dashboard_msgs`` integers, so this file needs neither ROS nor the
message package (which is installed on the robot only).  ``state_manager`` maps int to name with the same tables it
already uses for its log lines.
"""

from __future__ import annotations

import pytest

from ur_state_manager.readiness import (
    GOOD_SAFETY,
    POWERED_MODES,
    RETRYABLE_SAFETY,
    TERMINAL_SAFETY,
    classify_ready,
    describe_state,
    is_ready,
    needs_recover,
)


# ---- needs_recover: the auto-recovery trigger -----------------------------------------------------------------
def test_a_powered_arm_with_a_paused_program_needs_recovery():
    assert needs_recover("POWER_ON", program_running=False) is True


@pytest.mark.parametrize("mode", POWERED_MODES)
def test_every_powered_mode_triggers_the_recovery(mode):
    assert needs_recover(mode, program_running=False) is True


def test_a_running_program_needs_no_recovery():
    assert needs_recover("RUNNING", program_running=True) is False


def test_an_unknown_program_status_never_triggers_the_recovery():
    """``None`` means the topic said nothing AND the dashboard fallback failed - that is not a licence to act."""
    assert needs_recover("RUNNING", program_running=None) is False


def test_a_powered_off_arm_is_left_alone():
    """POWER_OFF / DISCONNECTED / BOOTING mean: deliberately off, or not up yet."""
    for mode in ("POWER_OFF", "DISCONNECTED", "BOOTING", "IDLE_UNKNOWN"):
        assert needs_recover(mode, program_running=False) is False, mode


def test_freedrive_is_left_alone():
    """BACKDRIVE is hand guiding: somebody is holding the arm.  Recovering there restarts the program under them."""
    assert needs_recover("BACKDRIVE", program_running=False) is False


def test_an_unreadable_robot_mode_is_left_alone():
    assert needs_recover(None, program_running=False) is False


# ---- is_ready: the idempotence check of prepare ---------------------------------------------------------------
def test_the_arm_is_in_service_when_all_three_signals_agree():
    assert is_ready("RUNNING", "NORMAL", program_running=True) is True


@pytest.mark.parametrize("safety", GOOD_SAFETY)
def test_reduced_speed_still_counts_as_in_service(safety):
    assert is_ready("RUNNING", safety, program_running=True) is True


def test_a_powered_arm_without_external_control_is_not_in_service():
    assert is_ready("RUNNING", "NORMAL", program_running=False) is False


def test_an_unknown_program_status_is_not_in_service():
    assert is_ready("RUNNING", "NORMAL", program_running=None) is False


def test_a_protective_stop_is_not_in_service():
    assert is_ready("RUNNING", "PROTECTIVE_STOP", program_running=True) is False


def test_an_arm_that_is_merely_powered_is_not_in_service():
    assert is_ready("POWER_ON", "NORMAL", program_running=True) is False


# ---- classify_ready: retry or give up -------------------------------------------------------------------------
def test_a_verified_arm_reports_ok():
    verdict = classify_ready("RUNNING", "NORMAL", program_running=True)
    assert verdict.ok is True


@pytest.mark.parametrize("safety", TERMINAL_SAFETY)
def test_an_emergency_stop_is_not_retryable(safety):
    """Only a human releases an e-stop.  Retrying against it burns the bring-up attempts for nothing."""
    verdict = classify_ready("POWER_OFF", safety, program_running=False)
    assert verdict.ok is False
    assert verdict.retryable is False
    assert "e-stop" in verdict.detail.lower()


@pytest.mark.parametrize("safety", RETRYABLE_SAFETY)
def test_a_protective_stop_or_fault_is_retryable(safety):
    """The CB3 throws these out of its own brake release; the second attempt gets through (module docstring)."""
    verdict = classify_ready("RUNNING", safety, program_running=False)
    assert verdict.ok is False
    assert verdict.retryable is True


def test_a_state_that_is_merely_not_ready_yet_is_retryable():
    """Nothing is wrong, the arm just has not arrived - the caller keeps polling until its deadline."""
    verdict = classify_ready("POWER_ON", "NORMAL", program_running=False)
    assert verdict.ok is False
    assert verdict.retryable is True


def test_an_unreadable_state_is_retryable():
    verdict = classify_ready(None, None, program_running=None)
    assert verdict.ok is False
    assert verdict.retryable is True


def test_the_verdict_unpacks_like_the_tuple_the_caller_expects():
    ok, detail, retryable, _settled = classify_ready("RUNNING", "NORMAL", program_running=True)
    assert (ok, detail, retryable) == (True, "", True)


# ---- settled: keep polling, or stop now? ----------------------------------------------------------------------
@pytest.mark.parametrize("safety", RETRYABLE_SAFETY)
def test_a_protective_stop_stops_the_polling_at_once(safety):
    """Polling a p-stop out to the deadline gains nothing -- only a fresh bring-up clears it.  Waiting the full
    verify timeout here would eat the time the remaining attempts need."""
    assert classify_ready("RUNNING", safety, program_running=False).settled is True


@pytest.mark.parametrize("safety", TERMINAL_SAFETY)
def test_an_emergency_stop_stops_the_polling_at_once(safety):
    assert classify_ready("POWER_OFF", safety, program_running=False).settled is True


def test_an_arm_still_coming_up_keeps_the_polling_going():
    """POWER_ON with safety fine is a bring-up in flight - exactly what the verify timeout is there to wait out."""
    assert classify_ready("POWER_ON", "NORMAL", program_running=False).settled is False


def test_an_unreadable_state_keeps_the_polling_going():
    """The dashboard may just be slow to answer after a restart; that heals within the timeout."""
    assert classify_ready(None, None, program_running=None).settled is False


# ---- describe_state: the one detail line ----------------------------------------------------------------------
def test_the_description_names_all_three_signals():
    detail = describe_state("POWER_ON", "PROTECTIVE_STOP", program_running=False)
    assert "POWER_ON" in detail
    assert "PROTECTIVE_STOP" in detail
    assert "False" in detail


def test_an_unreadable_signal_is_named_as_unknown_not_as_none():
    detail = describe_state(None, None, program_running=None)
    assert "unknown" in detail
    assert "None" not in detail.replace("program_running=None", "")
