"""Which command controller goes off so the next one can come on.

The command controllers of the arm claim the same command interfaces and therefore exclude one another: only one of
them may be active at a time.  ``ur_controller_mode_manager`` keeps them all loaded and switches between them, and
what it has to get right is the pair of sets it hands to ``switch_controller`` -- STRICT-ly, so a plan that activates
a second claimant is rejected as a whole and the arm keeps the hold target of the old controller.

Pure set arithmetic over controller names, so this module needs no ROS.  The broadcasters
(``joint_state_broadcaster``, ``io_and_status_controller``, ft/tcp/speed_scaling) claim no command interfaces, are
therefore not in the exclusive group, and are never touched by a plan.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import NamedTuple

#: Mode names and their controllers, as parallel arrays -- the node exposes both as parameters, these are the
#: defaults for the a200-0553.  Two lists rather than one mapping because ROS 2 parameters have no dict type.
DEFAULT_MODE_NAMES = ("trajectory", "freedrive", "forward_position", "forward_velocity", "passthrough")
DEFAULT_MODE_CONTROLLERS = (
    "arm_0_joint_trajectory_controller",
    "freedrive_mode_controller",
    "forward_position_controller",
    "forward_velocity_controller",
    "passthrough_trajectory_controller",
)


class SwitchPlan(NamedTuple):
    """What to hand to ``switch_controller`` -- or why not to call it at all."""

    activate: tuple[str, ...]
    deactivate: tuple[str, ...]
    #: ``None`` when the plan may be carried out; otherwise the reason to answer the caller with.
    refusal: str | None

    @property
    def is_noop(self) -> bool:
        """Nothing to do: the requested mode is already the only active one."""
        return not self.activate and not self.deactivate


def build_mode_map(
    mode_names: Iterable[str], mode_controllers: Iterable[str]
) -> tuple[dict[str, str], tuple[str, ...]]:
    """Turn the two parallel parameter arrays into the mapping plus the exclusive group.

    Zipping them without the length check would silently drop the trailing modes, and the node would answer
    ``Unknown mode`` for them from then on -- a configuration error that looks like a caller error.

    :raises ValueError: when the two lists differ in length.
    """
    names = list(mode_names)
    controllers = list(mode_controllers)
    if len(names) != len(controllers):
        raise ValueError(
            f"mode_names and mode_controllers must be the same length ({len(names)} vs {len(controllers)})"
        )
    # One controller may serve several modes; the exclusive group holds it once, in the order given.
    return dict(zip(names, controllers)), tuple(dict.fromkeys(controllers))


def plan_switch(
    mode: str,
    mode_to_controller: dict[str, str],
    exclusive: Iterable[str],
    active: Iterable[str],
    loaded: Iterable[str],
) -> SwitchPlan:
    """Plan the switch into ``mode`` from the controllers currently active.

    ``active`` and ``loaded`` are what ``list_controllers`` reported; only members of ``exclusive`` are ever
    deactivated, so the broadcasters stay up.  A controller that is not loaded is refused BY NAME here: STRICT
    ``switch_controller`` would reject it too, but with an error that does not say which controller was missing.
    """
    controller = mode_to_controller.get(mode)
    if controller is None:
        return SwitchPlan((), (), f"Unknown mode '{mode}'")
    if controller not in set(loaded):
        return SwitchPlan(
            (), (), f"Controller '{controller}' is not loaded - load it first via arm_controllers.launch.py"
        )
    active_in_group = [c for c in exclusive if c in set(active)]
    deactivate = tuple(c for c in active_in_group if c != controller)
    activate = () if controller in active_in_group else (controller,)
    return SwitchPlan(activate, deactivate, None)
