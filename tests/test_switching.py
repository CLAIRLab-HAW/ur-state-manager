"""The controller switch plan of ``ur_controller_mode_manager``, without a controller_manager.

The command controllers claim the same command interfaces and therefore exclude one another: what the node has to get
right is which of them to deactivate before it activates the next.  Getting that wrong is not a cosmetic error -- a
``switch_controller`` that activates a second claimant is rejected STRICT-ly, and the arm keeps whatever hold target
the old controller had.

Pure set arithmetic, so it needs neither ROS nor a running arm.
"""

from __future__ import annotations

import pytest
from ur_state_manager.switching import DEFAULT_MODE_CONTROLLERS, DEFAULT_MODE_NAMES, build_mode_map, plan_switch

TRAJECTORY = "arm_0_joint_trajectory_controller"
FREEDRIVE = "freedrive_mode_controller"
FORWARD = "forward_position_controller"
EXCLUSIVE = (TRAJECTORY, FREEDRIVE, FORWARD)
MODES = {"trajectory": TRAJECTORY, "freedrive": FREEDRIVE, "forward_position": FORWARD}
LOADED = frozenset(EXCLUSIVE)


def _plan(mode, active, loaded=LOADED):
    return plan_switch(mode, MODES, EXCLUSIVE, active=active, loaded=loaded)


# ---- build_mode_map: the parallel-array parameters ------------------------------------------------------------
def test_the_two_parameter_lists_become_one_mapping():
    mapping, exclusive = build_mode_map(["trajectory", "freedrive"], [TRAJECTORY, FREEDRIVE])
    assert mapping == {"trajectory": TRAJECTORY, "freedrive": FREEDRIVE}
    assert exclusive == (TRAJECTORY, FREEDRIVE)


def test_lists_of_different_length_are_refused():
    """Silently zipping them would drop the trailing modes - and the node would answer 'unknown mode' forever."""
    with pytest.raises(ValueError):
        build_mode_map(["trajectory", "freedrive"], [TRAJECTORY])


def test_a_controller_serving_two_modes_appears_once_in_the_exclusive_group():
    _mapping, exclusive = build_mode_map(["a", "b"], [TRAJECTORY, TRAJECTORY])
    assert exclusive == (TRAJECTORY,)


def test_the_shipped_defaults_are_a_consistent_pair():
    mapping, _exclusive = build_mode_map(list(DEFAULT_MODE_NAMES), list(DEFAULT_MODE_CONTROLLERS))
    assert mapping["trajectory"] == TRAJECTORY


# ---- plan_switch: which controller goes off, which comes on ---------------------------------------------------
def test_switching_from_one_mode_to_another_deactivates_the_incumbent():
    plan = _plan("freedrive", active=[TRAJECTORY])
    assert plan.activate == (FREEDRIVE,)
    assert plan.deactivate == (TRAJECTORY,)
    assert plan.refusal is None


def test_switching_with_nothing_active_only_activates():
    plan = _plan("trajectory", active=[])
    assert plan.activate == (TRAJECTORY,)
    assert plan.deactivate == ()


def test_every_other_member_of_the_exclusive_group_goes_off():
    """More than one active claimant is a state ros2_control should not reach - the plan must clear it anyway."""
    plan = _plan("trajectory", active=[FREEDRIVE, FORWARD])
    assert plan.activate == (TRAJECTORY,)
    assert set(plan.deactivate) == {FREEDRIVE, FORWARD}


def test_the_target_is_not_activated_twice_when_it_is_already_running():
    plan = _plan("trajectory", active=[TRAJECTORY, FREEDRIVE])
    assert plan.activate == ()
    assert plan.deactivate == (FREEDRIVE,)
    assert plan.is_noop is False


def test_switching_into_the_mode_that_is_already_alone_is_a_no_op():
    plan = _plan("trajectory", active=[TRAJECTORY])
    assert plan.is_noop is True
    assert plan.refusal is None


def test_a_broadcaster_is_never_deactivated():
    """joint_state_broadcaster & co. claim no command interfaces and stay out of the exclusive group."""
    plan = _plan("trajectory", active=["joint_state_broadcaster", FREEDRIVE])
    assert "joint_state_broadcaster" not in plan.deactivate


def test_an_unknown_mode_is_refused():
    plan = _plan("dance", active=[])
    assert plan.refusal is not None
    assert "dance" in plan.refusal
    assert plan.activate == ()
    assert plan.deactivate == ()


def test_a_controller_that_is_not_loaded_is_refused_by_name():
    """STRICT switch_controller would reject it anyway - but with an error that does not say which controller."""
    plan = _plan("freedrive", active=[TRAJECTORY], loaded=frozenset({TRAJECTORY}))
    assert plan.refusal is not None
    assert FREEDRIVE in plan.refusal
    assert plan.deactivate == ()


def test_a_refusal_never_carries_a_switch():
    for plan in (_plan("dance", active=[TRAJECTORY]), _plan("freedrive", active=[], loaded=frozenset())):
        assert plan.refusal is not None
        assert plan.activate == ()
        assert plan.deactivate == ()
