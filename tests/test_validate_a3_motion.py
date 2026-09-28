"""Numeric motion validator: left/right sign convention vs posture symmetry.

The acceptance failure this file pins down: the old ``*_mirror`` rule compared
the *raw whole-clip mean* of a mirrored roll pair and demanded opposite signs.
The A3 neutral pose is not roll-symmetric (left_hip_roll -0.0469 rad /
right_hip_roll -0.0179 rad, both negative, and the ``stand`` clip is constant at
exactly those values), so a clip whose hip roll merely drifted by 3 mrad used to
trip the rule and then fail on a comparison dominated by the static offset.

These tests hold the new contract in place:

  * a static neutral offset plus ordinary dynamic motion PASSES;
  * an inherently one-sided motion (lift one foot, raise one arm) PASSES;
  * a real sign/mirror break still FAILS;
  * every physical check (finite, limits, velocity, knees, root) stays hard.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    "validate_a3_motion", BRIDGE_ROOT / "tools" / "validate_a3_motion.py"
)
validator = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(validator)

DT = 1.0 / 30.0
N_FRAMES = 60

#: the A3 pose-init neutral actually produced by the retargeter -- measured from
#: the ``m5_stand`` clip, which is constant over all 150 of its frames
NEUTRAL = {
    "left_shoulder_roll_joint": +1.5450,
    "right_shoulder_roll_joint": -1.5415,
    "left_hip_roll_joint": -0.0469,
    "right_hip_roll_joint": -0.0179,
    "left_ankle_roll_joint": +0.0189,
    "right_ankle_roll_joint": -0.0044,
}


def synth(series: dict[str, np.ndarray] | None = None, n: int = N_FRAMES) -> dict:
    """Build a valid 29-DoF trajectory, optionally overriding named joints."""
    names = list(validator.load_contract().policy_joint_names)
    joint_pos = np.zeros((n, len(names)))
    index = {name: i for i, name in enumerate(names)}
    for name, values in (series or {}).items():
        joint_pos[:, index[name]] = values

    root_pos = np.zeros((n, 3))
    root_pos[:, 0] = np.linspace(0.0, 0.05, n)
    root_pos[:, 2] = 1.07
    root_quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (n, 1))
    joint_vel = np.zeros_like(joint_pos)
    if n > 1:
        joint_vel[1:] = np.diff(joint_pos, axis=0) / DT
        joint_vel[0] = joint_vel[1]
    return {
        "joint_pos": joint_pos,
        "joint_vel": joint_vel,
        "root_pos": root_pos,
        "root_quat": root_quat,
        "names": names,
    }


def run(series: dict[str, np.ndarray] | None = None, neutral=None, n: int = N_FRAMES) -> dict:
    traj = synth(series, n=n)
    return validator.validate(
        traj["joint_pos"],
        traj["joint_vel"],
        traj["root_pos"],
        traj["root_quat"],
        DT,
        bodies=None,
        neutral=neutral,
    )


def ramp(n: int, start: float, delta: float) -> np.ndarray:
    """Half-cosine ease from ``start`` to ``start + delta`` (no velocity spike)."""
    phase = 0.5 * (1.0 - np.cos(np.linspace(0.0, np.pi, n)))
    return start + delta * phase


# --------------------------------------------------------------------------
# the false positive that blocked the acceptance run
# --------------------------------------------------------------------------
def test_static_neutral_offset_is_not_a_mirror_failure():
    """The exact m5_bend_knees shape: both means negative, only an offset.

    raw means -0.068 / -0.025 rad used to hard-fail because both are negative,
    even though the *motion* is a sub-degree balance drift on top of the
    non-symmetric neutral.
    """
    n = N_FRAMES
    report = run(
        {
            "left_hip_roll_joint": ramp(n, NEUTRAL["left_hip_roll_joint"], -0.021),
            "right_hip_roll_joint": ramp(n, NEUTRAL["right_hip_roll_joint"], -0.007),
        }
    )
    assert report["checks"]["hip_roll_joint_mirror"]["ok"] is True
    assert report["acceptable"] is True, report["problems"]
    entry = report["mirror_reference"]["hip_roll_joint"]
    assert entry["left"] == pytest.approx(NEUTRAL["left_hip_roll_joint"], abs=1e-9)
    assert entry["right"] == pytest.approx(NEUTRAL["right_hip_roll_joint"], abs=1e-9)


def test_asymmetric_single_leg_motion_passes():
    """m5_lift_left_foot / m5_step_forward_slow shape: one hip rolls, one rests."""
    n = N_FRAMES
    report = run(
        {
            "left_hip_roll_joint": np.full(n, NEUTRAL["left_hip_roll_joint"]),
            "right_hip_roll_joint": ramp(n, NEUTRAL["right_hip_roll_joint"], -0.071),
        }
    )
    assert report["checks"]["hip_roll_joint_mirror"]["ok"] is True
    assert report["acceptable"] is True, report["problems"]


def test_small_excursion_is_reported_as_non_fatal_diagnostic():
    """Same-direction but sub-gross motion: a note, never a failure."""
    n = N_FRAMES
    report = run(
        {
            "left_hip_roll_joint": ramp(n, NEUTRAL["left_hip_roll_joint"], -0.090),
            "right_hip_roll_joint": ramp(n, NEUTRAL["right_hip_roll_joint"], -0.060),
        }
    )
    assert report["checks"]["hip_roll_joint_mirror"]["ok"] is True
    assert report["acceptable"] is True
    assert any("hip_roll_joint_mirror" in note for note in report["diagnostics"])


# --------------------------------------------------------------------------
# a real sign / mirror break must still fail
# --------------------------------------------------------------------------
def test_gross_same_direction_hip_roll_fails():
    """Both hip rolls sweeping 0.40 rad the same way is a broken convention.

    Both values stay inside their own joint limits (left +0.353 <= 1.6057,
    right +0.382 <= 0.5236), so the mirror check is the only thing that can
    catch it -- which is exactly why it must not have been deleted.
    """
    n = N_FRAMES
    report = run(
        {
            "left_hip_roll_joint": ramp(n, NEUTRAL["left_hip_roll_joint"], +0.40),
            "right_hip_roll_joint": ramp(n, NEUTRAL["right_hip_roll_joint"], +0.40),
        }
    )
    assert report["checks"]["joint_limits"]["ok"] is True
    assert report["checks"]["hip_roll_joint_mirror"]["ok"] is False
    assert report["acceptable"] is False
    assert "SAME" in report["checks"]["hip_roll_joint_mirror"]["detail"]


def test_sign_swapped_shoulder_raise_fails():
    """A raise that drives both shoulders into the same world direction."""
    n = N_FRAMES
    report = run(
        {
            "left_shoulder_roll_joint": ramp(n, NEUTRAL["left_shoulder_roll_joint"], +0.60),
            "right_shoulder_roll_joint": ramp(n, NEUTRAL["right_shoulder_roll_joint"], +0.60),
        }
    )
    assert report["checks"]["joint_limits"]["ok"] is True
    assert report["checks"]["shoulder_roll_joint_mirror"]["ok"] is False
    assert report["acceptable"] is False


def test_mirrored_scissor_still_passes():
    """Guard against over-strictness: the correct mirrored pairing is accepted."""
    n = N_FRAMES
    report = run(
        {
            "left_shoulder_roll_joint": ramp(n, NEUTRAL["left_shoulder_roll_joint"], +0.60),
            "right_shoulder_roll_joint": ramp(n, NEUTRAL["right_shoulder_roll_joint"], -0.60),
        }
    )
    assert report["checks"]["shoulder_roll_joint_mirror"]["ok"] is True
    assert report["acceptable"] is True, report["problems"]


def test_explicit_neutral_reference_changes_the_reading():
    """An explicit neutral pose overrides the clip's own frame 0."""
    n = N_FRAMES
    series = {
        "left_hip_roll_joint": ramp(n, NEUTRAL["left_hip_roll_joint"], +0.40),
        "right_hip_roll_joint": ramp(n, NEUTRAL["right_hip_roll_joint"], +0.40),
    }
    own = run(series)
    assert own["checks"]["hip_roll_joint_mirror"]["ok"] is False
    assert own["mirror_reference"]["hip_roll_joint"]["reference"].startswith("clip frame 0")

    # a reference that puts the same trajectory on the mirrored side of the
    # neutral flips the reading -- proof that the reference really is used
    shifted = dict(NEUTRAL)
    shifted["left_hip_roll_joint"] = -0.30
    shifted["right_hip_roll_joint"] = +0.50
    explicit = run(series, neutral=shifted)
    entry = explicit["mirror_reference"]["hip_roll_joint"]
    assert entry["left"] == pytest.approx(-0.30)
    assert entry["right"] == pytest.approx(+0.50)
    assert entry["excursion_left_rad"] != own["mirror_reference"]["hip_roll_joint"]["excursion_left_rad"]
    assert explicit["checks"]["hip_roll_joint_mirror"]["ok"] is True
    assert explicit["acceptable"] is True, explicit["problems"]


# --------------------------------------------------------------------------
# the physical checks stay hard
# --------------------------------------------------------------------------
def test_non_finite_is_fatal():
    n = N_FRAMES
    values = ramp(n, 0.0, 0.20)
    values[10] = np.nan
    report = run({"left_elbow_joint": values})
    assert report["checks"]["finite"]["ok"] is False
    assert report["acceptable"] is False


def test_limit_violation_is_fatal():
    n = N_FRAMES
    report = run({"left_elbow_joint": np.full(n, 5.0)})
    assert report["checks"]["joint_limits"]["ok"] is False
    assert report["acceptable"] is False


def test_reverse_knee_is_fatal():
    n = N_FRAMES
    report = run({"left_knee_joint": ramp(n, 0.0, -0.10)})
    assert report["checks"]["left_knee_joint_direction"]["ok"] is False
    assert report["acceptable"] is False


def test_root_teleport_is_fatal():
    n = N_FRAMES
    traj = synth(n=n)
    traj["root_pos"][n // 2, 0] += 1.0
    report = validator.validate(
        traj["joint_pos"], traj["joint_vel"], traj["root_pos"], traj["root_quat"], DT
    )
    assert report["checks"]["root_no_teleport"]["ok"] is False
    assert report["acceptable"] is False


def test_velocity_spike_is_fatal():
    n = N_FRAMES
    values = np.zeros(n)
    values[n // 2] = 2.0
    report = run({"left_elbow_joint": values})
    assert report["checks"]["joint_velocity"]["ok"] is False
    assert report["acceptable"] is False


def test_mirror_diagnostic_never_masks_a_physical_failure():
    n = N_FRAMES
    report = run(
        {
            "left_hip_roll_joint": ramp(n, NEUTRAL["left_hip_roll_joint"], +0.40),
            "right_hip_roll_joint": ramp(n, NEUTRAL["right_hip_roll_joint"], +0.40),
            "right_elbow_joint": np.full(n, 9.0),  # out of limits
        }
    )
    assert report["checks"]["hip_roll_joint_mirror"]["ok"] is False
    assert report["checks"]["joint_limits"]["ok"] is False
    assert len(report["problems"]) >= 2


def test_straight_knee_is_flagged_but_not_failed():
    """The pinned-knee artefact is visible without making a clip unacceptable."""
    n = N_FRAMES
    report = run({"left_knee_joint": np.zeros(n), "right_knee_joint": np.zeros(n)})
    assert report["acceptable"] is True
    assert report["knee_excursion_rad"]["left_knee_joint"] == 0.0
    assert any("knee_joint: flexes by only" in note for note in report["diagnostics"])


# --------------------------------------------------------------------------
# the real acceptance clips
# --------------------------------------------------------------------------
REAL_CLIP_DIR = Path("/home/wusichen/a3_teleop_ws/UMR/output/a3_pico_all")


@pytest.mark.skipif(
    not REAL_CLIP_DIR.is_dir(), reason="acceptance npz set not present on this machine"
)
def test_all_nine_acceptance_clips_pass():
    from a3_teleop_bridge.umr.offline import load_umr_result

    names = list(validator.load_contract().policy_joint_names)
    clips = sorted(REAL_CLIP_DIR.glob("*.npz"))
    assert len(clips) == 9, [c.name for c in clips]
    failures = []
    for path in clips:
        result = load_umr_result(path)
        joint_pos = np.asarray(
            [[result.joint_value(i, n) for n in names] for i in range(result.n_frames)]
        )
        joint_vel = np.zeros_like(joint_pos)
        joint_vel[1:] = np.diff(joint_pos, axis=0) / result.dt
        joint_vel[0] = joint_vel[1]
        report = validator.validate(
            joint_pos, joint_vel, result.root_pos, result.root_quat_wxyz, result.dt
        )
        if not report["acceptable"]:
            failures.append(f"{path.stem}: {report['problems']}")
    assert not failures, "\n".join(failures)
