"""The runbook's two one-command entry points must stay runnable.

docs/5060_FULL_RUNBOOK.md §17 promises exactly two commands for the two
remaining steps (PICO teleoperation in simulation, and the real robot).  These
tests keep that promise honest: the scripts exist, are executable, are valid
bash, document their modes, and their preflight actually runs to a verdict
instead of dying halfway.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = {
    "run_pico_sim.sh": ["--check", "--replay", "--duration", "--policy-steps", "--no-viewer"],
    "run_robot_live.sh": ["--check", "--a3-host", "--confirm-live", "--duration"],
}


@pytest.mark.parametrize("name", sorted(SCRIPTS))
def test_entry_point_is_executable_bash(name):
    path = BRIDGE_ROOT / "scripts" / name
    assert path.is_file(), f"{path} is missing (docs/5060_FULL_RUNBOOK.md 17 relies on it)"
    assert path.stat().st_mode & 0o111, f"{path} is not executable"
    proc = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr


@pytest.mark.parametrize("name,flags", sorted(SCRIPTS.items()))
def test_entry_point_documents_its_modes(name, flags):
    proc = subprocess.run(
        ["bash", str(BRIDGE_ROOT / "scripts" / name), "--help"],
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr
    for flag in flags:
        assert flag in proc.stdout, f"{name} --help does not mention {flag}"


@pytest.mark.parametrize("name", sorted(SCRIPTS))
def test_preflight_reaches_a_verdict(name):
    """`--check` must run every section and print a verdict.

    A missing interpreter or a missing optional asset is allowed (returncode 1);
    what must not happen is the script dying part-way with no verdict.
    """
    proc = subprocess.run(
        ["bash", str(BRIDGE_ROOT / "scripts" / name), "--check"],
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert proc.returncode in (0, 1), f"{name} --check exited {proc.returncode}\n{proc.stderr[-2000:]}"
    assert "[preflight]" in proc.stdout
    assert " ok, " in proc.stdout, proc.stdout[-2000:]


def test_robot_script_refuses_to_publish_without_confirmation():
    """The go-live gate is the last thing between a test run and a real robot."""
    source = (BRIDGE_ROOT / "scripts" / "run_robot_live.sh").read_text(encoding="utf-8")
    assert "--confirm-live" in source
    assert "refusing to publish motion to a real robot" in source
    assert 'exit 3' in source
    # and it must not silently default to publishing
    assert "CONFIRM_LIVE=0" in source


def test_runbook_links_the_entry_points():
    runbook = (BRIDGE_ROOT / "docs" / "5060_FULL_RUNBOOK.md").read_text(encoding="utf-8")
    assert "scripts/run_pico_sim.sh" in runbook
    assert "scripts/run_robot_live.sh" in runbook


def _live_chain_module():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "run_live_chain", BRIDGE_ROOT / "tools" / "run_live_chain.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stream_stats_parser_survives_python_none():
    """The no-headset path must report guidance, not raise.

    SONIC prints the stream stats as a *Python* dict.  ``last_seq`` is ``None``
    until the first packet arrives, so the old ``json.loads(text.replace("'",
    '"'))`` raised JSONDecodeError exactly when the operator had forgotten to
    unpause the PICO sender -- the single most common start-up mistake.
    """
    module = _live_chain_module()
    log = (
        "[reference-stream] no A3_REFERENCE_V1 packet yet; holding the startup pose "
        "for up to 30 s (is the PICO sender RUNNING and the bridge publishing?)\n"
        "[reference-stream] {'received': 0, 'rejected': 0, 'dropped': 0, 'jumps': 0, "
        "'max_jump_ms': 0.0, 'interpolated': 0, 'max_gap_ms': 0.0, 'last_seq': None, "
        "'last_reason': '', 'latency_p50_ms': None, 'latency_p95_ms': None}\n"
    )
    stats = module.parse_stream_stats(log)
    assert stats["received"] == 0
    assert stats["last_seq"] is None
    assert module.parse_stream_stats("") == {}
    assert module.parse_stream_stats("[reference-stream] {not a dict}") == {}
    assert module.parse_stream_stats("[reference-stream] {'received': 7, 'last_seq': 42}")["last_seq"] == 42


def test_live_chain_exposes_a_viewer_mode():
    """The teleoperation entry point must be able to show the MuJoCo window.

    sim2sim disables the passive viewer under --batch-once, so "show me the
    robot" means omitting that flag; run_live_chain.py grew --viewer for it.
    """
    module = _live_chain_module()
    proc = subprocess.run(
        [sys.executable, str(BRIDGE_ROOT / "tools" / "run_live_chain.py"), "--help"],
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr
    assert "--viewer" in proc.stdout
    source = (BRIDGE_ROOT / "tools" / "run_live_chain.py").read_text(encoding="utf-8")
    # headless is the default (acceptance runs), and the flag is what drops batch mode
    assert 'if not args.viewer:' in source and 'sim_cmd.append("--batch-once")' in source
    # and it must degrade instead of crashing when there is no display
    assert "WAYLAND_DISPLAY" in source


def test_pico_sim_defaults_to_the_viewer():
    source = (BRIDGE_ROOT / "scripts" / "run_pico_sim.sh").read_text(encoding="utf-8")
    assert "VIEWER=1" in source, "the manual teleop entry point should show the window by default"
    assert "--no-viewer" in source
    assert "Space pause" in source, "controls should be echoed to the operator"


def test_pico_sim_checks_the_pc_service_port():
    """Headset-only setups cannot work: the SDK talks to localhost:60061."""
    source = (BRIDGE_ROOT / "scripts" / "run_pico_sim.sh").read_text(encoding="utf-8")
    assert "60061" in source
    assert "/opt/apps/roboticsservice/runService.sh" in source
    assert "the headset alone is not enough" in source


def test_pico_probe_tool_reports_a_verdict():
    """A listening PC Service is not a streaming headset.

    The 2026-09-28 failure: the PC Service was up and the headset had a TCP
    connection, but the device had gone offline 7 minutes earlier, so the run
    sat for 30 s and then reported "is the PICO sender RUNNING?".  The probe
    asks the SDK directly and reports through its exit code.
    """
    path = BRIDGE_ROOT / "tools" / "probe_pico_sdk.py"
    assert path.is_file()
    proc = subprocess.run([sys.executable, str(path), "--help"], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    source = path.read_text(encoding="utf-8")
    for marker in ("BODY_DATA_OK", "BODY_DATA_TIMEOUT", "SDK_MISSING"):
        assert marker in source
    # it must not treat the SDK's static-destructor abort as a verdict
    assert "os._exit(0)" in source


def test_pico_sim_gates_on_real_body_data():
    source = (BRIDGE_ROOT / "scripts" / "run_pico_sim.sh").read_text(encoding="utf-8")
    assert "tools/probe_pico_sdk.py" in source, "preflight must ask the SDK, not just the port"
    assert "BODY_DATA_TIMEOUT" in source
    assert "--skip-pico-probe" in source
    # the sender must be unbuffered, otherwise its progress lines (and therefore
    # the wrapper's live diagnosis) only appear once the process dies
    assert "PYTHONUNBUFFERED=1" in source


def test_pico_sender_output_is_reachable_while_it_runs():
    """Regression for the 30 s silent wait: block-buffered output hid the reason."""
    source = (BRIDGE_ROOT / "scripts" / "run_pico_sim.sh").read_text(encoding="utf-8")
    assert "waiting for body data" in source, "the wrapper should watch for this line live"
    assert '"-u"' in source or " -u " in source


def test_fake_pico_sender_matches_the_real_payload():
    """The harness that caught the live-path fall must stay usable.

    --replay never touches the ZMQ subscriber, so this tool is the only way to
    exercise the packet -> frame path on a desk.
    """
    path = BRIDGE_ROOT / "tools" / "fake_pico_sender.py"
    assert path.is_file()
    proc = subprocess.run([sys.executable, str(path), "--help"], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    source = path.read_text(encoding="utf-8")
    # must be byte-compatible with pico_pose_zmq_minimal.py, i.e. no root by default
    for field in ('"smpl_pose"', '"smpl_joints"', '"body_quat_w"'):
        assert field in source
    assert '"root_translation"' in source, "--with-root should be able to publish one"
    assert "PACKED_HEADER_SIZE = 1280" in source


def test_pico_sim_exposes_the_fake_sender_for_desk_testing():
    source = (BRIDGE_ROOT / "docs" / "5060_FULL_RUNBOOK.md").read_text(encoding="utf-8")
    assert "tools/fake_pico_sender.py" in source


def test_generated_motions_start_from_a_natural_stance():
    """The validation set must not be "a human holding their arms out".

    make_smplx_validation_motions.py used to interpolate every clip from the
    SMPL-X rest pose, whose arms sit 74-82 deg from vertical.  The retarget
    faithfully reproduced that (A3 shoulder_roll 1.545 rad = its T-pose), so the
    whole acceptance set exercised a posture no operator ever stands in, and a
    real headset user with arms down could not be followed.
    """
    source = (BRIDGE_ROOT / "tools" / "make_smplx_validation_motions.py").read_text(encoding="utf-8")
    assert "def natural_stance(" in source
    assert "arm_angle_from_vertical" in source
    # the stance must be added to every clip, not only to `stand`
    assert "build_clip(fk, frames, specs) + stance[None, :, :]" in source


def test_acceptance_clips_have_arms_down():
    """End-to-end guard: the retargeted acceptance set stands with arms at the sides."""
    import numpy as np

    from a3_teleop_bridge.contract import load_contract
    from a3_teleop_bridge.umr.offline import load_umr_result

    clip_dir = Path("/home/wusichen/a3_teleop_ws/UMR/output/a3_pico_all")
    if not clip_dir.is_dir():
        pytest.skip("acceptance npz set not present on this machine")
    names = list(load_contract().policy_joint_names)
    result = load_umr_result(clip_dir / "m5_stand_smplx_agibot_a3.npz")
    values = np.asarray(
        [[result.joint_value(i, n) for n in names] for i in range(result.n_frames)]
    )
    left = values[:, names.index("left_shoulder_roll_joint")]
    right = values[:, names.index("right_shoulder_roll_joint")]
    # the A3's own keyframe stands at +0.112; its T-pose is +1.545
    assert left.max() < 0.5, f"left arm is not hanging: shoulder_roll {left.max():+.3f}"
    assert right.min() > -0.5, f"right arm is not hanging: shoulder_roll {right.min():+.3f}"
    assert left.mean() > 0.0 and right.mean() < 0.0


def test_sender_publishes_root_translation():
    """Without it the operator's steps never leave the headset (in-place walking)."""
    source = Path(
        "/home/wusichen/a3_teleop_ws/sonic_for_a3/gear_sonic/scripts/pico_pose_zmq_minimal.py"
    )
    if not source.is_file():
        pytest.skip("sonic_for_a3 checkout not present on this machine")
    text = source.read_text(encoding="utf-8")
    assert "_compute_root_translation" in text
    assert '"root_translation": np.stack(' in text
    assert "--no-root-translation" in text
