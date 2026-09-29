"""The runbook's two one-command entry points must stay runnable.

docs/5060_FULL_RUNBOOK.md §17 promises exactly two commands for the two
remaining steps (PICO teleoperation in simulation, and the real robot).  These
tests keep that promise honest: the scripts exist, are executable, are valid
bash, document their modes, and their preflight actually runs to a verdict
instead of dying halfway.
"""

from __future__ import annotations

import os
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


def test_viewer_sessions_stand_back_up_after_a_fall():
    """A fall must not end a viewer session, and must not touch batch runs.

    Leaving the robot on the floor for the rest of the run made an interactive
    session useless after the first fall, so viewer runs now pass --reset-on-fall
    and sim2sim re-places the robot on the live reference pose (root position,
    anchor orientation and the 29 joint positions, run through
    fill_loop_qpos_motors so the closed ankle/waist loops are satisfied).

    Batch and acceptance runs must keep the old behaviour: they score the fall, so
    recovering from it would mean measuring the recovery instead.
    """
    chain = (BRIDGE_ROOT / "tools" / "run_live_chain.py").read_text(encoding="utf-8")
    assert 'sim_cmd.append("--reset-on-fall")' in chain
    # it belongs in the viewer branch, next to the batch-once branch it complements
    viewer_branch = chain.index('sim_cmd.append("--reset-on-fall")')
    batch_branch = chain.index('sim_cmd.append("--batch-once")')
    assert abs(viewer_branch - batch_branch) < 600, "the flag must sit in the viewer branch"

    sim = Path("/home/wusichen/a3_teleop_ws/sonic_for_a3/gear_sonic/scripts/sim2sim_a3_mujoco.py")
    if not sim.is_file():
        pytest.skip("sonic_for_a3 checkout not present")
    source = sim.read_text(encoding="utf-8")
    assert '"--reset-on-fall"' in source
    assert "reset_on_fall: bool = False" in source, "batch runs must default to off"
    assert "def _recover_from_fall(" in source
    assert "fill_loop_qpos_motors(qpos, self.runtime, self.solver, dof)" in source
    # the closed-loop joints have to be made consistent, not written raw
    assert "FALL_RESET_GRACE_STEPS" in source


def test_sim_does_not_reset_the_robot_when_a_stream_is_driving():
    """A viewer session was hard-resetting the robot every 5.0 seconds.

    sim2sim advanced the CSV playlist's frame counter and, on reaching
    reference.num_frames (249 for m5_stand at 50 Hz), wrapped it to 0 and called
    _reset_to_current_reference -- teleporting the robot back to the standing
    pose every 5 s for the whole session.  Headless runs never showed it because
    --batch-once takes an earlier branch, so the acceptance suite was green while
    a viewer session was unusable.  With a live stream there is no end-of-motion.
    """
    sim = (
        Path("/home/wusichen/a3_teleop_ws/sonic_for_a3/gear_sonic/scripts/sim2sim_a3_mujoco.py")
    )
    if not sim.is_file():
        pytest.skip("sonic_for_a3 checkout not present")
    source = sim.read_text(encoding="utf-8")
    assert 'if getattr(self.config, "reference_source", "csv") == "stream":' in source
    assert "self.current_ref_frame = self.reference.num_frames - 1" in source
    # and the wrap+reset must remain unreachable for streams: it follows the guard
    guard = source.index('reference_source", "csv") == "stream"')
    wrap = source.index("self.current_ref_frame = 0", guard)
    assert wrap > guard


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


def test_pico_sim_source_rate_matches_what_the_solver_can_take():
    """The source rate was cut to 30 Hz only because the solver could not keep up.

    With the thread pools fixed the solve runs at p50 16 ms / p99 18 ms, a ~62 Hz
    ceiling, and the live chain measured 50 Hz in / 50 Hz solved with a single
    dropped frame over 34 s -- so the cap belongs back at the sender's own
    default.  A 30 Hz default would now be leaving fidelity on the floor; a 60 Hz
    default would drop ~1% of frames.
    """
    source = (BRIDGE_ROOT / "scripts" / "run_pico_sim.sh").read_text(encoding="utf-8")
    assert "PICO_FPS=50" in source, "the source rate should match the solver's headroom"
    assert "PICO_FPS=30" not in source
    # it has to stay reachable from the command line for slower machines
    assert "--pico-fps" in source
    assert "--target_fps" in source, "the rate must actually reach the sender"

    runbook = (BRIDGE_ROOT / "docs" / "5060_FULL_RUNBOOK.md").read_text(encoding="utf-8")
    assert "17.10" in runbook
    # the runbook has to say that publishing is driven by the solve, not a 50 Hz timer
    assert "发布频率就是求解频率" in runbook


def test_bridge_process_pins_the_math_runtimes_to_one_thread():
    """``torch.set_num_threads`` does not shrink a pool that already exists.

    torch builds its OpenMP pool at *import* time, sized to the whole machine,
    and the spare workers spin between parallel regions.  Capping the thread
    count from inside the session therefore left the pool intact: the bridge ran
    at ~35 threads and, measured on this 16-core box, **1453% CPU** while its
    real work is half a core -- about 14 cores of barrier waits, taken from the
    policy running beside it.  Naming the count in the environment builds the
    pool with one worker instead:

        threads   35 -> 5      bridge CPU   1453% -> 50%
        live rate 26.5 Hz -> 30.0 Hz (drops 135 -> 0, i.e. input-limited)

    Only the bridge is affected; the simulator keeps its own defaults because
    the policy there is a large enough workload to benefit from threads.
    """
    module = _live_chain_module()
    env = {"PATH": "/usr/bin", "HOME": "/tmp"}  # start from a clean environment
    old = os.environ.copy()
    try:
        os.environ.clear()
        os.environ.update(env)
        os.environ.pop("A3_OMP_THREADS", None)
        built = module.bridge_environment()
        assert built["OMP_NUM_THREADS"] == "1"
        assert built["MKL_NUM_THREADS"] == "1"

        os.environ["A3_OMP_THREADS"] = "4"
        assert module.bridge_environment()["OMP_NUM_THREADS"] == "4"

        for off in ("off", "0", "default"):
            os.environ["A3_OMP_THREADS"] = off
            assert "OMP_NUM_THREADS" not in module.bridge_environment()
    finally:
        os.environ.clear()
        os.environ.update(old)

    source = (BRIDGE_ROOT / "tools" / "run_live_chain.py").read_text(encoding="utf-8")
    assert "env=bridge_environment()" in source, "the env must actually reach the bridge Popen"
    # it must be attached to the bridge Popen only -- the simulator's policy is a
    # real multi-threaded workload and must keep its own thread settings
    bridge_popen = source.split("subprocess.Popen(")
    pinned = [chunk for chunk in bridge_popen if "env=bridge_environment()" in chunk]
    assert len(pinned) == 1, "exactly one Popen (the bridge) may be pinned"
    assert "live_cmd" in pinned[0], "the pinned Popen must be the bridge, not the simulator"
    runbook = (BRIDGE_ROOT / "docs" / "5060_FULL_RUNBOOK.md").read_text(encoding="utf-8")
    assert "A3_OMP_THREADS" in runbook


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
    # must be byte-compatible with pico_pose_zmq_minimal.py
    for field in ('"smpl_pose"', '"smpl_joints"', '"body_quat_w"'):
        assert field in source
    assert '"root_translation"' in source
    assert "PACKED_HEADER_SIZE = 1280" in source


def test_fake_pico_sender_publishes_the_root_by_default():
    """Omitting the root is not a smaller test, it is a different one.

    The real sender publishes ``root_translation``.  While this tool defaulted to
    leaving it out, the subscriber synthesised a constant standing pelvis, so
    every desk test silently discarded the pelvis motion of a squat or a step --
    which is exactly the failure class the desk tests exist to catch.  It hid a
    live-path fall behind a frozen root.
    """
    source = (BRIDGE_ROOT / "tools" / "fake_pico_sender.py").read_text(encoding="utf-8")
    assert 'dest="with_root"' in source
    assert "default=True" in source, "the root must be published unless --no-root is given"
    assert '"--no-root"' in source
    # the stale "the real sender does not" claim must be gone
    assert "the real sender does not" not in source


def test_walk_following_probe_is_documented_and_correct():
    """"The operator walks but the robot will not move" needs its own measurement.

    The A3-fast observation is joint commands plus a 6D *orientation* difference
    (ENCODER_TERMS); the reference's horizontal position never enters it, so the
    robot cannot see or correct the gap.  On the official 001_walk_front_slow it
    travels 50% of the reference, and scaling the reference 2x makes it worse
    (21%), i.e. forward speed saturates rather than scaling.  Without this probe
    that reads as a bridge or retarget bug.
    """
    tool = BRIDGE_ROOT / "tools" / "check_walk_following.py"
    assert tool.is_file()
    proc = subprocess.run([sys.executable, str(tool), "--help"], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    source = tool.read_text(encoding="utf-8")
    # the verdict must be based on the official set, not on synthetic clips
    assert "a3_data" in source and "DEFAULT_CLIPS" in source
    # and it must be able to show that scaling the reference does not help
    assert "scale_root" in source
    runbook = (BRIDGE_ROOT / "docs" / "5060_FULL_RUNBOOK.md").read_text(encoding="utf-8")
    assert "check_walk_following.py" in runbook
    # the runbook must name the missing observation term, since that is the cause
    assert "ENCODER_TERMS" in runbook
    assert "ENCODER_FRAME_DIM" in runbook


def test_pico_tracker_probe_is_documented_and_runnable():
    """The "legs barely move" limit is on the headset, and it needs its own probe.

    probe_pico_body.py answers "does the SDK see the legs at all"; it does not
    answer "is the amplitude anywhere near a walk".  A real 100 s run produced 70
    leg-lift events -- so the legs *were* tracked -- but at p50 17 deg of knee
    flexion against the 66-79 deg the validated walk references use.  Without a
    probe that measures amplitude, that reads as a pipeline bug when it is not.
    """
    tool = BRIDGE_ROOT / "tools" / "check_pico_trackers.py"
    assert tool.is_file()
    proc = subprocess.run([sys.executable, str(tool), "--help"], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    source = tool.read_text(encoding="utf-8")
    # it must report the tracker count -- 0 explains estimated lower-body tracking
    assert "get_motion_tracker_serial_numbers" in source
    assert "walk-grade" in source
    # and it must clean up after the SDK
    assert "xrt.close()" in source

    runbook = (BRIDGE_ROOT / "docs" / "5060_FULL_RUNBOOK.md").read_text(encoding="utf-8")
    assert "check_pico_trackers.py" in runbook


def test_pico_sim_exposes_the_fake_sender_for_desk_testing():
    source = (BRIDGE_ROOT / "docs" / "5060_FULL_RUNBOOK.md").read_text(encoding="utf-8")
    assert "tools/fake_pico_sender.py" in source


def test_generated_motions_derive_the_root_from_the_pose():
    """A pinned pelvis makes squat/jump/leg-lift untestable by construction.

    ``trans`` used to be ``[0, 0, root_height]`` for every clip -- constant to
    the last digit in ``data/smplx_validation/*.npz`` and in every converted
    recording.  Bending the knees therefore pushed the feet through the floor
    instead of lowering the hips, so the acceptance set could not tell a working
    squat from a broken one, and "the robot can't squat" was unanswerable.
    """
    source = (BRIDGE_ROOT / "tools" / "make_smplx_validation_motions.py").read_text(encoding="utf-8")
    assert "def support_anchored_root(" in source
    assert "support_anchored_root(fk, body, BASE_ROOT_ROTATION, root_height, stance)" in source
    # the old constant assignment must be gone
    assert "trans[:, 2] = root_height" not in source
    assert "root_z_span_m" in source
    # a squat is only a squat if the pelvis descends and the feet stay planted
    assert "SQUAT_KNEE_RAD" in source and "SQUAT_HIP_RAD" in source


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
    # the stance must be added to every clip, not only to `stand` -- check the
    # intent rather than one literal line, since phased clips join the same builder
    assert "motion + stance[None, :, :]" in source
    assert "build_clip(fk, frames, spec) if kind" in source


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


def test_pico_body_probe_separates_headset_from_pipeline():
    """'the legs do not move' needs one probe to split into its two causes."""
    path = BRIDGE_ROOT / "tools" / "probe_pico_body.py"
    assert path.is_file()
    proc = subprocess.run([sys.executable, str(path), "--help"], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    source = path.read_text(encoding="utf-8")
    for marker in ("NO_MOTION", "TRACKING", "NO_BODY_DATA"):
        assert marker in source
    # the XR layout matters: the leg joints are the last block before Root
    assert "LeftUpLeg" in source and "RightToe" in source


def test_live_frame_dump_and_report():
    """One command must produce a source-vs-reference verdict for the live path."""
    report = BRIDGE_ROOT / "tools" / "report_live_dump.py"
    assert report.is_file()
    proc = subprocess.run([sys.executable, str(report), "--help"], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    text = report.read_text(encoding="utf-8")
    for marker in ("SOURCE_STATIC", "REFERENCE_STATIC", "REFERENCE_WEAK", "VERDICT: OK"):
        assert marker in text
    # the degrees->radians conversion is the whole point: without it a working
    # pipeline reads as "0.02x, the reference barely moves"
    assert "np.radians" in text

    chain = (BRIDGE_ROOT / "tools" / "run_live_chain.py").read_text(encoding="utf-8")
    assert "--dump-frames" in chain and "live_frames.jsonl" in chain
    runner = (BRIDGE_ROOT / "scripts" / "run_pico_sim.sh").read_text(encoding="utf-8")
    assert "report_live_dump.py" in runner and "startup_info.txt" in runner
