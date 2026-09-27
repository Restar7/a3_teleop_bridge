"""Regression: the ReferenceProvider abstraction must not change CSV behaviour.

Plan section 40: the same input CSV must produce byte-identical encoder
observations before and after the provider refactor.  This test runs inside the
sonic_for_a3 virtualenv (where ``sim2sim_a3_mujoco`` imports) and compares

    build_tokenizer_terms(reference, frame, ...)                 # legacy path
    build_tokenizer_terms(reference, frame, ..., provider)       # provider path

for many frames of the shipped sample CSV, plus the A3 the encoder input itself.
"""

from __future__ import annotations

import json
import subprocess
import textwrap
from pathlib import Path

import pytest

from a3_teleop_bridge.contract import load_contract

BRIDGE_ROOT = Path(__file__).resolve().parents[1]


def _sonic_python() -> Path:
    python = load_contract().sonic_root / ".venv_sim" / "bin" / "python"
    if not python.is_file():
        pytest.skip("sonic_for_a3 .venv_sim not available")
    return python


def test_csv_provider_matches_legacy_path(tmp_path):
    contract = load_contract()
    python = _sonic_python()
    out_json = tmp_path / "obs_compare.json"
    script = textwrap.dedent(
        f"""
        import importlib.util, json, sys
        import numpy as np

        ROOT = {str(contract.sonic_root)!r}
        sys.path.insert(0, ROOT)
        spec = importlib.util.spec_from_file_location(
            "sim2sim_a3", ROOT + "/gear_sonic/scripts/sim2sim_a3_mujoco.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules["sim2sim_a3"] = mod
        spec.loader.exec_module(mod)

        from pathlib import Path
        csv_path = Path(ROOT) / "a3_data/agibot_a3/001_walk_front_slow.csv"
        # The tokenizer only reads num_frames / anchor_quat_wxyz / dof_il /
        # dof_vel_il, so the real CSV columns are loaded with the official loader
        # and the remaining MotionReference fields are filled with placeholders.
        root_pos, root_quat, dof29, _raw = mod.load_a3_flat_csv(
            csv_path, source_fps=30.0, frame_stride=4)
        dof_vel = np.zeros_like(dof29)
        if dof29.shape[0] > 1:
            dof_vel[1:] = (dof29[1:] - dof29[:-1]) * 30.0
        reference = mod.MotionReference(
            path=csv_path, fps=30.0,
            qpos=np.zeros((dof29.shape[0], 1)), qvel=np.zeros((dof29.shape[0], 1)),
            dof_full=dof29, dof_mj29=dof29, dof_il=dof29, dof_vel_il=dof_vel,
            root_pos=root_pos, root_quat_wxyz=root_quat, anchor_quat_wxyz=root_quat)

        provider = mod.CsvReferenceProvider(reference)
        rng = np.random.default_rng(0)
        worst_input = 0.0
        worst_terms = 0.0
        frames = list(range(0, min(600, reference.num_frames), 7))
        for frame in frames:
            anchor = rng.normal(size=4); anchor /= np.linalg.norm(anchor)
            legacy = mod.build_encoder_input(
                reference, frame, anchor, mod.DEFAULT_REFERENCE_ON_END,
                mod.FUTURE_FRAME_SKIP, 0, None, False)
            via_provider = mod.build_encoder_input(
                reference, frame, anchor, mod.DEFAULT_REFERENCE_ON_END,
                mod.FUTURE_FRAME_SKIP, 0, None, False, provider)
            legacy_terms = mod.build_tokenizer_terms(
                reference, frame, anchor, mod.DEFAULT_REFERENCE_ON_END,
                mod.FUTURE_FRAME_SKIP, 0, None, False)
            provider_terms = mod.build_tokenizer_terms(
                reference, frame, anchor, mod.DEFAULT_REFERENCE_ON_END,
                mod.FUTURE_FRAME_SKIP, 0, None, False, provider)
            worst_input = max(worst_input, float(np.max(np.abs(legacy - via_provider))))
            worst_terms = max(
                worst_terms,
                float(np.max(np.abs(legacy_terms[0] - provider_terms[0]))),
                float(np.max(np.abs(legacy_terms[1] - provider_terms[1]))),
            )
        json.dump(
            {{"frames": len(frames), "worst_encoder_input": worst_input,
              "worst_tokenizer_terms": worst_terms,
              "encoder_input_shape": list(legacy.shape)}},
            open({str(out_json)!r}, "w"),
        )
        print("ok")
        """
    )
    proc = subprocess.run([str(python), "-c", script], capture_output=True, text=True, timeout=1800)
    if proc.returncode != 0:
        pytest.fail(f"sonic-side comparison failed:\n{proc.stdout[-3000:]}\n{proc.stderr[-3000:]}")

    report = json.loads(out_json.read_text(encoding="utf-8"))
    assert report["encoder_input_shape"] == [10, 64]
    assert report["worst_encoder_input"] == 0.0, report
    assert report["worst_tokenizer_terms"] == 0.0, report


def test_streaming_provider_decodes_bridge_packet(tmp_path):
    """The sonic-side decoder must read packets produced by the bridge encoder."""
    contract = load_contract()
    python = _sonic_python()

    # build a packet with the bridge encoder (this interpreter)
    import numpy as np

    from a3_teleop_bridge.types import A3ReferenceWindow
    from a3_teleop_bridge.transport.protocol import encode_packet

    frames = contract.window_frames
    n = contract.n_policy_joints
    window = A3ReferenceWindow(
        seq=42,
        timestamp_ns=1_700_000_000_000_000_000,
        dt=contract.window_dt,
        root_pos_m=np.linspace(0, 0.2, frames * 3).reshape(frames, 3),
        root_quat_wxyz=np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (frames, 1)),
        joint_pos_rad=np.linspace(-0.3, 0.3, frames * n).reshape(frames, n),
        joint_vel_rad_s=np.zeros((frames, n)),
        source_age_ms=4.5,
        solver_latency_ms=6.25,
        valid=True,
    )
    packet_path = tmp_path / "packet.bin"
    packet_path.write_bytes(encode_packet(window, contract))

    script = textwrap.dedent(
        f"""
        import importlib.util, json, sys
        import numpy as np
        ROOT = {str(contract.sonic_root)!r}
        sys.path.insert(0, ROOT)
        spec = importlib.util.spec_from_file_location(
            "sim2sim_a3", ROOT + "/gear_sonic/scripts/sim2sim_a3_mujoco.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules["sim2sim_a3"] = mod
        spec.loader.exec_module(mod)
        from gear_sonic.utils.reference_provider import decode_reference_packet
        packet = open({str(packet_path)!r}, "rb").read()
        w = decode_reference_packet(packet)
        json.dump({{
            "seq": w.seq,
            "valid": w.valid,
            "source_age_ms": w.source_age_ms,
            "anchor_quat_shape": list(w.anchor_quat_wxyz.shape),
            "dof_il_shape": list(w.dof_il.shape),
            "dof_vel_shape": list(w.dof_vel_il.shape),
            "joint_pos_first": float(w.dof_il[0, 0]),
            "joint_pos_last": float(w.dof_il[-1, -1]),
            "root_pos_last": [float(v) for v in w.root_pos_m[-1]],
        }}, open({str(tmp_path / "decoded.json")!r}, "w"))
        print("ok")
        """
    )
    proc = subprocess.run([str(python), "-c", script], capture_output=True, text=True, timeout=900)
    if proc.returncode != 0:
        pytest.fail(f"streaming decode failed:\n{proc.stdout[-2000:]}\n{proc.stderr[-2000:]}")

    decoded = json.loads((tmp_path / "decoded.json").read_text(encoding="utf-8"))
    assert decoded["seq"] == 42
    assert decoded["valid"] is True
    assert decoded["source_age_ms"] == pytest.approx(4.5)
    assert decoded["anchor_quat_shape"] == [10, 4]
    assert decoded["dof_il_shape"] == [10, 29]
    # the wire carries ENCODER (il) order, so the first wire joint is the policy
    # joint named by il_joint_names[0]
    expected_first = float(window.joint_pos_rad[0, contract.il_to_policy_index[0]])
    expected_last = float(window.joint_pos_rad[-1, contract.il_to_policy_index[-1]])
    assert decoded["joint_pos_first"] == pytest.approx(expected_first, abs=1e-6)
    assert decoded["joint_pos_last"] == pytest.approx(expected_last, abs=1e-6)
    np.testing.assert_allclose(decoded["root_pos_last"], window.root_pos_m[-1], atol=1e-6)
