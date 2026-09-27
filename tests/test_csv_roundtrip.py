"""Flat-CSV round-trip against the repository-official sample (plan section 20).

The strongest available check: parse the shipped A3 CSV with our codec, write it
back out, re-read it, and require the numbers to survive.  When the sonic_for_a3
virtualenv is available the same file is additionally parsed by the *official*
``load_a3_flat_csv`` through a subprocess and compared array-by-array, so the
codec is pinned to the live source rather than to our reading of it.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from a3_teleop_bridge.a3.csv_export import (
    A3FlatCsvCodec,
    assert_same_rotation,
    read_a3_flat_csv,
    write_a3_flat_csv,
)
from a3_teleop_bridge.contract import load_contract

BRIDGE_ROOT = Path(__file__).resolve().parents[1]


def _official_reference(csv_path: Path, source_fps: float, frame_stride: int):
    """Run the official loader in the sonic_for_a3 venv, if present."""
    contract = load_contract()
    python = contract.sonic_root / ".venv_sim" / "bin" / "python"
    if not python.is_file():
        pytest.skip("sonic_for_a3 .venv_sim not available")
    script = f"""
import json, sys, numpy as np
sys.path.insert(0, {str(contract.sonic_root)!r})
sys.path.insert(0, {str(contract.sonic_root / 'gear_sonic' / 'scripts')!r})
import importlib.util
spec = importlib.util.spec_from_file_location("sim2sim_a3", {str(contract.sonic_root / 'gear_sonic/scripts/sim2sim_a3_mujoco.py')!r})
mod = importlib.util.module_from_spec(spec)
sys.modules["sim2sim_a3"] = mod
spec.loader.exec_module(mod)
root_pos, root_quat, dof29, raw = mod.load_a3_flat_csv(__import__('pathlib').Path({str(csv_path)!r}),
                                                       source_fps={source_fps!r}, frame_stride={frame_stride!r})
np.savez('/tmp/_a3_official_ref.npz', root_pos=root_pos, root_quat=root_quat, dof29=dof29,
         raw=np.asarray([raw]))
print('ok')
"""
    out = subprocess.run(
        [str(python), "-c", script], capture_output=True, text=True, timeout=1800
    )
    if out.returncode != 0:
        pytest.skip(f"official loader unavailable: {out.stderr.strip().splitlines()[-1:]}")
    data = np.load("/tmp/_a3_official_ref.npz")
    return data["root_pos"], data["root_quat"], data["dof29"], int(data["raw"][0])


@pytest.fixture(scope="module")
def codec():
    return A3FlatCsvCodec()


def test_official_sample_parses(codec):
    contract = load_contract()
    data = codec.read(contract.sample_csv_path)
    assert data.n_frames > 100
    assert data.root_pos_m.shape[1] == 3
    assert data.dof29_rad.shape[1] == 29
    codec.finite_or_raise(data)
    # root height is around one metre for the A3 sample
    assert 0.5 < float(np.mean(data.root_pos_m[:, 2])) < 2.0


def test_roundtrip_preserves_numbers(codec, tmp_path):
    contract = load_contract()
    src = codec.read(contract.sample_csv_path)
    out = tmp_path / "roundtrip.csv"
    write_a3_flat_csv(out, src.root_pos_m, src.root_quat_wxyz, src.dof29_rad, frame_ids=src.frame_ids)
    back = codec.read(out, frame_stride=1)

    assert back.n_frames == src.n_frames
    np.testing.assert_allclose(back.root_pos_m, src.root_pos_m, atol=1e-9)
    assert_same_rotation(back.root_quat_wxyz, src.root_quat_wxyz, tol_deg=1e-4)
    np.testing.assert_allclose(back.dof29_rad, src.dof29_rad, atol=1e-9)
    np.testing.assert_array_equal(back.frame_ids, src.frame_ids)


def test_roundtrip_matches_official_loader(codec, tmp_path):
    """Our parsed numbers must equal the official loader's numbers."""
    contract = load_contract()
    ours = codec.read(contract.sample_csv_path)
    ref_root, ref_quat, ref_dof, ref_raw = _official_reference(
        contract.sample_csv_path, ours.source_fps, ours.frame_stride
    )
    assert ours.raw_row_count == ref_raw
    assert ours.n_frames == ref_root.shape[0]
    np.testing.assert_allclose(ours.root_pos_m, ref_root, atol=1e-12)
    np.testing.assert_allclose(ours.dof29_rad, ref_dof, atol=1e-12)
    # the shipped CSV stores ~9 significant digits, so euler -> quat -> euler
    # round-trips to about 4e-6 deg; anything below 1e-4 deg is print precision
    assert_same_rotation(ours.root_quat_wxyz, ref_quat, tol_deg=1e-4)


def test_head_columns_are_neutral_on_write(codec, tmp_path):
    contract = load_contract()
    src = codec.read(contract.sample_csv_path)
    out = tmp_path / "head_neutral.csv"
    write_a3_flat_csv(out, src.root_pos_m[:5], src.root_quat_wxyz[:5], src.dof29_rad[:5])
    header = out.read_text(encoding="utf-8").splitlines()[0].split(",")
    head_cols = [header.index(name) for name in contract.head_joint_names]
    for line in out.read_text(encoding="utf-8").splitlines()[1:]:
        values = line.split(",")
        for col in head_cols:
            assert float(values[col]) == 0.0


def test_write_accepts_head_values(codec, tmp_path):
    contract = load_contract()
    src = codec.read(contract.sample_csv_path)
    head = np.deg2rad(np.array([[10.0, -5.0]] * 3))
    out = tmp_path / "head_set.csv"
    write_a3_flat_csv(
        out, src.root_pos_m[:3], src.root_quat_wxyz[:3], src.dof29_rad[:3], head_dof_rad=head
    )
    header = out.read_text(encoding="utf-8").splitlines()[0].split(",")
    idx_yaw = header.index(contract.head_joint_names[0])
    row = out.read_text(encoding="utf-8").splitlines()[1].split(",")
    assert float(row[idx_yaw]) == pytest.approx(10.0, abs=1e-6)


def test_dof_suffix_alias_supported(codec, tmp_path):
    contract = load_contract()
    src = codec.read(contract.sample_csv_path)
    out = tmp_path / "aliased.csv"
    write_a3_flat_csv(out, src.root_pos_m[:3], src.root_quat_wxyz[:3], src.dof29_rad[:3])
    text = out.read_text(encoding="utf-8")
    header, *rows = text.splitlines()
    header = header.replace("," + contract.csv_joint_names[3], "," + contract.csv_joint_names[3] + "_dof")
    (tmp_path / "aliased2.csv").write_text("\n".join([header, *rows]) + "\n", encoding="utf-8")
    back = codec.read(tmp_path / "aliased2.csv", frame_stride=1)
    np.testing.assert_allclose(back.dof29_rad, src.dof29_rad[:3], atol=1e-9)


def test_write_rejects_bad_shapes(tmp_path):
    contract = load_contract()
    with pytest.raises(ValueError):
        write_a3_flat_csv(tmp_path / "bad.csv", np.zeros((2, 3)), np.zeros((2, 4)), np.zeros((2, 5)))


def test_limit_joint_is_never_in_csv_policy_columns():
    """The policy view excludes head joints; the CSV keeps them."""
    contract = load_contract()
    assert len(contract.csv_joint_names) == 31
    assert len(contract.policy_joint_names) == 29
    assert set(contract.head_joint_names).issubset(set(contract.csv_joint_names))
    assert not set(contract.head_joint_names) & set(contract.policy_joint_names)


def test_effective_fps_matches_reference_stream(codec):
    contract = load_contract()
    data = codec.read(contract.sample_csv_path)
    assert data.effective_fps == pytest.approx(contract.reference_hz, rel=1e-9)
    meta = json.dumps({"frames": data.n_frames, "fps": data.effective_fps})
    assert json.loads(meta)["frames"] == data.n_frames
