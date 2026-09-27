"""UMR A3 robot config invariants (plan sections 13-16)."""

from __future__ import annotations

import json

import pytest
import yaml

from a3_teleop_bridge.a3.limits import load_limits
from a3_teleop_bridge.contract import load_contract

BRIDGE_ROOT = __import__("pathlib").Path(__file__).resolve().parents[1]


def find_umr_config():
    import os
    from pathlib import Path

    candidates = []
    if os.environ.get("UMR_ROOT"):
        candidates.append(Path(os.environ["UMR_ROOT"]))
    candidates.append(Path.home() / "a3_teleop_ws" / "UMR")
    candidates.append(BRIDGE_ROOT.parent / "UMR")
    for root in candidates:
        cfg = root / "robot_configs" / "humanoid_retarget_agibot_a3.json"
        if cfg.is_file():
            return cfg
    pytest.skip("UMR A3 robot config not generated yet")


@pytest.fixture(scope="module")
def umr_cfg():
    return json.loads(find_umr_config().read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def tpose_doc():
    return json.loads((BRIDGE_ROOT / "generated" / "a3_tpose.json").read_text(encoding="utf-8"))


def test_robot_block(umr_cfg):
    robot = umr_cfg["robot"]
    assert robot["name"] == "agibot_a3"
    assert robot["xml"].startswith("${SONIC_A3_ROOT}/")
    assert robot["xml"].endswith(".xml")
    assert robot["point_cloud_center"].startswith("body:")
    assert robot["sample_pose"] == "tpose"


def test_tpose_matches_generated_source(umr_cfg, tpose_doc):
    for name, value in umr_cfg["robot"]["tpose_qpos"].items():
        assert value == pytest.approx(tpose_doc["tpose_qpos"][name], abs=1e-9)


def test_tpose_within_limits(umr_cfg):
    limits = load_limits()
    for name, value in umr_cfg["robot"]["tpose_qpos"].items():
        if name not in limits.joint_names:
            continue
        idx = limits.joint_names.index(name)
        assert limits.lower[idx] - 1e-9 <= value <= limits.upper[idx] + 1e-9, name


def test_policy_limits_match_extracted_limits(umr_cfg):
    doc = yaml.safe_load((BRIDGE_ROOT / "generated" / "a3_joint_limits.yaml").read_text(encoding="utf-8"))
    for name in load_contract().policy_joint_names:
        lo, hi = umr_cfg["robot"]["joint_limits"][name]
        assert lo == pytest.approx(float(doc["joints"][name]["position_lower"]), abs=1e-9)
        assert hi == pytest.approx(float(doc["joints"][name]["position_upper"]), abs=1e-9)


def test_locked_joints_are_pinned(umr_cfg):
    contract = load_contract()
    locked = set(umr_cfg["_bridge"]["locked_joints"])
    for name in contract.passive_foot_joint_names:
        assert name in locked, "passive foot joints must be locked"
        assert umr_cfg["robot"]["joint_limits"][name] == [0.0, 0.0]
    for name in locked:
        assert name not in contract.policy_joint_names
        assert umr_cfg["robot"]["joint_limits"][name] == [0.0, 0.0]


def test_head_not_in_policy_limits(umr_cfg):
    contract = load_contract()
    limits = umr_cfg["robot"]["joint_limits"]
    for name in contract.head_joint_names:
        assert name not in contract.policy_joint_names
        assert limits[name] == [0.0, 0.0]


def test_config_records_its_sources(umr_cfg):
    meta = umr_cfg["_bridge"]
    assert meta["generated_by"].endswith("make_umr_a3_config.py")
    assert meta["policy_joint_count"] == 29
    assert set(meta["sources"]) >= {"mjcf", "urdf", "tpose", "joint_limits", "contract"}


def test_tpose_arms_are_horizontal(tpose_doc):
    arms = tpose_doc["arms"]
    for side in ("left", "right"):
        report = arms[side]
        assert report["ok"], f"{side} arm not a valid T-pose"
        assert report["lateral_fraction"] > 0.99
        assert report["vertical_error_m"] < 0.02
        assert report["straightness_deg"] < 3.0


def test_tpose_keeps_passive_and_head_out():
    doc = json.loads((BRIDGE_ROOT / "generated" / "a3_tpose.json").read_text(encoding="utf-8"))
    contract = load_contract()
    for name in contract.passive_foot_joint_names:
        assert name not in doc["tpose_qpos"]


def test_umr_loader_expands_env_vars():
    """The UMR config loader must expand ${VAR} (plan section 14)."""
    import os
    import subprocess
    from pathlib import Path

    umr_root = find_umr_config().parents[1]
    env = dict(os.environ)
    env["SONIC_A3_ROOT"] = str(load_contract().sonic_root)
    script = (
        "import sys; sys.path.insert(0, 'scripts');"
        "from humanoid_retarget_config import load_config, resolve_path, robot_config;"
        "cfg = load_config('robot_configs/humanoid_retarget_agibot_a3.json');"
        "rob = robot_config(cfg);"
        "print(resolve_path(rob['xml'], cfg))"
    )
    python = umr_root / ".venv_umr" / "bin" / "python"
    if not python.is_file():
        pytest.skip("UMR venv not available")
    out = subprocess.run(
        [str(python), "-c", script],
        cwd=str(umr_root),
        env=env,
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert out.returncode == 0, out.stderr
    resolved = Path(out.stdout.strip().splitlines()[-1])
    assert resolved.is_file()
    assert "${" not in str(resolved)
