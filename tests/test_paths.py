"""Path resolution must be portable across machines (repository portability).

The committed ``generated/a3_contract.json`` used to store the generating
machine's absolute ``sonic_root``; every tool that defaulted to it then failed on
any other checkout.  These tests pin the resolution order and the fallbacks:

    1. explicit CLI argument
    2. environment variable
    3. contract/config value, but only when it exists on this machine
    4. sibling checkout next to this repository
    5. a clear error listing what was tried (never a hard-coded developer path)
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from a3_teleop_bridge import paths
from a3_teleop_bridge.contract import A3Contract, load_contract


def _fake_checkout(root: Path, name: str = "sonic_for_a3") -> Path:
    """Create a directory that looks like a checkout (a marker file is enough)."""
    checkout = root / name
    (checkout / paths.SONIC_MARKER).parent.mkdir(parents=True, exist_ok=True)
    (checkout / paths.SONIC_MARKER).write_text("# fake\n", encoding="utf-8")
    return checkout


def _fake_umr(root: Path) -> Path:
    checkout = root / "UMR"
    (checkout / paths.UMR_MARKER).parent.mkdir(parents=True, exist_ok=True)
    (checkout / paths.UMR_MARKER).write_text("# fake\n", encoding="utf-8")
    return checkout


@pytest.fixture(autouse=True)
def _clean_env(monkeypatch):
    for var in (paths.ENV_SONIC_ROOT, paths.ENV_UMR_ROOT, paths.ENV_WORKSPACE):
        monkeypatch.delenv(var, raising=False)
    yield


# 1 -------------------------------------------------------------------------
def test_cli_argument_wins(tmp_path, monkeypatch) -> None:
    cli = _fake_checkout(tmp_path / "cli")
    env = _fake_checkout(tmp_path / "env")
    monkeypatch.setenv(paths.ENV_SONIC_ROOT, str(env))
    assert paths.resolve_sonic_root(explicit=cli) == cli.resolve()


# 2 -------------------------------------------------------------------------
def test_env_wins_over_stale_contract(tmp_path, monkeypatch) -> None:
    """A contract recorded on another machine must not shadow the env var."""
    env = _fake_checkout(tmp_path / "env")
    monkeypatch.setenv(paths.ENV_SONIC_ROOT, str(env))
    stale = tmp_path / "does_not_exist" / "sonic_for_a3"
    assert paths.resolve_sonic_root(contract_value=stale) == env.resolve()


def test_env_wins_over_sibling(tmp_path, monkeypatch) -> None:
    env = _fake_checkout(tmp_path / "env")
    sibling = _fake_checkout(tmp_path / "ws")
    monkeypatch.setenv(paths.ENV_SONIC_ROOT, str(env))
    monkeypatch.setenv(paths.ENV_WORKSPACE, str(tmp_path / "ws"))
    assert paths.resolve_sonic_root() == env.resolve()
    assert sibling.is_dir()  # the sibling exists but the env var still wins


# 3 -------------------------------------------------------------------------
def test_contract_value_used_when_it_exists(tmp_path) -> None:
    checkout = _fake_checkout(tmp_path / "contract")
    assert paths.resolve_sonic_root(contract_value=checkout) == checkout.resolve()


# 4a ------------------------------------------------------------------------
def test_missing_contract_value_falls_back_to_sibling(tmp_path, monkeypatch) -> None:
    sibling = _fake_checkout(tmp_path)
    monkeypatch.setenv(paths.ENV_WORKSPACE, str(tmp_path))
    assert paths.resolve_sonic_root(contract_value=tmp_path / "gone") == sibling.resolve()


def test_sibling_autodiscovery_uses_repo_parent() -> None:
    """Without any override the resolver finds the real sibling checkout here."""
    resolved = paths.resolve_sonic_root()
    assert (resolved / paths.SONIC_MARKER).is_file()
    assert resolved == paths.workspace_root() / "sonic_for_a3"


def test_umr_sibling_autodiscovery() -> None:
    resolved = paths.resolve_umr_root()
    assert (resolved / paths.UMR_MARKER).is_file()


# 4b ------------------------------------------------------------------------
def test_not_found_raises_with_actionable_message(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv(paths.ENV_WORKSPACE, str(tmp_path / "empty"))
    with pytest.raises(paths.PathResolutionError) as excinfo:
        paths.resolve_sonic_root(contract_value=tmp_path / "nope")
    message = str(excinfo.value)
    assert paths.ENV_SONIC_ROOT in message
    assert "--sonic-root" in message
    assert "tried" in message


def test_no_hardcoded_developer_path_in_sources() -> None:
    """No source file may embed a machine-specific workspace path."""
    # assembled at runtime so this test file does not itself contain the tokens
    forbidden = ("/inspi" + "re/", "wsc-" + "workspace")
    offenders: list[str] = []
    for folder in ("src", "tools", "scripts", "tests", "configs", "generated"):
        for path in (paths.repo_root() / folder).rglob("*"):
            if not path.is_file() or path.suffix not in {".py", ".sh", ".json", ".yaml", ".yml"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            if any(token in text for token in forbidden):
                offenders.append(str(path.relative_to(paths.repo_root())))
    assert offenders == [], f"machine-specific paths leaked into: {offenders}"


# rebasing ------------------------------------------------------------------
def test_asset_paths_rebase_onto_the_resolved_root(tmp_path, monkeypatch) -> None:
    """A contract from another machine resolves its assets here."""
    checkout = _fake_checkout(tmp_path / "sonic")
    mjcf_rel = "gear_sonic/data/assets/robot_description/mjcf/robot.xml"
    (checkout / mjcf_rel).parent.mkdir(parents=True, exist_ok=True)
    (checkout / mjcf_rel).write_text("<mujoco/>\n", encoding="utf-8")

    raw = {
        "schema": "a3_contract/v1",
        "generated_by": "test",
        "sonic_root": "/home/someone/else/ws/sonic_for_a3",  # stale, must be ignored
        "assets": {
            "mjcf": mjcf_rel,  # relative -> rebased
            "urdf": "/home/someone/else/ws/sonic_for_a3/robot.urdf",  # absolute + missing
            "sample_csv": "a3_data/sample.csv",
        },
        "dims": {
            "policy_joint_count": 1,
            "reference_frame_count": 1,
            "window_dt": 0.02,
            "observation_dim": 1,
            "encoder_frame_dim": 1,
            "encoder_input_dim": 1,
            "action_dim": 1,
        },
        "joints": {
            "policy": ["waist_yaw_joint"],
            "csv": ["waist_yaw_joint"],
            "il": ["waist_yaw_joint"],
            "head": ["head_yaw_joint"],
            "passive_foot": ["left_foot_toe_joint"],
        },
        "dims_extra": {},
    }
    contract = A3Contract(raw=raw, path=tmp_path / "contract.json")
    monkeypatch.setenv(paths.ENV_SONIC_ROOT, str(checkout))
    assert contract.sonic_root == checkout.resolve()
    assert contract.mjcf_path == checkout / mjcf_rel
    # a missing absolute asset is rebased onto the resolved root, not silently kept
    assert contract.urdf_path == checkout / "robot.urdf"


def test_committed_contract_has_no_machine_path() -> None:
    raw = json.loads((paths.repo_root() / "generated" / "a3_contract.json").read_text())
    assert raw["sonic_root"] is None, "the committed contract must not record a machine path"
    for key, value in raw["assets"].items():
        assert not Path(value).is_absolute(), f"asset {key} is absolute: {value}"
    contract = load_contract()
    assert contract.mjcf_path.is_file()
    assert contract.urdf_path.is_file()
    assert contract.sample_csv_path.is_file()


# robot XML recorded inside a UMR result --------------------------------------
def test_umr_robot_xml_reanchors_onto_this_checkout(tmp_path, monkeypatch) -> None:
    """A UMR result generated elsewhere must still load here."""
    from a3_teleop_bridge.umr.offline import _resolve_robot_xml, _suffix_after_sonic_root

    checkout = _fake_checkout(tmp_path / "sonic")
    mjcf_dir = checkout / "gear_sonic/data/assets/robot_description/mjcf"
    mjcf_dir.mkdir(parents=True, exist_ok=True)
    floating = mjcf_dir / "clip_smplx_agibot_a3.floating_mjcf.xml"
    floating.write_text("<mujoco/>\n", encoding="utf-8")

    foreign = "/home/other/ws/sonic_for_a3/gear_sonic/data/assets/robot_description/mjcf/clip_smplx_agibot_a3.floating_mjcf.xml"
    assert _suffix_after_sonic_root(Path(foreign)) == Path(
        "gear_sonic/data/assets/robot_description/mjcf/clip_smplx_agibot_a3.floating_mjcf.xml"
    )
    monkeypatch.setenv(paths.ENV_SONIC_ROOT, str(checkout))
    assert _resolve_robot_xml(foreign, tmp_path / "result.npz") == floating

    # a path relative to the result directory still works
    relative = _resolve_robot_xml("clip_smplx_agibot_a3.floating_mjcf.xml", mjcf_dir / "result.npz")
    assert relative == mjcf_dir / "clip_smplx_agibot_a3.floating_mjcf.xml"

    # nothing resolvable -> None (caller raises with an actionable message)
    assert _resolve_robot_xml("/nope/sonic_for_a3/missing.xml", tmp_path / "result.npz") is None
