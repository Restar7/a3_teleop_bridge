#!/usr/bin/env python3
"""Inspect the *current* sonic_for_a3 sources and emit the A3 policy contract.

Plan section 7 / 18: never guess joint counts, dt, window length, obs dim or
joint order from documentation.  Everything below is extracted from:

  * ``gear_sonic/scripts/sim2sim_a3_mujoco.py``   (python constants, AST-parsed)
  * ``gear_sonic_deploy/.../a3_obs_builder.hpp``  (C++ obs_dict layout)
  * ``a3_data/agibot_a3/001_walk_front_slow.csv`` (official CSV header)

Output: ``generated/a3_contract.json`` — downstream code must read this file
instead of copying magic numbers.

Usage:
    python tools/inspect_a3_contract.py [--sonic-root PATH] [--out PATH]
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = BRIDGE_ROOT / "generated" / "a3_contract.json"

SIM2SIM_REL = "gear_sonic/scripts/sim2sim_a3_mujoco.py"
OBS_BUILDER_REL = (
    "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_deploy/a3_obs_builder.hpp"
)
SAMPLE_CSV_REL = "a3_data/agibot_a3/001_walk_front_slow.csv"

# Names we require from the sim2sim module.  Missing name => hard failure.
REQUIRED_PY_CONSTANTS = (
    "NUM_POLICY_DOFS",
    "NUM_FUTURE_FRAMES",
    "TARGET_FPS",
    "POLICY_DT",
    "FUTURE_FRAME_SKIP",
    "ENCODER_FRAME_DIM",
    "ENCODER_INPUT_DIM",
    "OBS_TERM_ORDER",
    "OBS_TERM_DIMS",
    "A3_CSV_JOINT_NAMES",
    "A3_POLICY_TO_SDK_IDX",
    "HEAD_JOINTS",
    "PASSIVE_FOOT_JOINT_NAMES",
    "DEFAULT_CSV_SOURCE_FPS",
    "DEFAULT_CSV_FRAME_STRIDE",
    "DEFAULT_REFERENCE_ON_END",
    "DEFAULT_LOOP_MJCF",
    "DEFAULT_URDF",
    "ENCODER_MODE_PRESETS",
    "FOOT_SPRING_JOINT_NAMES",
)


def find_sonic_root(explicit: str | None = None) -> Path:
    """Locate the sonic_for_a3 checkout (env var, then well-known paths)."""
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    if os.environ.get("SONIC_A3_ROOT"):
        candidates.append(Path(os.environ["SONIC_A3_ROOT"]))
    candidates.append(Path.home() / "a3_teleop_ws" / "sonic_for_a3")
    candidates.append(BRIDGE_ROOT.parent / "sonic_for_a3")
    for cand in candidates:
        if (cand / SIM2SIM_REL).is_file():
            return cand.expanduser().resolve()
    raise SystemExit(
        "Could not find sonic_for_a3. Set SONIC_A3_ROOT or pass --sonic-root.\n"
        f"Tried: {[str(c) for c in candidates]}"
    )


class _UnsupportedExpression(Exception):
    """Raised when a module-level expression cannot be evaluated statically."""


_BIN_OPS = {
    ast.Add: lambda a, b: a + b,
    ast.Sub: lambda a, b: a - b,
    ast.Mult: lambda a, b: a * b,
    ast.Div: lambda a, b: a / b,
    ast.FloorDiv: lambda a, b: a // b,
    ast.Mod: lambda a, b: a % b,
}


def _eval_node(node: ast.AST, env: dict[str, object]) -> object:
    """Evaluate a small, side-effect-free subset of Python expressions."""
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Tuple):
        return tuple(_eval_node(e, env) for e in node.elts)
    if isinstance(node, ast.List):
        return [_eval_node(e, env) for e in node.elts]
    if isinstance(node, ast.Dict):
        return {
            _eval_node(k, env): _eval_node(v, env)
            for k, v in zip(node.keys, node.values)
            if k is not None
        }
    if isinstance(node, ast.Name):
        if node.id in env:
            return env[node.id]
        raise _UnsupportedExpression(node.id)
    if isinstance(node, ast.UnaryOp):
        operand = _eval_node(node.operand, env)
        if isinstance(node.op, ast.USub):
            return -operand  # type: ignore[operator]
        if isinstance(node.op, ast.UAdd):
            return +operand  # type: ignore[operator]
        if isinstance(node.op, ast.Not):
            return not operand
        raise _UnsupportedExpression("unary op")
    if isinstance(node, ast.BinOp):
        op = _BIN_OPS.get(type(node.op))
        if op is None:
            raise _UnsupportedExpression("binary op")
        return op(_eval_node(node.left, env), _eval_node(node.right, env))
    if isinstance(node, ast.Call):
        func = node.func
        if isinstance(func, ast.Name) and func.id in env and callable(env[func.id]):
            callable_obj = env[func.id]
            args = [_eval_node(a, env) for a in node.args]
            return callable_obj(*args)
        raise _UnsupportedExpression("call")
    if isinstance(node, ast.JoinedStr):
        parts: list[str] = []
        for value in node.values:
            if isinstance(value, ast.FormattedValue):
                parts.append(str(_eval_node(value.value, env)))
            else:
                parts.append(str(_eval_node(value, env)))
        return "".join(parts)
    raise _UnsupportedExpression(type(node).__name__)


def parse_py_constants(path: Path) -> dict[str, object]:
    """Extract module-level constants, resolving simple derived expressions.

    ``REPO_ROOT / "gear_sonic/..."``, ``1.0 / TARGET_FPS`` and
    ``NUM_POLICY_DOFS * 2 + 6`` are all resolved, so the contract always comes
    from the live source rather than from a duplicated table.
    """
    text = path.read_text(encoding="utf-8")
    tree = ast.parse(text, filename=str(path))
    repo_root = path.resolve().parents[2]

    env: dict[str, object] = {
        "Path": Path,
        "REPO_ROOT": repo_root,
        "__file__": str(path),
        "os": os,
        "environ": os.environ,
    }
    for node in tree.body:
        targets: list[ast.expr] = []
        value_node: ast.expr | None = None
        if isinstance(node, ast.Assign):
            targets = list(node.targets)
            value_node = node.value
        elif isinstance(node, ast.AnnAssign) and node.value is not None:
            targets = [node.target]
            value_node = node.value
        if value_node is None:
            continue
        try:
            value = _eval_node(value_node, env)
        except Exception:
            continue
        for target in targets:
            if isinstance(target, ast.Name):
                env[target.id] = value
    return env


_CPP_INT_RE = re.compile(
    r"inline\s+constexpr\s+std::size_t\s+(?P<name>\w+)\s*=\s*(?P<expr>[^;]+);"
)


def _eval_cpp_int(expr: str, known: dict[str, int]) -> int | None:
    """Evaluate a C++ constexpr integer expression using already-known names."""
    expr = expr.strip()
    expr = re.sub(r"//.*", "", expr)
    for name, value in sorted(known.items(), key=lambda kv: -len(kv[0])):
        expr = re.sub(rf"\b{re.escape(name)}\b", str(value), expr)
    if not re.fullmatch(r"[0-9+\-*/() ]+", expr):
        return None
    try:
        return int(eval(expr, {"__builtins__": {}}, {}))  # noqa: S307 - sanitised above
    except Exception:  # pragma: no cover - defensive
        return None


def parse_cpp_constants(path: Path) -> dict[str, int]:
    text = path.read_text(encoding="utf-8")
    known: dict[str, int] = {}
    for match in _CPP_INT_RE.finditer(text):
        value = _eval_cpp_int(match.group("expr"), known)
        if value is not None:
            known[match.group("name")] = value
    return known


def parse_urdf_actuated_joint_order(path: Path, passive_joint_names: set[str]) -> list[str]:
    """Encoder joint order (``dof_il``) as SONIC derives it from the URDF.

    ``sim2sim_a3_mujoco.load_urdf_actuated_joints`` walks the URDF kinematic tree
    breadth-first, visiting each link's children sorted by *child link name*, and
    keeps non-fixed joints that are not passive foot hinges.  This reproduces that
    exactly -- the encoder order is a real permutation of the CSV/MJCF order, and
    getting it wrong silently destroys the policy input (verified in M6).
    """
    root = ET.parse(path).getroot()
    all_links = [elem.attrib["name"] for elem in root.findall("link")]
    child_links: set[str] = set()
    parent_to_joints: dict[str, list[tuple[str, str, str]]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        child_links.add(child)
        parent_to_joints.setdefault(parent, []).append(
            (joint.attrib["name"], joint.attrib.get("type", ""), child)
        )

    roots = [link for link in all_links if link not in child_links]
    if len(roots) != 1:
        raise SystemExit(f"{path}: expected exactly one URDF root link, got {roots}")

    actuated: list[str] = []
    queue = [roots[0]]
    while queue:
        link = queue.pop(0)
        for joint_name, joint_type, child in sorted(
            parent_to_joints.get(link, []), key=lambda item: item[2].lower()
        ):
            if joint_type != "fixed" and joint_name not in passive_joint_names:
                actuated.append(joint_name)
            queue.append(child)
    return actuated


def parse_csv_header(path: Path) -> list[str]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        header = handle.readline().strip()
    return header.split(",")


def build_contract(sonic_root: Path) -> dict:
    sim2sim = sonic_root / SIM2SIM_REL
    consts = parse_py_constants(sim2sim)
    missing = [name for name in REQUIRED_PY_CONSTANTS if name not in consts]
    if missing:
        raise SystemExit(f"{sim2sim}: missing expected constants: {missing}")

    cpp = parse_cpp_constants(sonic_root / OBS_BUILDER_REL)

    num_policy_dofs = int(consts["NUM_POLICY_DOFS"])
    num_future_frames = int(consts["NUM_FUTURE_FRAMES"])
    target_fps = float(consts["TARGET_FPS"])
    policy_dt = float(consts["POLICY_DT"])
    encoder_frame_dim = int(consts["ENCODER_FRAME_DIM"])
    encoder_input_dim = int(consts["ENCODER_INPUT_DIM"])

    csv_joint_names = list(consts["A3_CSV_JOINT_NAMES"])
    policy_to_sdk = list(consts["A3_POLICY_TO_SDK_IDX"])
    head_joints = list(consts["HEAD_JOINTS"])
    passive_foot_joints = list(consts["PASSIVE_FOOT_JOINT_NAMES"])

    policy_joint_names = [csv_joint_names[i] for i in policy_to_sdk]
    excluded_joint_names = [n for n in csv_joint_names if n not in policy_joint_names]

    # --- csv / root conventions -------------------------------------------
    sample_csv_header = parse_csv_header(sonic_root / SAMPLE_CSV_REL)
    root_columns = sample_csv_header[1:7]
    csv_joint_columns = sample_csv_header[7:]

    # --- encoder (IsaacLab/URDF) joint order ------------------------------
    urdf_path = sonic_root / consts["DEFAULT_URDF"]
    il_joint_names = parse_urdf_actuated_joint_order(urdf_path, set(passive_foot_joints))
    policy_to_il = [il_joint_names.index(n) for n in policy_joint_names]
    il_to_policy = [policy_joint_names.index(n) for n in il_joint_names]

    presets = consts["ENCODER_MODE_PRESETS"]
    a3_fast_preset = list(presets["a3_fast"])

    obs_dim = cpp.get("kA3ObsDictTotalFloats")
    smpl_obs_dim = cpp.get("kA3SmplObsDictTotalFloats")
    proprio_total = cpp.get("kA3ProprioTotalFloats")
    tokenizer_total = cpp.get("kA3TokenizerTotalFloats")

    contract = {
        "schema": "a3_contract/v1",
        "generated_by": "tools/inspect_a3_contract.py",
        "sonic_root": str(sonic_root),
        "sources": {
            "sim2sim": SIM2SIM_REL,
            "obs_builder": OBS_BUILDER_REL,
            "sample_csv": SAMPLE_CSV_REL,
        },
        "policy_joint_names": policy_joint_names,
        "policy_joint_count": num_policy_dofs,
        "csv_joint_names": csv_joint_names,
        "csv_joint_count": len(csv_joint_names),
        "policy_to_csv_index": policy_to_sdk,
        "il_joint_names": il_joint_names,
        "policy_to_il_index": policy_to_il,
        "il_to_policy_index": il_to_policy,
        "excluded_joint_names": excluded_joint_names,
        "head_joint_names": head_joints,
        "passive_foot_joint_names": passive_foot_joints,
        "foot_spring_joint_names": list(consts["FOOT_SPRING_JOINT_NAMES"]),
        "csv_columns": {
            "frame": sample_csv_header[0],
            "root": root_columns,
            "joints": csv_joint_columns,
            "total_columns": len(sample_csv_header),
        },
        "timing": {
            "policy_hz": target_fps,
            "policy_dt": policy_dt,
            "csv_source_fps": float(consts["DEFAULT_CSV_SOURCE_FPS"]),
            "csv_frame_stride": int(consts["DEFAULT_CSV_FRAME_STRIDE"]),
            "reference_stream_hz": float(consts["DEFAULT_CSV_SOURCE_FPS"])
            / float(consts["DEFAULT_CSV_FRAME_STRIDE"]),
        },
        "reference_window": {
            "frame_count": num_future_frames,
            "frame_skip": int(consts["FUTURE_FRAME_SKIP"]),
            "dt": policy_dt,
            "a3_fast_preset": {
                "encoder": a3_fast_preset[0],
                "frame_skip": int(a3_fast_preset[1]),
                "history_frames": int(a3_fast_preset[2]),
                "valid_future_frames": a3_fast_preset[3],
                "zero_pad_invalid_frames": bool(a3_fast_preset[4]),
            },
            "future_horizon_s": policy_dt
            * int(a3_fast_preset[1])
            * (num_future_frames - 1),
            "on_end_default": consts["DEFAULT_REFERENCE_ON_END"],
        },
        "dims": {
            "action_dim": num_policy_dofs,
            "encoder_frame_dim": encoder_frame_dim,
            "encoder_input_dim": encoder_input_dim,
            "observation_dim": obs_dim,
            "observation_dim_smpl": smpl_obs_dim,
            "proprio_history_dim": proprio_total,
            "tokenizer_dim": tokenizer_total,
            "obs_term_order": list(consts["OBS_TERM_ORDER"]),
            "obs_term_dims": dict(consts["OBS_TERM_DIMS"]),
        },
        "assets": {
            "mjcf": str(sonic_root / consts["DEFAULT_LOOP_MJCF"]),
            "mjcf_rel": str(consts["DEFAULT_LOOP_MJCF"]),
            "urdf": str(sonic_root / consts["DEFAULT_URDF"]),
            "urdf_rel": str(consts["DEFAULT_URDF"]),
            "sample_csv": str(sonic_root / SAMPLE_CSV_REL),
        },
    }
    return contract


def assert_contract(contract: dict) -> list[str]:
    """Assert the cross-checks required by plan sections 7 and 18."""
    checks: list[str] = []

    def check(name: str, condition: bool, detail: str) -> None:
        if not condition:
            raise SystemExit(f"CONTRACT ASSERTION FAILED [{name}]: {detail}")
        checks.append(f"{name}: {detail}")

    names = contract["policy_joint_names"]
    check("joint_count", len(names) == 29 and contract["policy_joint_count"] == 29,
          f"{len(names)} policy joints")
    check("joint_unique", len(set(names)) == 29, "all policy joint names unique")

    head = set(contract["head_joint_names"])
    passive = set(contract["passive_foot_joint_names"])
    check("no_head_in_policy", not (head & set(names)), f"head joints excluded: {sorted(head)}")
    check("no_passive_in_policy", not (passive & set(names)),
          f"passive joints excluded: {sorted(passive)}")

    csv_names = contract["csv_joint_names"]
    check("csv_minus_excluded",
          [n for n in csv_names if n not in head] == names,
          "csv joints minus head joints == policy joints (in order)")

    idx = contract["policy_to_csv_index"]
    check("policy_to_csv_index", len(idx) == 29 and sorted(idx) == sorted(
        i for i, n in enumerate(csv_names) if n not in head),
        "policy_to_csv_index selects exactly the non-head csv columns")

    dims = contract["dims"]
    check("encoder_frame_dim", dims["encoder_frame_dim"] == 29 * 2 + 6,
          f"encoder frame = 29 q + 29 dq + 6d ori = {dims['encoder_frame_dim']}")
    check("encoder_input_dim",
          dims["encoder_input_dim"] == contract["reference_window"]["frame_count"] * dims["encoder_frame_dim"],
          f"encoder input = 10 x {dims['encoder_frame_dim']} = {dims['encoder_input_dim']}")
    check("obs_dim", dims["observation_dim"] == 1570,
          f"policy obs_dict dim = {dims['observation_dim']} (tokenizer {dims['tokenizer_dim']} + proprio {dims['proprio_history_dim']})")
    check("action_dim", dims["action_dim"] == 29, f"action dim = {dims['action_dim']}")

    timing = contract["timing"]
    check("policy_dt", abs(timing["policy_dt"] - 0.02) < 1e-12,
          f"policy dt = {timing['policy_dt']}")
    check("policy_hz", abs(timing["policy_hz"] - 50.0) < 1e-9,
          f"policy rate = {timing['policy_hz']} Hz")

    win = contract["reference_window"]
    check("window_frames", win["frame_count"] == 10, f"{win['frame_count']} frames")
    check("window_dt_a3_fast",
          abs(win["a3_fast_preset"]["frame_skip"] * timing["policy_dt"] - 0.02) < 1e-12,
          "a3_fast frame spacing = 0.02 s")
    check("future_horizon", abs(win["future_horizon_s"] - 0.18) < 1e-9,
          f"future horizon = {win['future_horizon_s']} s")

    header = contract["csv_columns"]
    check("csv_columns", header["total_columns"] == 38,
          f"{header['total_columns']} csv columns (frame + 6 root + 31 joints)")
    check("csv_joint_order",
          header["joints"] == csv_names,
          "csv joint column order matches A3_CSV_JOINT_NAMES")
    check("root_columns", header["root"] == [
        "root_translateX", "root_translateY", "root_translateZ",
        "root_rotateX", "root_rotateY", "root_rotateZ",
    ], "root columns are translateXYZ + rotateXYZ")

    il_names = contract["il_joint_names"]
    check("il_joint_count", len(il_names) == 29 and len(set(il_names)) == 29,
          f"{len(il_names)} unique encoder (URDF) joints")
    p2i = contract["policy_to_il_index"]
    i2p = contract["il_to_policy_index"]
    check("policy_to_il_permutation",
          sorted(p2i) == list(range(29)) and sorted(i2p) == list(range(29)),
          "policy<->encoder index maps are permutations")
    check("il_roundtrip",
          all(i2p[p2i[i]] == i for i in range(29)),
          "policy -> il -> policy is the identity")
    check("il_names_match_policy",
          sorted(il_names) == sorted(contract["policy_joint_names"]),
          "encoder joint names are the same set as the policy joints")
    check("il_order_differs_from_policy",
          il_names != list(contract["policy_joint_names"]),
          "encoder order is a real permutation of the CSV/MJCF order")

    mjcf = Path(contract["assets"]["mjcf"])
    check("mjcf_exists", mjcf.is_file(), f"MJCF present: {mjcf}")
    urdf = Path(contract["assets"]["urdf"])
    check("urdf_exists", urdf.is_file(), f"URDF present: {urdf}")
    sample = Path(contract["assets"]["sample_csv"])
    check("sample_csv_exists", sample.is_file(), f"official sample CSV present: {sample}")
    return checks


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sonic-root", default=None, help="path to sonic_for_a3 checkout")
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="contract JSON output path")
    args = parser.parse_args(argv)

    sonic_root = find_sonic_root(args.sonic_root)
    contract = build_contract(sonic_root)
    checks = assert_contract(contract)

    out_path = Path(args.out).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(contract, indent=2, sort_keys=False) + "\n", encoding="utf-8")

    print(f"sonic_root            : {contract['sonic_root']}")
    print(f"policy_joint_names    : {len(contract['policy_joint_names'])} joints")
    print(f"  {contract['policy_joint_names']}")
    print(f"excluded joints       : {contract['excluded_joint_names']}")
    print(f"passive foot joints   : {contract['passive_foot_joint_names']}")
    print(f"reference dt          : {contract['reference_window']['dt']} s "
          f"({contract['timing']['policy_hz']} Hz)")
    print(f"reference frames      : {contract['reference_window']['frame_count']} "
          f"(skip {contract['reference_window']['a3_fast_preset']['frame_skip']})")
    print(f"future horizon        : {contract['reference_window']['future_horizon_s']} s")
    print(f"observation_dim       : {contract['dims']['observation_dim']}")
    print(f"action_dim            : {contract['dims']['action_dim']}")
    print(f"mjcf                  : {contract['assets']['mjcf']}")
    print("")
    for line in checks:
        print(f"  [ok] {line}")
    print(f"\nwrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
