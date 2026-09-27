#!/usr/bin/env python3
"""Extract A3 joint limits and gains from the live sonic_for_a3 sources.

Sources (plan section 16 -- never fill these in from experience):

  1. sim2sim MJCF (position range, damping, frictionloss, armature,
     actuatorfrcrange, stiffness/springref for the passive foot hinges)
  2. training/deploy URDF (lower/upper + effort + velocity, the only place
     joint *velocity* limits exist)
  3. ``a3_policy_parameters.hpp`` (deploy-authoritative default angles, Kp, Kd,
     action scale -- declared in the 29-DOF MuJoCo policy view)

Output: ``generated/a3_joint_limits.yaml`` in contract policy-joint order.

Usage:
    python tools/extract_a3_joint_limits.py [--sonic-root PATH] [--out PATH]
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = BRIDGE_ROOT / "generated" / "a3_joint_limits.yaml"

POLICY_PARAMS_REL = "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_policy_parameters.hpp"

# MJCF rounds joint ranges to ~6 significant digits; anything beyond this is a
# genuine asset mismatch rather than formatting.
RANGE_TOLERANCE = 1e-4


# --------------------------------------------------------------------------
# MJCF
# --------------------------------------------------------------------------
def _strip_ns(tag: str) -> str:
    return tag.split("}", 1)[1] if "}" in tag else tag


def _expand_mjcf(path: Path, seen: set[Path] | None = None) -> list[ET.Element]:
    """Return all <joint> elements, following <include file="..."> recursively."""
    seen = seen or set()
    resolved = path.resolve()
    if resolved in seen:
        return []
    seen.add(resolved)
    tree = ET.parse(resolved)
    root = tree.getroot()

    joints: list[ET.Element] = []
    for elem in root.iter():
        if _strip_ns(elem.tag) == "include":
            inc = elem.get("file")
            if inc:
                joints.extend(_expand_mjcf(resolved.parent / inc, seen))
    for elem in root.iter():
        if _strip_ns(elem.tag) == "joint":
            joints.append(elem)
    return joints


def _floats(text: str | None) -> list[float] | None:
    if text is None:
        return None
    return [float(v) for v in text.replace(",", " ").split()]


def parse_mjcf(path: Path) -> dict[str, dict]:
    joints: dict[str, dict] = {}
    for elem in _expand_mjcf(path):
        name = elem.get("name")
        if not name:
            continue
        info: dict = {"type": elem.get("type", "hinge")}
        rng = _floats(elem.get("range"))
        if rng and len(rng) == 2:
            info["range"] = rng
        axis = _floats(elem.get("axis"))
        if axis:
            info["axis"] = axis
        frc = _floats(elem.get("actuatorfrcrange"))
        if frc and len(frc) == 2:
            info["actuator_frc_range"] = frc
        for attr in ("damping", "frictionloss", "armature", "stiffness", "springref"):
            if elem.get(attr) is not None:
                info[attr] = float(elem.get(attr))
        joints[name] = info
    return joints


# --------------------------------------------------------------------------
# URDF
# --------------------------------------------------------------------------
def parse_urdf(path: Path) -> dict[str, dict]:
    root = ET.parse(path).getroot()
    joints: dict[str, dict] = {}
    for elem in root.iter():
        if _strip_ns(elem.tag) != "joint":
            continue
        name = elem.get("name")
        if not name:
            continue
        info: dict = {"type": elem.get("type", "revolute")}
        for child in elem:
            if _strip_ns(child.tag) == "limit":
                for attr in ("lower", "upper", "effort", "velocity"):
                    if child.get(attr) is not None:
                        info[attr] = float(child.get(attr))
            elif _strip_ns(child.tag) == "axis":
                xyz = _floats(child.get("xyz"))
                if xyz:
                    info["axis"] = xyz
        joints[name] = info
    return joints


# --------------------------------------------------------------------------
# C++ policy parameters
# --------------------------------------------------------------------------
_ARRAY_RE = re.compile(
    r"constexpr\s+std::array<double,\s*(?P<size>\d+)>\s+(?P<name>\w+)\s*=\s*\{(?P<body>.*?)\};",
    re.DOTALL,
)


def parse_policy_parameters(path: Path) -> dict[str, list[float]]:
    text = path.read_text(encoding="utf-8")
    arrays: dict[str, list[float]] = {}
    for match in _ARRAY_RE.finditer(text):
        body = re.sub(r"//[^\n]*", "", match.group("body"))
        values = [float(v) for v in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", body)]
        declared = int(match.group("size"))
        if len(values) != declared:
            raise SystemExit(
                f"{path}: {match.group('name')} declared {declared} values, parsed {len(values)}"
            )
        arrays[match.group("name")] = values
    return arrays


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sonic-root", default=None)
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    args = parser.parse_args(argv)

    contract = load_contract()
    sonic_root = Path(args.sonic_root).expanduser().resolve() if args.sonic_root else contract.sonic_root

    mjcf_path = contract.mjcf_path
    urdf_path = contract.urdf_path
    if not mjcf_path.is_file() or not urdf_path.is_file():
        raise SystemExit(f"missing robot description: {mjcf_path} / {urdf_path}")

    mjcf = parse_mjcf(mjcf_path)
    urdf = parse_urdf(urdf_path)
    params = parse_policy_parameters(sonic_root / POLICY_PARAMS_REL)

    default_angles = params.get("a3_default_angles")
    kps = params.get("a3_kps")
    kds = params.get("a3_kds")
    action_scale = params.get("a3_action_scale")
    n = contract.n_policy_joints
    for name, arr in (
        ("a3_default_angles", default_angles),
        ("a3_kps", kps),
        ("a3_kds", kds),
    ):
        if arr is None or len(arr) != n:
            raise SystemExit(f"{POLICY_PARAMS_REL}: {name} must hold {n} values")

    warnings: list[str] = []
    joints_out: dict[str, dict] = {}
    for idx, name in enumerate(contract.policy_joint_names):
        m = mjcf.get(name)
        u = urdf.get(name)
        if m is None:
            raise SystemExit(f"policy joint {name!r} missing from MJCF {mjcf_path}")
        if u is None:
            raise SystemExit(f"policy joint {name!r} missing from URDF {urdf_path}")

        pos_lower: float | None = None
        pos_upper: float | None = None
        source: list[str] = []
        # The MJCF stores limits rounded to ~6 significant digits while the URDF
        # keeps full precision, so prefer the URDF value when both agree.
        if "range" in m:
            pos_lower, pos_upper = m["range"]
            source.append("mjcf")
        if "lower" in u and "upper" in u:
            if pos_lower is None:
                pos_lower, pos_upper = u["lower"], u["upper"]
                source.append("urdf")
            else:
                drift = max(abs(pos_lower - u["lower"]), abs(pos_upper - u["upper"]))
                if drift > RANGE_TOLERANCE:
                    warnings.append(
                        f"{name}: MJCF range [{pos_lower}, {pos_upper}] differs from "
                        f"URDF range [{u['lower']}, {u['upper']}] by {drift:.3e} rad"
                    )
                    pos_lower = min(pos_lower, u["lower"])
                    pos_upper = max(pos_upper, u["upper"])
                else:
                    pos_lower, pos_upper = u["lower"], u["upper"]
                    source.append("urdf")

        entry: dict = {
            "index": idx,
            "csv_index": contract.policy_to_csv_index[idx],
            "type": m.get("type", u.get("type")),
            "position_lower": pos_lower,
            "position_upper": pos_upper,
            "position_limit_source": source,
            "velocity_limit": u.get("velocity"),
            "effort_limit": u.get("effort"),
            "actuator_force_range": m.get("actuator_frc_range"),
            "default_angle": default_angles[idx],
            "kp": kps[idx],
            "kd": kds[idx],
            "damping": m.get("damping"),
            "frictionloss": m.get("frictionloss"),
            "armature": m.get("armature"),
            "axis": m.get("axis") or u.get("axis"),
        }
        if action_scale is not None and len(action_scale) == n:
            entry["action_scale"] = action_scale[idx]
        joints_out[name] = entry

    excluded: dict[str, dict] = {}
    for name in list(contract.head_joint_names) + list(contract.passive_foot_joint_names):
        info = mjcf.get(name)
        if info is None:
            continue
        excluded[name] = {
            "mjcf_type": info.get("type"),
            "position_range": info.get("range"),
            "damping": info.get("damping"),
            "stiffness": info.get("stiffness"),
            "armature": info.get("armature"),
            "in_policy_view": False,
        }

    doc = {
        "schema": "a3_joint_limits/v1",
        "generated_by": "tools/extract_a3_joint_limits.py",
        "sources": {
            "mjcf": str(mjcf_path),
            "urdf": str(urdf_path),
            "policy_parameters": str(sonic_root / POLICY_PARAMS_REL),
        },
        "policy_joint_order": list(contract.policy_joint_names),
        "joint_count": n,
        "joints": joints_out,
        "excluded_joints": excluded,
        "warnings": warnings,
    }

    out_path = Path(args.out).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(doc, handle, sort_keys=False, default_flow_style=False)

    print(f"mjcf      : {mjcf_path}")
    print(f"urdf      : {urdf_path}")
    print(f"joints    : {n} policy joints (+{len(excluded)} excluded)")
    for name, entry in joints_out.items():
        print(
            f"  [{entry['index']:2d}] {name:32s} "
            f"pos=[{entry['position_lower']:+.4f},{entry['position_upper']:+.4f}] "
            f"vel={entry['velocity_limit']:7.3f} eff={entry['effort_limit']:6.1f} "
            f"default={entry['default_angle']:+.4f} kp={entry['kp']:5.1f} kd={entry['kd']:4.1f}"
        )
    if warnings:
        print("\nWARNINGS:")
        for line in warnings:
            print(f"  ! {line}")
    print(f"\nwrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
