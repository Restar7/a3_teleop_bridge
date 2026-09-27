"""Per-frame UMR session: everything heavy is built once (plan sections 28-30).

This module runs **inside the UMR virtualenv** (it imports the UMR retargeting
script as a module) and is driven by :class:`~a3_teleop_bridge.umr.backends.UmrOnlineBackend`
from the bridge.

Stage I (``initialize``): load the SMPL-X body model, the A3 MJCF and the cached
correspondence slots; bind the source slots to the SMPL-X mesh; build the solver
arguments, the joint limits and the penetration caches; warm up the CUDA kernels.

Stage II (``solve_frame``): update the source surface for one frame, then call the
*existing* ``solve_frame_body_segment_qp`` with the previous solution as the warm
start, exactly like the offline pipeline's inner loop does.
"""

from __future__ import annotations

import importlib.util
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

__all__ = ["UmrRetargetSession", "UmrSessionError"]


class UmrSessionError(RuntimeError):
    """Raised when the UMR session cannot be prepared."""


@dataclass
class UmrRetargetSession:
    robot_config: Path | str | None = None
    umr_root: Path | str | None = None
    device: str = "cuda"
    verbose: bool = False

    # prepared state (Stage I)
    module: object = None
    common: object = None
    args: object = None
    model: object = None
    data: object = None
    joint_qpos_addrs: np.ndarray | None = None
    joint_dof_addrs: np.ndarray | None = None
    joint_names: list = field(default_factory=list)
    joint_limits_by_qpos: dict | None = None
    robot_template: object = None
    source_binding: object = None
    faces: np.ndarray | None = None
    template_vertices_centered: np.ndarray | None = None
    selected_slot_ids: np.ndarray | None = None
    source_slot_part_ids: np.ndarray | None = None
    surface_point_slot_costs: object = None
    surface_normal_slot_costs: object = None
    surface_normal_targets: np.ndarray | None = None
    caches: dict = field(default_factory=dict)
    anchor_state: dict = field(default_factory=dict)
    prepared: bool = False
    timings: dict = field(default_factory=dict)

    # ------------------------------------------------------------------
    @property
    def _root(self) -> Path:
        if self.umr_root:
            return Path(self.umr_root).expanduser()
        return Path.home() / "a3_teleop_ws" / "UMR"

    def _load_module(self):
        root = self._root
        scripts = root / "scripts"
        if str(scripts) not in sys.path:
            sys.path.insert(0, str(scripts))
        spec = importlib.util.spec_from_file_location(
            "umr_retarget_smpl", scripts / "retarget_smpl_to_humanoid_surface_vector.py"
        )
        if spec is None or spec.loader is None:
            raise UmrSessionError("cannot load the UMR retarget module")
        module = importlib.util.module_from_spec(spec)
        sys.modules["umr_retarget_smpl"] = module
        spec.loader.exec_module(module)
        return module

    # ------------------------------------------------------------------
    def initialize(self) -> None:
        """Build every heavy object exactly once."""
        started = time.perf_counter()
        if self.verbose:
            print("[umr-online] loading the UMR retarget module …")
        module = self._load_module()
        import smpl_surface_retarget_common as common  # noqa: E402  (UMR scripts dir)

        self.module = module
        self.common = common

        config_path = Path(self.robot_config).expanduser() if self.robot_config else None
        if config_path is None or not config_path.is_file():
            raise UmrSessionError(
                f"robot config not found: {config_path}; pass robot_config="
                "robot_configs/humanoid_retarget_agibot_a3.json"
            )
        from humanoid_retarget_config import load_config  # noqa: E402

        config = load_config(config_path)
        # The online path always uses the cached neutral correspondence.
        args = _build_args(module, config_path)
        self.args = args
        self.timings["args_ms"] = (time.perf_counter() - started) * 1e3

        stage = time.perf_counter()
        (
            self.model,
            self.data,
            self.joint_qpos_addrs,
            self.joint_dof_addrs,
            self.joint_names,
            self.joint_limits_by_qpos,
            self.robot_template,
            self.faces,
            self.template_vertices_centered,
            self.source_binding,
            self.selected_slot_ids,
            self.surface_point_slot_costs,
            self.surface_normal_slot_costs,
            self.surface_normal_targets,
        ) = _prepare_geometry(module, common, args, config)
        self.timings["geometry_ms"] = (time.perf_counter() - stage) * 1e3

        stage = time.perf_counter()
        self.caches = _prepare_caches(module, args, self.model, self.faces)
        self.timings["caches_ms"] = (time.perf_counter() - stage) * 1e3

        # CUDA warm-up: one throwaway solve so the first real frame is not slow
        stage = time.perf_counter()
        warm = _blank_frame(self)
        try:
            self.solve_frame(warm, first=True)
        except Exception as exc:  # pragma: no cover - reported, not fatal
            if self.verbose:
                print(f"[umr-online] warm-up solve raised: {exc}")
        self.timings["warmup_ms"] = (time.perf_counter() - stage) * 1e3

        self.prepared = True
        if self.verbose:
            print(f"[umr-online] ready in {sum(self.timings.values()):.1f} ms {self.timings}")

    # ------------------------------------------------------------------
    def reset(self, initial_frame=None) -> None:
        self.anchor_state = {}
        self._previous = None

    def solve_frame(self, frame, first: bool = False):
        """Solve one frame and return ``(qpos, info)``."""
        if not self.prepared and not first:
            raise UmrSessionError("initialize() has not completed")
        module = self.module
        common = self.common
        args = self.args

        started = time.perf_counter()
        vertices, joints = self.source_surface(frame)
        source_slots, source_slot_normals = self.bind_source(vertices)
        prepare_ms = (time.perf_counter() - started) * 1e3

        q_prev = getattr(self, "_previous_qpos", None)
        q_init = q_prev.copy() if q_prev is not None else self.initial_qpos(joints)
        solve_started = time.perf_counter()
        q_opt, cost = module.solve_frame_body_segment_qp(
            self.model,
            self.data,
            q_init,
            q_prev,
            getattr(self, "_previous_qpos2", None),
            source_slots,
            source_slot_normals,
            None if self.surface_normal_targets is None else self.surface_normal_targets,
            self.selected_slot_ids,
            self.source_slot_part_ids,
            self.surface_point_slot_costs,
            self.surface_normal_slot_costs,
            None,
            None,
            None,
            None,
            self.robot_template,
            self.joint_qpos_addrs,
            self.joint_dof_addrs,
            args,
            iters=int(args.iters),
            joint_limits_by_qpos=self.joint_limits_by_qpos,
            robot_self_penetration_cache=self.caches.get("self_penetration"),
            ground_penetration_collision_cache=self.caches.get("ground_penetration"),
            robot_object_penetration_cache=None,
            ground_contact_anchor_state=self.anchor_state,
        )
        solve_ms = (time.perf_counter() - solve_started) * 1e3
        self._previous_qpos2 = q_prev
        self._previous_qpos = q_opt
        info = {
            "cost": float(cost),
            "iterations": int(getattr(args, "iters", 0)),
            "prepare_ms": prepare_ms,
            "solve_ms": solve_ms,
            "dt": float(getattr(args, "dt", 1.0 / 30.0)),
        }
        return q_opt, info

    # ------------------------------------------------------------------
    def source_surface(self, frame) -> tuple[np.ndarray, np.ndarray]:
        """SMPL-X vertices/joints for one :class:`HumanSmplFrame` (world frame)."""
        common = self.common
        sequence = _sequence_from_frame(frame, getattr(self, "_sequence_template", None))
        vertices, joints, _faces = common.source_motion_vertices_joints(
            sequence,
            np.asarray([0]),
            self.args.smplx_model_dir,
            batch_size=1,
            zero_source_finger_pose=bool(self.args.zero_source_finger_pose),
            smplx_device=self.args.smplx_device,
            smplx_batch_size=1,
            smplx_batch_size_max=1,
            smplx_batch_size_safety_factor=1.0,
        )
        vertices = common.source_points_to_retarget_frame(vertices, "smplx", self.args.source_up)
        joints = common.source_points_to_retarget_frame(joints, "smplx", self.args.source_up)
        vertices, joints, _ground_z = common.preprocess_source_ground_for_retarget(
            vertices,
            joints,
            self.args.mat_height,
            self.args.source_ground_align,
            joint_names=self.source_joint_names,
        )
        return vertices[0], joints[0]

    def bind_source(self, vertices: np.ndarray):
        """Map the learned correspondence slots onto the current surface."""
        common = self.common
        slots, normals = common.bind_points_to_surface(
            self.smpl_slots, vertices, self.faces, nearest_vertex_k=self.args.bind_nearest_vertex_k
        ) if hasattr(common, "bind_points_to_surface") else (None, None)
        if slots is None:
            raise UmrSessionError(
                "smpl_surface_retarget_common has no bind_points_to_surface; "
                "online binding must be added"
            )
        normals = normals / np.maximum(np.linalg.norm(normals, axis=-1, keepdims=True), 1e-12)
        return slots, normals

    # ------------------------------------------------------------------
    def initial_qpos(self, joints: np.ndarray) -> np.ndarray:
        module = self.module
        common = self.common
        q = self.data.qpos.copy()
        q[self.joint_qpos_addrs] = 0.0
        q = common.clamp_joint_ranges(self.model, q, joint_limits_by_qpos=self.joint_limits_by_qpos)
        if joints is not None:
            q[:3] = common.source_root_joint_position(joints, self.source_joint_names)
            q[3:7] = common.source_qpos_body_heading(joints, self.source_joint_names)
        return q

    def joint_values(self, qpos: np.ndarray) -> dict[str, float]:
        return {
            name: float(qpos[int(self.joint_qpos_addrs[i])])
            for i, name in enumerate(self.joint_names)
        }

    def root_position(self, qpos: np.ndarray) -> np.ndarray:
        return np.asarray(qpos[:3], dtype=np.float64)

    def root_quaternion(self, qpos: np.ndarray) -> np.ndarray:
        return np.asarray(qpos[3:7], dtype=np.float64)


# ----------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------
def _build_args(module, config_path: Path):
    """Build the solver argument namespace from the robot config."""
    argv = [
        "--config",
        str(config_path),
        "--data",
        str(config_path),
    ]
    previous = sys.argv
    sys.argv = ["umr_online"] + argv
    try:
        args = module.parse_args()
    finally:
        sys.argv = previous
    module.fill_args_from_config(args)
    return args


def _prepare_geometry(module, common, args, config):
    raise UmrSessionError(
        "geometry preparation is not implemented yet: the online session must build "
        "the model, joints, template and correspondence exactly like the offline "
        "pipeline (see retarget_smpl_to_humanoid_surface_vector.main)."
    )


def _prepare_caches(module, args, model, faces):
    return {}


def _blank_frame(session):
    from ..types import HumanSmplFrame

    return HumanSmplFrame(
        seq=0,
        timestamp_ns=time.time_ns(),
        smpl_joints=np.zeros((24, 3)),
        smpl_pose=np.zeros((21, 3)),
        root_translation=np.zeros(3),
        root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
    )


def _sequence_from_frame(frame, template=None):
    raise UmrSessionError("_sequence_from_frame is not implemented yet")
