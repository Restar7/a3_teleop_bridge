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
    # SMPL-X device for the *single live frame* forward pass. Measured on this
    # host (batch size 1, SMPL-X neutral): cpu 7.3 ms vs cuda:0 9.5 ms for the
    # forward itself, and 15.5 ms vs 37.8 ms for the whole per-frame prepare
    # (the CUDA path pays a launch/sync cost per call that a batch of one cannot
    # amortise). The offline batch pipeline keeps the configured device.
    smplx_device: str = "cpu"
    verbose: bool = False
    #: optional JSONL path for per-frame diagnostics (see LiveFrameDump)
    dump_frames: Path | str | None = None

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
    body_segment_module: object = None
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
        """UMR checkout: explicit -> $UMR_ROOT -> sibling of this repo."""
        from .. import paths

        if self.umr_root:
            return Path(self.umr_root).expanduser()
        return paths.resolve_umr_root()

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
    def _reference_sequence(self, config) -> dict:
        """A one-frame UMR ``sequence`` describing the *source template*.

        The template (neutral betas, neutral hands) is what the correspondence was
        learned against, so the online session must use the same one.
        """
        template_cfg = config.get("smpl_template", {}) if isinstance(config, dict) else {}
        betas = template_cfg.get("betas") if isinstance(template_cfg, dict) else None
        return {
            "pose_aa": np.zeros((1, 66), dtype=np.float32),
            "trans_orig": np.zeros((1, 3), dtype=np.float32),
            "beta": np.zeros(10, dtype=np.float32) if betas is None else np.asarray(betas, dtype=np.float32),
            "gender": str(template_cfg.get("gender", "neutral")).lower() if isinstance(template_cfg, dict) else "neutral",
            "fps": 30.0,
            "output_up": "z",
            "human_scale": 1.0,
            "human_scale_mode": "off",
            "source_format": "smplx_npz",
            "source_file": "<template>",
            "source_sequence_key": "template",
        }

    def initialize(self) -> None:
        """Build every heavy object exactly once."""
        started = time.perf_counter()
        if self.verbose:
            print("[umr-online] loading the UMR retarget module …")
        module = self._load_module()
        import smpl_surface_retarget_common as common  # noqa: E402  (UMR scripts dir)
        import retarget_body_segment_surface as body_segment  # noqa: E402

        self.body_segment_module = body_segment

        self.module = module
        self.common = common

        config_path = Path(self.robot_config).expanduser() if self.robot_config else None
        if config_path is None:
            # the plan fixes A3 as the robot: default to the config the M3/M4
            # runs used instead of failing on an omitted flag
            default_config = self._root / "robot_configs" / "humanoid_retarget_agibot_a3.json"
            config_path = default_config if default_config.is_file() else None
        if config_path is None or not config_path.is_file():
            raise UmrSessionError(
                f"robot config not found: {config_path}; pass robot_config="
                "robot_configs/humanoid_retarget_agibot_a3.json"
            )
        from humanoid_retarget_config import load_config  # noqa: E402

        config = load_config(config_path)
        # The online path always uses the cached neutral correspondence.
        args = _build_args(module, config_path)
        if self.smplx_device:
            args.smplx_device = str(self.smplx_device)
        self.args = args
        self.timings["args_ms"] = (time.perf_counter() - started) * 1e3

        stage = time.perf_counter()
        reference_sequence = self._reference_sequence(config)
        prepared = _prepare_geometry(module, common, args, config, reference_sequence)
        self.model = prepared["model"]
        self.data = prepared["data"]
        self.joint_qpos_addrs = prepared["joint_qpos_addrs"]
        self.joint_dof_addrs = prepared["joint_dof_addrs"]
        self.joint_names = prepared["joint_names"]
        self.joint_limits_by_qpos = prepared["joint_limits_by_qpos"]
        self.robot_template = prepared["robot_template"]
        self.faces = prepared["faces"]
        self.template_vertices_centered = prepared["template_vertices_centered"]
        self.template_joints_centered = prepared["template_joints_centered"]
        self.source_joint_names = prepared["source_joint_names"]
        self.source_model_type = prepared["source_model_type"]
        self.source_up = str(reference_sequence.get("output_up", "z"))
        self.smpl_slots = prepared["smpl_slots"]
        self.robot_slots = prepared["robot_slots"]
        self.smpl_slot_name = prepared["smpl_slot_name"]
        self.center_mode = prepared["center_mode"]
        self.robot_xml = prepared["robot_xml"]
        # Knee posture prior (UMR ``solver.joint_map_cost``).  The offline loop
        # wires this in solve_sequence; the online session drives
        # solve_frame_body_segment_qp itself, so without this the live reference
        # would silently keep the collapsed-knee behaviour the offline run just
        # fixed.  Calibrated once from the model, applied per frame.
        self.frame_dump = None
        _dump_path = self.dump_frames or getattr(args, "dump_frames", None)
        if _dump_path:
            self.frame_dump = LiveFrameDump(_dump_path)
            print(f"[umr-online] dumping per-frame diagnostics to {self.frame_dump.path}")

        self._knee_prior = None
        _prior_cost = float(getattr(args, "joint_map_cost", 0.0) or 0.0)
        if _prior_cost > 0.0:
            try:
                import mujoco as _mj

                knee_adrs, knee_dofs, knee_ranges = [], [], []
                for _side in ("left", "right"):
                    _jid = _mj.mj_name2id(self.model, _mj.mjtObj.mjOBJ_JOINT, f"{_side}_knee_joint")
                    if _jid < 0:
                        raise ValueError(f"{_side}_knee_joint not in the model")
                    knee_adrs.append(int(self.model.jnt_qposadr[_jid]))
                    knee_dofs.append(int(self.model.jnt_dofadr[_jid]))
                    knee_ranges.append(self.model.jnt_range[_jid])
                _calib = module.robot_knee_interior_calibration(
                    self.model,
                    self.data,
                    "left_hip_pitch_Link",
                    "left_knee_Link",
                    "left_ankle_roll_Link",
                    knee_adrs[0],
                )
                if _calib is not None:
                    self._knee_prior = {
                        "cost": _prior_cost,
                        "calib": _calib,
                        "adrs": knee_adrs,
                        "dofs": knee_dofs,
                        "ranges": knee_ranges,
                    }
            except Exception as exc:  # pragma: no cover - defensive
                self._knee_prior = None
                print(f"[umr-online][WARN] knee posture prior disabled: {exc}")
        # NOTE: filled in below, *after* the slot part labels exist. The offline
        # pipeline samples a bounded number of slots per body segment
        # (``sample_segment_slots``); feeding all 4096 correspondence slots to
        # the QP made every iteration ~5x more expensive for no accuracy gain.
        self.selected_slot_ids: np.ndarray = np.zeros(0, dtype=np.int32)
        # slot part labels + per-slot costs: derived once from the template (the
        # offline pipeline derives them from the same binding before its loop)
        # The slot->template binding depends only on the template, so it is
        # computed once here (it costs ~1.2 s) instead of on every frame.
        self.binding = common.bind_points_to_mesh(
            self.smpl_slots,
            self.template_vertices_centered,
            self.faces,
            nearest_vertex_k=self.args.bind_nearest_vertex_k,
        )
        self.surface_binding = {
            "face_ids": np.asarray(self.binding["face_ids"], dtype=np.int32),
            "bary": np.asarray(self.binding["bary"], dtype=np.float32),
        }
        self.bound_faces = np.asarray(self.faces, dtype=np.int32)[self.surface_binding["face_ids"]]
        self.source_slot_part_ids = np.asarray(
            self.body_segment_module.smplx_slot_part_ids_from_binding(
                self.faces,
                self.surface_binding["face_ids"],
                log_prefix="UmrOnlineInit",
                template_vertices=self.template_vertices_centered,
                bary=self.surface_binding["bary"],
                slot_points=self.binding["closest_points"],
            ),
            dtype=np.int32,
        )
        self.surface_point_slot_costs = module.surface_slot_costs_from_segments(
            len(self.smpl_slots), self.source_slot_part_ids, "point_cost", "SurfacePoint",
            log_prefix="UmrOnline",
        )
        self.surface_normal_slot_costs = module.surface_slot_costs_from_segments(
            len(self.smpl_slots), self.source_slot_part_ids, "normal_cost", "SurfaceNormal",
            log_prefix="UmrOnline",
        )
        self.surface_normal_targets = None  # direct source normals (see solve_frame)
        # Mirror ``retarget_smpl_to_humanoid_surface_vector.main``: a segmented
        # source (SMPL-X) keeps a bounded, seeded subset of the slots of every
        # body segment instead of the full correspondence pool.
        segment_groups = self.body_segment_module.body_segment_slot_groups(self.source_slot_part_ids)
        sample_counts = self.body_segment_module.segment_sample_counts()
        self.selected_slot_ids, self.selected_segment_groups = self.body_segment_module.sample_segment_slots(
            segment_groups,
            sample_counts,
            getattr(self.args, "seed", 0) or 0,
            log_prefix="UmrOnline",
        )
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
        ground_distances, ground_weight_distances = _ground_contact_inputs(common, source_slots, args)
        prepare_ms = (time.perf_counter() - started) * 1e3

        q_prev = getattr(self, "_previous_qpos", None)
        q_init = q_prev.copy() if q_prev is not None else self.initial_qpos(joints)
        # mirror the offline loop: the first frame gets the (larger) pose-init
        # iteration budget, every later frame the cheap warm-started budget
        iters = (
            int(args.pose_init_iters)
            if q_prev is None and int(getattr(args, "pose_init_iters", 0)) > 0
            else int(args.iters)
        )
        solve_started = time.perf_counter()
        if self._knee_prior is not None:
            args._joint_prior_cost = float(self._knee_prior["cost"])
            _intercept, _slope = self._knee_prior["calib"]
            _interior = module.source_knee_interior_deg(joints, self.source_joint_names)
            _rows = []
            for _i, (_adr, _dof) in enumerate(zip(self._knee_prior["adrs"], self._knee_prior["dofs"])):
                _lo, _hi = self._knee_prior["ranges"][_i]
                _target = (float(_interior[_i]) - _intercept) / _slope
                _rows.append((_adr, _dof, float(np.clip(_target, _lo, _hi))))
            args._joint_prior_rows = _rows
        q_opt, cost = module.solve_frame_body_segment_qp(
            self.model,
            self.data,
            q_init,
            q_prev,
            getattr(self, "_previous_qpos2", None),
            source_slots,
            source_slot_normals,
            None,  # direct mode: the solver matches against source_slot_normals
            self.selected_slot_ids,
            self.source_slot_part_ids,
            self.surface_point_slot_costs,
            self.surface_normal_slot_costs,
            None,  # source self-contact maps (no object/self map in the online path)
            ground_distances,
            ground_weight_distances,
            None,
            self.robot_template,
            self.joint_qpos_addrs,
            self.joint_dof_addrs,
            args,
            iters=iters,
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
            "iterations": int(iters),
            "prepare_ms": prepare_ms,
            "solve_ms": solve_ms,
            "dt": float(getattr(args, "dt", 1.0 / 30.0)),
        }
        if self.frame_dump is not None:
            record = {
                "t": round(time.time(), 4),
                "cost": round(float(cost), 6),
                "root_z": round(float(self.root_position(q_opt)[2]), 5),
            }
            record.update(_leg_angles_from_joints(joints))
            solved = self.joint_values(q_opt)
            for name in LEG_JOINTS:
                record[f"ref_{name}"] = round(float(solved.get(name, float("nan"))), 5)
            self.frame_dump.write(**record)
        return q_opt, info

    # ------------------------------------------------------------------
    def close(self) -> None:
        if getattr(self, "frame_dump", None) is not None:
            print(f"[umr-online] frame dump: {self.frame_dump.frames} frames -> {self.frame_dump.path}")
            self.frame_dump.close()
            self.frame_dump = None

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
        source_up = getattr(self, "source_up", "z")
        vertices = common.source_points_to_retarget_frame(vertices, "smplx", source_up)
        joints = common.source_points_to_retarget_frame(joints, "smplx", source_up)
        vertices, joints, _ground_z = common.preprocess_source_ground_for_retarget(
            vertices,
            joints,
            self.args.mat_height,
            self.args.source_ground_align,
            joint_names=self.source_joint_names,
        )
        return vertices[0], joints[0]

    def bind_source(self, vertices: np.ndarray):
        """Map the learned correspondence slots onto the current surface.

        The slot->template binding depends only on the template and is cached at
        initialization (it costs ~1.2 s); per frame only the barycentric
        interpolation and the slot normals are recomputed -- the same operations
        ``bind_source_slots_with_normals`` performs per frame offline.
        """
        common = self.common
        vertices = np.asarray(vertices, dtype=np.float64)
        slots = common.dynamic_surface_template_to_world(
            vertices, self.faces, self.surface_binding
        )
        triangles = vertices[self.bound_faces]
        normals = self.body_segment_module.normalize_vectors(
            np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        )
        return np.asarray(slots, dtype=np.float64), np.asarray(normals, dtype=np.float64)

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
def find_correspondence_slots(umr_root: Path, robot_name: str = "agibot_a3",
                              template: str = "smplx_neutral") -> Path | None:
    """Locate the cached correspondence slots for the neutral template.

    The offline pipeline computes this path (``correspondence_<robot>`` for the
    neutral template, ``correspondence_<robot>_smplx_neutral_betas_<hash>`` when the
    motion carries betas).  The online session always uses the neutral template,
    which is the one the PICO path needs.
    """
    out = Path(umr_root) / "output"
    candidates = [
        out / f"correspondence_{robot_name}" / "correspondence_slots_final.npz",
    ]
    for extra in sorted(out.glob(f"correspondence_{robot_name}_*/correspondence_slots_final.npz")):
        candidates.append(extra)
    for cand in candidates:
        if cand.is_file():
            try:
                names = np.load(cand, allow_pickle=True)["names"].astype(str).tolist()
            except Exception:
                continue
            if template in names:
                return cand
    return None


def _build_args(module, config_path: Path, defaults: Path | None = None, out_path: Path | None = None):
    """Build the solver argument namespace exactly like the offline pipeline.

    The offline pipeline merges the motion-source defaults with the robot config
    and writes a *runtime config* that the retarget script consumes; the online
    session does the same, so every solver flag keeps its repository default.
    """
    import tempfile

    from humanoid_retarget_pipeline import load_pipeline_config, write_runtime_config

    merged = load_pipeline_config(config_path, defaults) if defaults else load_pipeline_config(config_path)
    runtime_dir = Path(tempfile.mkdtemp(prefix="umr_online_"))
    write_runtime_config(merged, runtime_dir)  # returns the merged dict, writes the file
    runtime_path = runtime_dir / "humanoid_retarget_runtime_config.json"

    argv = [
        "--config",
        str(runtime_path),
        "--out",
        str(out_path or (runtime_dir / "online_session.npz")),
    ]
    previous = sys.argv
    sys.argv = ["umr_online"] + argv
    try:
        args = module.parse_args()
    finally:
        sys.argv = previous
    module.fill_args_from_config(args)
    # The online session matches robot normals against the *current* source normals
    # ("direct" mode): the tpose-offset transport needs the full trajectory's
    # template alignment and would add a per-frame cost we do not want online.
    # Offline runs can be compared 1:1 with --surface-normal-cost-mode direct.
    args.surface_normal_cost_mode = "direct"

    # the pipeline resolves config paths before handing them to the retarget
    # script (${VAR} expansion, relative-to-config resolution); do the same
    from humanoid_retarget_config import resolve_path, robot_config

    args.robot_xml = str(resolve_path(robot_config(args.config_data)["xml"], args.config_data))
    if getattr(args, "slots", None):
        args.slots = str(resolve_path(args.slots, args.config_data))
    # The offline pipeline computes the correspondence cache path; online we point
    # at the cached neutral-template slots explicitly.
    slots = find_correspondence_slots(_umr_root_from_config(args.config_data))
    if slots is None:
        raise UmrSessionError(
            "no cached correspondence slots found under <umr>/output/correspondence_agibot_a3; "
            "run the offline A3 retarget once (tools/run_umr_a3_batch.py)"
        )
    args.slots = str(slots)
    if getattr(args, "smplx_model_dir", None) and not Path(args.smplx_model_dir).is_absolute():
        candidate = resolve_path(args.smplx_model_dir, args.config_data)
        if candidate is not None and Path(candidate).is_dir():
            args.smplx_model_dir = str(candidate)
    return args


def _prepare_geometry(module, common, args, config, reference_sequence):
    """Build model / template / correspondence once (offline main() equivalent)."""
    import mujoco

    robot_xml = module.prepare_robot_xml(args)
    model = mujoco.MjModel.from_xml_path(str(robot_xml))
    data = mujoco.MjData(model)

    joint_qpos_addrs, joint_dof_addrs, _ranges, joint_names = common.scalar_qpos_joint_addrs(model)
    joint_limits_by_qpos, matched = common.build_scalar_joint_limits(
        model, module.config_joint_limits(args.config_data)
    )
    dof_max_dq_box_by_dof, _matched_dq = common.build_dof_max_dq_box(
        model, module.config_dof_max_dq_box(args.config_data)
    )
    args.dof_max_dq_box_by_dof = dof_max_dq_box_by_dof

    # template + correspondence (Stage I): the neutral templates are cached in the
    # correspondence slots file produced by the offline run.
    template_cfg = module.smpl_template_config(
        args.config_data,
        reference_sequence,
        str(reference_sequence.get("source_sequence_key", "")),
        str(reference_sequence.get("gender", "neutral")).lower(),
    )
    template_vertices, template_joints, faces, source_joint_names, source_model_type = (
        common.source_template_vertices_joints_faces(
            reference_sequence, template_cfg, args.smplx_model_dir, soma_usd_path=None
        )
    )
    if source_model_type != "smplx":
        raise UmrSessionError(
            f"the online session currently supports the smplx source, got {source_model_type!r}"
        )

    smpl_slots, center_mode, smpl_slot_name = common.load_slot_data(
        args.slots, args.smpl_name, args.slots_field
    )
    robot_slots, _robot_center, robot_slot_name = common.load_slot_data(
        args.slots, args.robot_name, args.slots_field
    )
    template_vertices_centered, template_joints_centered, template_center = (
        common.center_source_template(
            template_vertices,
            template_joints,
            center_mode,
            joint_names=source_joint_names,
            source_type=source_model_type,
        )
    )
    robot_template = module.bind_robot_slots(
        model,
        args,
        robot_slots,
        nearest_vertex_k=args.bind_nearest_vertex_k,
        project_to_surface=bool(args.project_robot_slots),
        source_model_type=source_model_type,
        template_cfg=template_cfg,
    )
    return {
        "robot_xml": robot_xml,
        "model": model,
        "data": data,
        "joint_qpos_addrs": joint_qpos_addrs,
        "joint_dof_addrs": joint_dof_addrs,
        "joint_names": list(joint_names),
        "joint_limits_by_qpos": joint_limits_by_qpos,
        "robot_template": robot_template,
        "faces": faces,
        "template_vertices_centered": template_vertices_centered,
        "template_joints_centered": template_joints_centered,
        "source_joint_names": source_joint_names,
        "source_model_type": source_model_type,
        "smpl_slots": smpl_slots,
        "robot_slots": robot_slots,
        "smpl_slot_name": smpl_slot_name,
        "robot_slot_name": robot_slot_name,
        "center_mode": center_mode,
        "template_center": template_center,
    }


def _umr_root_from_config(config_data) -> Path:
    """The UMR checkout that owns ``config_data``'s scripts directory."""
    module = sys.modules.get("umr_retarget_smpl")
    if module is not None and getattr(module, "ROOT", None):
        return Path(module.ROOT)
    from .. import paths

    return paths.resolve_umr_root()


def _prepare_caches(module, args, model, faces):
    """Collision/penetration caches: built once, reused by every solve."""
    common = module_common()
    return {
        "self_penetration": common.build_robot_self_penetration_cache(model, args),
        "ground_penetration": common.build_ground_penetration_collision_cache(model, args),
    }


def module_common():
    import smpl_surface_retarget_common as common

    return common


def _ground_contact_inputs(common, source_slots, args):
    """Per-frame ground-contact distances for the solver's contact map terms.

    Mirrors ``smpl_surface_retarget_common.compute_source_slot_ground_contact``
    (the offline pipeline computes it once for the whole batch; here it is
    computed for the single live frame). Returns ``(None, None)`` when the terms
    are disabled, exactly like the offline path.
    """
    if not (
        float(getattr(args, "ground_contact_map_cost", 0.0)) > 0.0
        or float(getattr(args, "ground_contact_anchor_cost", 0.0)) > 0.0
    ):
        return None, None
    raw = np.asarray(source_slots[:, 2], dtype=np.float32)
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    weight_distances = raw - raw.min(axis=1, keepdims=True)
    distances = np.maximum(raw, 0.0)
    snap_threshold = float(getattr(args, "ground_contact_map_snap_threshold", 0.0) or 0.0)
    if snap_threshold > 0.0:
        distances = distances.copy()
        distances[distances < snap_threshold] = 0.0
    return distances.reshape(-1), weight_distances.reshape(-1)


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


def sequence_from_frame(frame, betas=None, fps: float = 30.0) -> dict:
    """One :class:`HumanSmplFrame` as a UMR ``sequence`` (same keys as the loader).

    ``pose_aa`` starts with the root rotation, exactly like the SMPL-X npz the
    offline pipeline consumes; hands/face stay neutral.
    """
    from scipy.spatial.transform import Rotation

    quat = frame.body_quat_w if frame.body_quat_w is not None else frame.root_quat_wxyz
    root_rotvec = Rotation.from_quat(np.asarray(quat, dtype=np.float64)[[1, 2, 3, 0]]).as_rotvec()
    pose_aa = np.concatenate([root_rotvec[None, :], np.asarray(frame.smpl_pose, dtype=np.float64)])
    return {
        "pose_aa": pose_aa.reshape(1, -1).astype(np.float32),
        "trans_orig": np.asarray(frame.root_translation, dtype=np.float64).reshape(1, 3).astype(np.float32),
        "beta": np.zeros(10, dtype=np.float32) if betas is None else np.asarray(betas, dtype=np.float32),
        "gender": "neutral",
        "fps": float(fps),
        "output_up": "z",
        "human_scale": 1.0,
        "human_scale_mode": "off",
        "source_format": "smplx_npz",
        "source_file": "<live>",
        "source_sequence_key": "live",
    }


LEG_JOINTS = (
    "left_hip_pitch_joint", "left_knee_joint", "left_ankle_pitch_joint",
    "right_hip_pitch_joint", "right_knee_joint", "right_ankle_pitch_joint",
)
#: SMPL-X indices used for the source-side leg angles (hip, knee, ankle)
_SMPLX_LEG = {"L": (1, 4, 7), "R": (2, 5, 8)}


def _leg_angles_from_joints(joints) -> dict:
    """Source-side hip/knee/ankle angles, in the same units the retarget sees."""
    import numpy as _np

    out: dict[str, float] = {}
    for side, (hip, knee, ankle) in _SMPLX_LEG.items():
        h = _np.asarray(joints[hip], dtype=_np.float64)
        k = _np.asarray(joints[knee], dtype=_np.float64)
        a = _np.asarray(joints[ankle], dtype=_np.float64)
        thigh, shank = h - k, a - k
        cosang = float(
            _np.dot(thigh, shank) / max(float(_np.linalg.norm(thigh) * _np.linalg.norm(shank)), 1e-12)
        )
        out[f"{side}_knee_interior_deg"] = float(
            _np.degrees(_np.arccos(_np.clip(cosang, -1.0, 1.0)))
        )
        out[f"{side}_hip_to_ankle_m"] = float(_np.linalg.norm(h - a))
        # thigh tilt away from straight down: knee - hip, so 0 deg = vertical
        # (using hip - knee points up and reports ~180 for a standing leg)
        down = _np.array([0.0, 0.0, -1.0])
        thigh_down = k - h
        out[f"{side}_thigh_tilt_deg"] = float(
            _np.degrees(
                _np.arccos(
                    _np.clip(
                        _np.dot(thigh_down / max(float(_np.linalg.norm(thigh_down)), 1e-12), down),
                        -1.0,
                        1.0,
                    )
                )
            )
        )
    return out


class LiveFrameDump:
    """Append-only JSONL of what the live session saw and what it produced.

    One line per solved frame: the source-side leg angles plus the reference
    joint values that came out.  Comparing the excursions of the two answers
    "did the operator's leg motion reach the reference?" without guessing.
    """

    def __init__(self, path):
        from pathlib import Path as _Path

        self.path = _Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("w", encoding="utf-8")
        self.frames = 0

    def write(self, **fields) -> None:
        import json as _json

        self._handle.write(_json.dumps(fields, separators=(",", ":")) + "\n")
        self.frames += 1
        if self.frames % 50 == 0:
            self._handle.flush()

    def close(self) -> None:
        try:
            self._handle.flush()
            self._handle.close()
        except Exception:
            pass


def _sequence_from_frame(frame, template=None):
    return sequence_from_frame(frame, betas=None if template is None else template.get("betas"))
