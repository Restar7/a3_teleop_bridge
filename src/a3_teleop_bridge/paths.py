"""Machine-independent path resolution for the three sibling repositories.

The workspace layout is fixed by the plan:

    <workspace>/
    ├── a3_teleop_bridge/     <- this repository
    ├── sonic_for_a3/
    └── UMR/

Nothing in the code or in the committed artifacts may hard-code a developer's
absolute workspace path: ``generated/a3_contract.json`` is checked in, so a
``sonic_root`` recorded on one machine is stale on every other one.  Every tool
must therefore go through the resolvers here.

Resolution order (identical for both repositories):

    1. explicit CLI argument (``--sonic-root`` / ``--umr-root``)
    2. environment variable (``SONIC_A3_ROOT`` / ``UMR_ROOT``, plus ``A3WS``)
    3. the value recorded in the contract/config, **but only if it exists here**
    4. the sibling repository next to this checkout
    5. otherwise a PathResolutionError listing everything that was tried

There is deliberately no "well-known developer path" fallback: if the repository
cannot be found, the user is told exactly what to set.
"""

from __future__ import annotations

import os
from pathlib import Path

ENV_SONIC_ROOT = "SONIC_A3_ROOT"
ENV_UMR_ROOT = "UMR_ROOT"
ENV_WORKSPACE = "A3WS"

# A file that only exists in the respective checkout.
SONIC_MARKER = Path("gear_sonic/scripts/sim2sim_a3_mujoco.py")
UMR_MARKER = Path("scripts/retarget_smpl_to_humanoid_surface_vector.py")


class PathResolutionError(RuntimeError):
    """Raised when a sibling repository cannot be located on this machine."""


def repo_root() -> Path:
    """``<workspace>/a3_teleop_bridge`` (derived from this file, never configured)."""
    return Path(__file__).resolve().parents[2]


def workspace_root() -> Path:
    """``<workspace>``: the parent directory holding the sibling checkouts."""
    env = os.environ.get(ENV_WORKSPACE)
    if env:
        return Path(env).expanduser().resolve()
    return repo_root().parent


def is_sonic_root(path: Path | str | None) -> bool:
    if path is None:
        return False
    return (Path(path).expanduser() / SONIC_MARKER).is_file()


def is_umr_root(path: Path | str | None) -> bool:
    if path is None:
        return False
    return (Path(path).expanduser() / UMR_MARKER).is_file()


def _resolve(
    *,
    what: str,
    explicit: Path | str | None,
    env_var: str,
    contract_value: Path | str | None,
    sibling_name: str,
    marker: Path,
    cli_flag: str,
) -> Path:
    tried: list[str] = []

    def consider(label: str, value: Path | str | None) -> Path | None:
        if value in (None, ""):
            return None
        candidate = Path(value).expanduser()
        tried.append(f"{label}: {candidate}")
        if (candidate / marker).is_file():
            return candidate.resolve()
        if candidate.exists():
            tried[-1] += "  (exists, but does not look like the checkout)"
        else:
            tried[-1] += "  (missing)"
        return None

    for label, value in (
        (f"CLI {cli_flag}", explicit),
        (f"env {env_var}", os.environ.get(env_var)),
        ("contract/config value", contract_value),
        ("sibling checkout", workspace_root() / sibling_name),
    ):
        found = consider(label, value)
        if found is not None:
            return found

    raise PathResolutionError(
        f"could not locate the {what} checkout on this machine.\n"
        f"Set {env_var}=/path/to/{sibling_name} or pass {cli_flag}.\n"
        + "\n".join(f"  tried {line}" for line in tried)
    )


def resolve_sonic_root(
    explicit: Path | str | None = None,
    contract_value: Path | str | None = None,
) -> Path:
    """Locate the ``sonic_for_a3`` checkout (see the module docstring for order)."""
    return _resolve(
        what="sonic_for_a3",
        explicit=explicit,
        env_var=ENV_SONIC_ROOT,
        contract_value=contract_value,
        sibling_name="sonic_for_a3",
        marker=SONIC_MARKER,
        cli_flag="--sonic-root",
    )


def resolve_umr_root(
    explicit: Path | str | None = None,
    contract_value: Path | str | None = None,
) -> Path:
    """Locate the ``UMR`` checkout (same order as :func:`resolve_sonic_root`)."""
    return _resolve(
        what="UMR",
        explicit=explicit,
        env_var=ENV_UMR_ROOT,
        contract_value=contract_value,
        sibling_name="UMR",
        marker=UMR_MARKER,
        cli_flag="--umr-root",
    )


def rebase_asset(
    stored: Path | str | None,
    stored_root: Path | str | None,
    resolved_root: Path,
) -> Path | None:
    """Re-point an asset path recorded relative to another machine's checkout.

    The contract stores paths such as
    ``/home/dev/ws/sonic_for_a3/gear_sonic/data/assets/...``.  If that absolute
    path still exists here it is used as-is; otherwise the part below the recorded
    root is re-attached to ``resolved_root`` (and relative paths are joined to it
    directly).
    """
    if stored in (None, ""):
        return None
    candidate = Path(stored).expanduser()
    if candidate.is_absolute() and candidate.exists():
        return candidate
    if not candidate.is_absolute():
        return resolved_root / candidate
    if stored_root not in (None, ""):
        try:
            relative = candidate.relative_to(Path(stored_root).expanduser())
        except ValueError:
            relative = None
        if relative is not None:
            return resolved_root / relative
    return candidate


def portable_path(path: Path | str | None, base: Path | None = None) -> str:
    """Render a path for reports/logs: ``$A3WS/...`` when it is inside the workspace."""
    if path in (None, ""):
        return ""
    candidate = Path(path).expanduser()
    for prefix, token in ((base, "$A3WS"), (workspace_root(), "$A3WS"), (repo_root(), "$A3WS/a3_teleop_bridge")):
        if prefix is None:
            continue
        try:
            return f"{token}/{candidate.relative_to(prefix)}"
        except ValueError:
            continue
    return str(candidate)
