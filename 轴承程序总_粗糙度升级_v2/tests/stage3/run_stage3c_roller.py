#!/usr/bin/env python3
"""Stage 3C roller traction validation with one MATLAB process per worker."""
import argparse
import io
import json
import os
import shutil
import subprocess
import tarfile
import tempfile
from datetime import datetime
from pathlib import Path

MATLAB = os.environ.get("MATLAB_EXECUTABLE", "matlab")
LEVEL1 = "stage3c_level1_snapshot_worker"
LEVEL2 = "stage3c_level2_traction_worker"
SPEED = "stage3c_level2_speed_worker"
OFF = "stage3c_off_worker"
DEFAULT_SEED = Path(tempfile.gettempdir()) / "stage3-recovery-seed" / "roller_rough_feedback_validation.mat"


def matlab_quote(value: Path) -> str:
    return str(value).replace("'", "''")


def roughness(rq: float, level: int) -> dict:
    pair = {"etaAsperity": 1e10, "betaAsperity": 1e-6,
            "C_GT": 1e-4, "muBoundary": 0.15}
    return {"enabled": True, "mode": "diagnostic", "feedback_level": level,
            "Rq_inner": rq, "Rq_outer": rq, "Rq_element": rq,
            "inner_pair": pair, "outer_pair": pair, "omega": 0.5}


def repo_relative(repo_root: Path, project_root: Path) -> str:
    return project_root.relative_to(repo_root).as_posix()


def make_run_directory(repo_root: Path, project_root: Path, step_dir: Path) -> Path:
    """Export an exact HEAD legacy template into an isolated worker directory."""
    relative = repo_relative(repo_root, project_root)
    target = step_dir / "roller_run"
    archive = subprocess.run(
        ["git", "archive", "--format=tar", "HEAD", f"{relative}/滚子轴承程序"],
        cwd=repo_root, check=True, capture_output=True).stdout
    with tarfile.open(fileobj=io.BytesIO(archive)) as package:
        package.extractall(step_dir)
    exported = step_dir / relative / "滚子轴承程序"
    shutil.move(str(exported), str(target))
    shutil.rmtree(step_dir / relative, ignore_errors=True)
    shutil.copy2(project_root / "滚子轴承程序" / "ffSPEED.m", target / "ffSPEED.m")
    loader = subprocess.run(
        ["git", "show", f"HEAD:{relative}/load_micro_interface_config.m"],
        cwd=repo_root, check=True, capture_output=True).stdout
    (target / "load_micro_interface_config.m").write_bytes(loader)
    return target


def launch(repo_root: Path, project_root: Path, temp_root: Path, index: int,
           worker: str, payload: dict) -> tuple[Path, dict, str]:
    step = temp_root / f"step_{index:03d}"
    step.mkdir(parents=True, exist_ok=False)
    run_dir = make_run_directory(repo_root, project_root, step)
    output = step / "output_state.mat"
    status = step / "status.json"
    payload = dict(payload)
    payload.update({"repository_root": str(project_root), "run_directory": str(run_dir),
                    "output_state_path": str(output), "status_json_path": str(status)})
    control = step / "control.json"
    control.write_text(json.dumps(payload), encoding="utf-8")
    expression = ("try, addpath('" + matlab_quote(project_root / "tests" / "stage3") + "'); "
                  + worker + "('" + matlab_quote(control) + "'); exit(0); "
                  "catch ME, disp(getReport(ME,'extended','hyperlinks','off')); exit(1); end")
    environment = os.environ.copy()
    runtime = step / "matlab_runtime"
    runtime.mkdir()
    environment.update({"HOME": str(runtime), "TMPDIR": str(runtime),
                        "MATLAB_PREFDIR": str(runtime)})
    process = subprocess.run([MATLAB, "-nodisplay", "-nosplash", "-r", expression],
                             cwd=run_dir, env=environment, text=True,
                             capture_output=True, timeout=900)
    if not status.is_file():
        raise RuntimeError(f"{worker} did not create status.json; return={process.returncode}; "
                           f"stderr={process.stderr[-2000:]}")
    result = json.loads(status.read_text(encoding="utf-8"))
    if process.returncode != 0:
        result["process_returncode"] = process.returncode
        result["process_stderr"] = process.stderr[-2000:]
        result["success"] = False
    if not output.is_file():
        result["success"] = False
        result["failure_reason"] = "worker did not create output_state.mat"
    return output, result, str(step)


def run_level2_path(repo_root: Path, project_root: Path, temp_root: Path,
                    index: int, snapshot: Path, case_name: str, config: dict) -> tuple[list, int]:
    reports = []
    for gamma in (0.0, 0.5, 1.0):
        output, status, step = launch(repo_root, project_root, temp_root, index, LEVEL2, {
            "input_snapshot_path": str(snapshot), "case_name": case_name,
            "gamma_mu": gamma, "roughness": config})
        reports.append({"output_state_path": str(output), "status": status, "step_directory": step})
        index += 1
        if not status.get("success", False):
            break
    return reports, index


def run_speed_path(repo_root: Path, project_root: Path, temp_root: Path,
                   index: int, snapshot: Path, case_name: str, config: dict) -> tuple[list, int]:
    """Run Stage 3C-2 separately: legacy speed recomputation on a frozen normal state."""
    reports = []
    for gamma in (0.0, 0.5, 1.0):
        output, status, step = launch(repo_root, project_root, temp_root, index, SPEED, {
            "input_snapshot_path": str(snapshot), "case_name": case_name,
            "gamma_mu": gamma, "roughness": config})
        reports.append({"output_state_path": str(output), "status": status, "step_directory": step})
        index += 1
        if not status.get("success", False):
            break
    return reports, index


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=Path, default=DEFAULT_SEED)
    parser.add_argument("--temp-root", type=Path)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()
    project_root = Path(__file__).resolve().parents[2]
    repo_root = project_root.parent
    if not args.seed.is_file():
        raise RuntimeError(f"Stage 2C gamma=1 seed is unavailable: {args.seed}")
    temp_root = args.temp_root or Path(tempfile.mkdtemp(prefix="stage3c_"))
    temp_root.mkdir(parents=True, exist_ok=True)
    index = 1
    manifest = {"execution_mode": "isolated_matlab_workers", "temp_root": str(temp_root),
                "gamma_mu_path": [0.0, 0.5, 1.0], "cases": {}}
    off, off_status, off_step = launch(repo_root, project_root, temp_root, index, OFF, {
        "case_name": "OFF"})
    manifest["cases"]["off"] = {"output_state_path": str(off), "status": off_status,
                                "step_directory": off_step}
    index += 1
    smooth_level1, smooth_status, smooth_step = launch(repo_root, project_root, temp_root, index, LEVEL1, {
        "input_state_path": "", "case_name": "LEVEL1_SMOOTH", "gamma": 1.0,
        "staggered_iteration": 1, "roughness": roughness(0.0, 1)})
    manifest["cases"]["level1_smooth"] = {"output_state_path": str(smooth_level1),
                                             "status": smooth_status, "step_directory": smooth_step}
    index += 1
    smooth_path, index = run_level2_path(repo_root, project_root, temp_root, index,
        smooth_level1, "LEVEL2_SMOOTH", roughness(0.0, 2))
    manifest["cases"]["level2_smooth"] = smooth_path
    smooth_speed_path, index = run_speed_path(repo_root, project_root, temp_root, index,
        smooth_level1, "LEVEL2_SMOOTH_SPEED_RECOMPUTE", roughness(0.0, 2))
    manifest["cases"]["level2_smooth_speed_recompute"] = smooth_speed_path
    rough_level1, rough_status, rough_step = launch(repo_root, project_root, temp_root, index, LEVEL1, {
        "input_state_path": str(args.seed), "case_name": "LEVEL1_ROUGH", "gamma": 1.0,
        "staggered_iteration": 1, "roughness": roughness(12e-9, 1)})
    manifest["cases"]["level1_rough"] = {"output_state_path": str(rough_level1),
                                            "status": rough_status, "step_directory": rough_step}
    index += 1
    rough_path, index = run_level2_path(repo_root, project_root, temp_root, index,
        rough_level1, "LEVEL2_ROUGH", roughness(12e-9, 2))
    manifest["cases"]["level2_rough"] = rough_path
    rough_speed_path, index = run_speed_path(repo_root, project_root, temp_root, index,
        rough_level1, "LEVEL2_ROUGH_SPEED_RECOMPUTE", roughness(12e-9, 2))
    manifest["cases"]["level2_rough_speed_recompute"] = rough_speed_path
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest))


if __name__ == "__main__":
    main()
