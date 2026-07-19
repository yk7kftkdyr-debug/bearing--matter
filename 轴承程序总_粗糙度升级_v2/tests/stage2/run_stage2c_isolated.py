#!/usr/bin/env python3
"""Stage 2C smoke test: one fresh MATLAB process per mechanical solve."""
import json
import os
import shutil
import subprocess
import tempfile
import argparse
from pathlib import Path
from typing import Optional, Tuple

MATLAB = "/Applications/MATLAB_R2024a.app/bin/matlab"
WORKER = "stage2c_isolated_worker"
MAX_OUTER_ITERATIONS = 20
ERR_Q_TOL = 1e-6
ERR_H_TOL = 1e-6
CLOSURE_TOL = 1e-6


def matlab_quote(value: Path) -> str:
    return str(value).replace("'", "''")


def roughness_config(rq: float) -> dict:
    pair = {"etaAsperity": 1e10, "betaAsperity": 1e-6,
            "C_GT": 1e-4, "muBoundary": 0.15}
    return {"enabled": True, "mode": "diagnostic", "feedback_level": 1,
            "Rq_inner": rq, "Rq_outer": rq, "Rq_element": rq,
            "inner_pair": pair, "outer_pair": pair, "omega": 0.5}


def git_status(root: Path) -> str:
    return subprocess.run(["git", "status", "--porcelain"], cwd=root,
                          check=True, text=True, capture_output=True).stdout


def launch_worker(project_root: Path, step_dir: Path, previous_output: Optional[Path],
                  iteration: int, gamma: float, case_name: str, rq: float) -> Tuple[Path, dict]:
    shutil.copytree(project_root / "滚子轴承程序", step_dir)
    input_state = step_dir / "input_state.mat"
    if previous_output is not None:
        shutil.copy2(previous_output, input_state)
    control = {
        "repository_root": str(project_root),
        "run_directory": str(step_dir),
        "input_state_path": str(input_state),
        "output_state_path": str(step_dir / "output_state.mat"),
        "status_json_path": str(step_dir / "status.json"),
        "case_name": case_name,
        "gamma": gamma,
        "staggered_iteration": iteration,
        "roughness": roughness_config(rq),
    }
    control_path = step_dir / "control.json"
    control_path.write_text(json.dumps(control), encoding="utf-8")
    expression = (
        f"addpath('{matlab_quote(project_root / 'tests' / 'stage2')}'); "
        f"{WORKER}('{matlab_quote(control_path)}');"
    )
    environment = os.environ.copy()
    environment.setdefault("MATLAB_PREFDIR", "/private/tmp/matlab-stage2c-pref")
    process = subprocess.run([MATLAB, "-batch", expression], cwd=project_root,
                             env=environment, text=True, capture_output=True, timeout=900)
    status_path = Path(control["status_json_path"])
    if process.returncode != 0:
        raise RuntimeError(f"worker {iteration} returned {process.returncode}: {process.stderr}")
    if not status_path.is_file():
        raise RuntimeError(f"worker {iteration} did not write status.json")
    status = json.loads(status_path.read_text(encoding="utf-8"))
    if not status.get("success"):
        raise RuntimeError(f"worker {iteration} failed: {status.get('failure_reason', '')}")
    if status.get("qiujieall_call_count") != 1:
        raise RuntimeError(f"worker {iteration} qiujieall count is not one")
    output = Path(control["output_state_path"])
    if not output.is_file():
        raise RuntimeError(f"worker {iteration} did not write output_state.mat")
    return output, status


def converged(status: dict, previous_ids: Optional[list]) -> bool:
    current_ids = status.get("roller_id", [])
    ids_equal = previous_ids is not None and current_ids == previous_ids
    return (ids_equal and status.get("active_set_stable") is True and
            status.get("mechanical_converged") is True and
            status.get("gt_converged") is True and
            status.get("physical_finite_real") is True and
            status.get("errQ", float("inf")) <= ERR_Q_TOL and
            status.get("errH", float("inf")) <= ERR_H_TOL and
            status.get("closure_error", float("inf")) < CLOSURE_TOL)


def run_gamma(project_root: Path, temp_root: Path, previous_output: Optional[Path],
              gamma: float, step_index: int, rq: float, case_name: str) -> Tuple[Path, dict, int]:
    previous_ids = None
    for iteration in range(1, MAX_OUTER_ITERATIONS + 1):
        step_dir = temp_root / f"step_{step_index:03d}"
        output, status = launch_worker(project_root, step_dir, previous_output,
                                       iteration, gamma, case_name, rq)
        if converged(status, previous_ids):
            return output, status, step_index + 1
        previous_output = output
        previous_ids = status.get("roller_id", [])
        step_index += 1
    raise RuntimeError(f"gamma={gamma} did not meet fixed tolerances in {MAX_OUTER_ITERATIONS} iterations")


def run_path(project_root: Path, temp_root: Path, gamma_path: tuple, rq: float, case_name: str) -> Tuple[Path, list]:
    state = None
    reports = []
    step_index = 1
    for gamma in gamma_path:
        state, status, step_index = run_gamma(project_root, temp_root, state, gamma, step_index, rq, case_name)
        reports.append(status)
    return state, reports


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--smooth", action="store_true")
    parser.add_argument("--persist-dir", type=Path,
                        help="Copy the final MAT/JSON evidence here after success.")
    arguments = parser.parse_args()
    project_root = Path(__file__).resolve().parents[2]
    if not (project_root / "滚子轴承程序").is_dir():
        raise RuntimeError("roller run template is missing")
    before = git_status(project_root)
    temp_root = Path(tempfile.mkdtemp(prefix="stage2c_", dir="/private/tmp"))
    try:
        rq = 0.0 if arguments.smooth else 12e-9
        case_name = "roller_smooth_feedback" if arguments.smooth else "roller_rough_feedback"
        output2, reports = run_path(project_root, temp_root, (0.0, 0.5, 1.0), rq, case_name)
        if git_status(project_root) != before:
            raise RuntimeError("worker smoke test polluted the repository working tree")
        result = {"success": True, "case_name": case_name, "gamma_path": [0.0, 0.5, 1.0],
                  "gamma_reports": reports, "final_output_state": str(output2)}
        if arguments.persist_dir is not None:
            arguments.persist_dir.mkdir(parents=True, exist_ok=True)
            stem = "roller_smooth_feedback" if arguments.smooth else "roller_rough_feedback"
            mat_target = arguments.persist_dir / f"{stem}_validation.mat"
            json_target = arguments.persist_dir / f"{stem}_status.json"
            shutil.copy2(output2, mat_target)
            json_target.write_text(json.dumps(result), encoding="utf-8")
            result["persisted_mat"] = str(mat_target)
            result["persisted_status"] = str(json_target)
        print(json.dumps(result))
        print("STAGE2C_GAMMA_PATH_PASS")
    finally:
        shutil.rmtree(temp_root, ignore_errors=True)


if __name__ == "__main__":
    main()
