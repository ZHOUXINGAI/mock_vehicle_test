#!/usr/bin/env python3

"""Generate and validate Orin2-led two-rover plans without hardware access."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]
if str(REPO_DIR) not in sys.path:
    sys.path.insert(0, str(REPO_DIR))

from src.orin2_carrier_leader_core import (  # noqa: E402
    DEFAULT_EASYDOCKING_SRC,
    ORIN1_MINI_SYSTEM_ID,
    ORIN2_CARRIER_SYSTEM_ID,
    build_parallel_straight_plan,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("parallel", "docking", "all"), default="all")
    parser.add_argument("--output-dir", default="results/orin2_carrier_coordination")
    parser.add_argument("--easydocking-src", type=Path, default=DEFAULT_EASYDOCKING_SRC)
    return parser.parse_args()


def write_parallel(output_dir: Path) -> dict[str, object]:
    plan = build_parallel_straight_plan()
    payload = asdict(plan)
    (output_dir / "parallel_straight_plan.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    with (output_dir / "parallel_straight_paths.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=("role", "index", "x_m", "y_m"))
        writer.writeheader()
        for role, points in (("carrier", plan.carrier_path), ("mini", plan.mini_path)):
            for index, point in enumerate(points):
                writer.writerow(
                    {"role": role, "index": index, "x_m": point.x_m, "y_m": point.y_m}
                )
    return {
        "pass": True,
        "distance_m": plan.distance_m,
        "speed_mps": plan.speed_mps,
        "initial_front_gap_m": plan.initial_front_gap_m,
        "carrier_points": len(plan.carrier_path),
        "mini_points": len(plan.mini_path),
    }


def write_docking_replay(output_dir: Path, easydocking_src: Path) -> dict[str, object]:
    easy_root = easydocking_src.expanduser().resolve().parent
    easy_scripts = easy_root / "scripts"
    for path in (easydocking_src.expanduser().resolve(), easy_scripts):
        if not path.is_dir():
            raise FileNotFoundError(f"EasyDocking path unavailable: {path}")
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from ground_docking_leader import LeaderConfig
    from run_ground_docking_replay import run_fault_replays, run_nominal_replay

    config = LeaderConfig(
        carrier_vehicle_id=ORIN2_CARRIER_SYSTEM_ID,
        mini_vehicle_id=ORIN1_MINI_SYSTEM_ID,
    )
    nominal = run_nominal_replay(config=config)
    faults = run_fault_replays()
    result = {
        "pass": bool(nominal["pass"] and faults["pass"]),
        "hardware_access": False,
        "role_map": {
            "orin2": {"docking_role": "carrier", "mav_sys_id": 2},
            "orin1": {"docking_role": "mini", "mav_sys_id": 1},
        },
        "nominal": nominal,
        "faults": faults,
    }
    (output_dir / "docking_leader_replay.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return result


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir) / datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=False)
    summary: dict[str, object] = {
        "mode": "offline_only",
        "hardware_access": False,
        "role_map": "Orin2/system2=Carrier leader; Orin1/system1=Mini executor",
    }
    if args.scenario in {"parallel", "all"}:
        summary["parallel"] = write_parallel(output_dir)
    if args.scenario in {"docking", "all"}:
        summary["docking"] = write_docking_replay(output_dir, args.easydocking_src)
    summary["pass"] = all(
        bool(value.get("pass"))
        for value in summary.values()
        if isinstance(value, dict) and "pass" in value
    )
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"output_dir={output_dir}")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
