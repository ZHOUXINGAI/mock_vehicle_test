#!/usr/bin/env python3
"""Analyze paired Carrier/Mini LR24 Pair B dry-run artifacts offline."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_pairb_run_analysis import (  # noqa: E402
    SCHEMA_VERSION,
    ArtifactError,
    analyze_pair,
    load_contract,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--carrier-log", required=True)
    parser.add_argument("--carrier-csv", required=True)
    parser.add_argument("--mini-log", required=True)
    parser.add_argument("--mini-csv", required=True)
    parser.add_argument(
        "--config", default=str(REPO_DIR / "config/lr24/pairb_v1.json")
    )
    parser.add_argument(
        "--mode", choices=("targeted", "broadcast"), default="targeted"
    )
    parser.add_argument("--min-state-coverage", type=float, default=0.95)
    parser.add_argument("--min-command-coverage", type=float, default=0.95)
    parser.add_argument("--min-plan-coverage", type=float, default=0.95)
    parser.add_argument("--state-stale-ms", type=int)
    parser.add_argument("--watchdog-ms", type=int)
    parser.add_argument("--json-out")
    return parser


def _error_report(args: argparse.Namespace, message: str) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "inputs": {
            "config": args.config,
            "carrier_log": args.carrier_log,
            "carrier_csv": args.carrier_csv,
            "mini_log": args.mini_log,
            "mini_csv": args.mini_csv,
        },
        "mode": args.mode,
        "diagnostic_only": args.mode == "broadcast",
        "directed_acceptance": False,
        "endpoints": {},
        "metrics": {},
        "checks": [],
        "failures": [
            {
                "code": "input_artifact_error",
                "pass": False,
                "message": message,
                "evidence": {},
            }
        ],
        "pass": False,
    }


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        report = analyze_pair(
            carrier_log=args.carrier_log,
            carrier_csv=args.carrier_csv,
            mini_log=args.mini_log,
            mini_csv=args.mini_csv,
            contract=load_contract(args.config),
            mode=args.mode,
            min_state_coverage=args.min_state_coverage,
            min_command_coverage=args.min_command_coverage,
            min_plan_coverage=args.min_plan_coverage,
            state_stale_ms=args.state_stale_ms,
            command_watchdog_ms=args.watchdog_ms,
        )
        exit_code = 0 if report["pass"] else 1
    except ArtifactError as exc:
        report = _error_report(args, str(exc))
        exit_code = 2

    rendered = json.dumps(
        report, indent=2, sort_keys=True, ensure_ascii=True
    )
    print(rendered)
    if args.json_out:
        try:
            Path(args.json_out).write_text(rendered + "\n", encoding="utf-8")
        except OSError as exc:
            print(f"cannot write JSON report {args.json_out}: {exc}", file=sys.stderr)
            return 2
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
