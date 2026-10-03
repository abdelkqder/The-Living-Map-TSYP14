#!/usr/bin/env python3
"""
run_simulation.py
==================
Entry point for the Living Map demo and benchmark.

Interactive demo
----------------
    python run_simulation.py [--seed N] [--difficulty EASY|MEDIUM|HARD]

Benchmark mode (headless, no pygame)
-------------------------------------
    python run_simulation.py --benchmark [--seed N] [--difficulty EASY|MEDIUM|HARD]

Controls (keyboard)
-------------------
    W       deploy writer (any time)
    E       dispatch executor (needs Living Map candidates)
    D       deploy replacement writer (once primary writer is done)
    P       pause / resume
    SPACE   smart start (writer → executor, sequential path)
    R       reset same scenario
    Q/Esc   quit

Controls (buttons in Command Post panel)
-----------------------------------------
    [DEPLOY WRITER]         same as W
    [DISPATCH EXECUTOR]     same as E  (greyed-out when no candidates)
    [REPLACEMENT WRITER]    same as D  (greyed-out while writer is still active)
    [AUTO DISPATCH ON/OFF]  toggle event-driven dispatch (PATCH 3)
    [PAUSE / RESUME]        same as P
"""
import argparse
import sys


def main() -> None:
    parser = argparse.ArgumentParser(
        description="THE LIVING MAP — TSYP14 Phase-1 simulation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--seed",       type=int, default=42,
                        help="scenario random seed (event placement)")
    parser.add_argument("--difficulty", choices=["EASY", "MEDIUM", "HARD"],
                        default="MEDIUM",
                        help="scenario difficulty (PATCH 6)")
    parser.add_argument("--benchmark",  action="store_true",
                        help="run headless benchmark (no pygame) and print KPI table")
    args = parser.parse_args()

    if args.benchmark:
        from simulation.benchmark import run_benchmark, print_report
        print(f"Running benchmark  seed={args.seed}  difficulty={args.difficulty} ...")
        results = run_benchmark(seed=args.seed, difficulty=args.difficulty)
        print_report(results)
        return

    try:
        from simulation.renderer import run
    except ImportError as exc:
        print(f"pygame is required for the interactive demo: {exc}", file=sys.stderr)
        print("Install with:  pip install -r requirements.txt", file=sys.stderr)
        print("Or run headless:  python run_simulation.py --benchmark", file=sys.stderr)
        sys.exit(1)

    run(seed=args.seed, difficulty=args.difficulty)


if __name__ == "__main__":
    main()
