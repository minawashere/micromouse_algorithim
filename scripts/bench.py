#!/usr/bin/env python3
"""Benchmark: sweep (maze x heuristic recipe) over the classic mazes.

Runs the compiled `run` binary in offline SimWorld mode, collects the mms-style
score, and prints a comparison table. Score (mms semantics, replicated in
SimWorld) = best-run turns + best-run effective distance
            + 0.1 * (total turns + total effective distance).
"""
import argparse
import concurrent.futures as cf
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUN = ROOT / "build" / "run"
EXTRA = []

RECIPES = {
    "p1opt-p2opt": ["--p1heur", "opt", "--p2heur", "opt"],
    "p1man-p2opt": ["--p1heur", "manhattan", "--p2heur", "opt"],
    "p1true-p2opt": ["--p1heur", "true", "--p2heur", "opt"],
    "p1opt-p2true": ["--p1heur", "opt", "--p2heur", "true"],
    "p1true-p2true": ["--p1heur", "true", "--p2heur", "true"],
    "p1opt-p2man": ["--p1heur", "opt", "--p2heur", "manhattan"],
}

EPS_RECIPES = {
    "eps1": ["--p1eps", "1", "--p2eps", "1"],
    "eps2": ["--p1eps", "2", "--p2eps", "2"],
    "eps4": ["--p1eps", "4", "--p2eps", "4"],
}

TURN_RECIPES = {
    "tc0": ["--turncost", "0"],
    "tc64": ["--turncost", "64"],
    "tc256": ["--turncost", "256"],
}


def one(maze: Path, extra):
    import time
    try:
        t0 = time.perf_counter()
        p = subprocess.run([str(RUN), "--maze", str(maze), "--json", *extra, *EXTRA],
                           capture_output=True, text=True, timeout=60)
        dt = time.perf_counter() - t0
        j = json.loads(p.stdout)
        j["ms"] = dt * 1000.0
    except Exception:
        return None
    return j


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mazes", type=Path, default=ROOT / "mazes" / "mazefiles" / "classic")
    ap.add_argument("--max", type=int, default=0, help="limit number of mazes")
    ap.add_argument("--sweep", choices=["heur", "eps", "turn"], default="heur")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--extra", nargs="*", default=[],
                    help="extra CLI args passed to every run (e.g. --turncost 0)")
    args, extra_args = ap.parse_known_args()
    global EXTRA
    EXTRA = args.extra + extra_args

    mazes = sorted(args.mazes.glob("*.txt"))
    if args.max:
        mazes = mazes[: args.max]
    # skip known-broken files
    mazes = [m for m in mazes if m.name not in ("001.txt", "001-anomaly-test.txt")]

    recipes = {"heur": RECIPES, "eps": EPS_RECIPES, "turn": TURN_RECIPES}[args.sweep]
    combos = [(rname, extra, m) for rname, extra in recipes.items() for m in mazes]

    with cf.ThreadPoolExecutor(args.jobs) as ex:
        futs = {ex.submit(one, m, extra): (rname, m) for rname, extra, m in combos}
        results = {}
        for f in cf.as_completed(futs):
            rname, m = futs[f]
            results[(rname, m.name)] = f.result()

    print(f"{'recipe':16s} {'n':>5s} {'ok':>5s} {'avgScore':>9s} {'medScore':>9s} "
          f"{'avgDist':>8s} {'avgEff':>8s} {'avgTrn':>7s} {'avgTrps':>7s} "
          f"{'avgRpl':>7s} {'avgExp':>8s} {'avgMs':>7s}")
    for rname, extra in recipes.items():
        rows = [results[(rname, m.name)] for m in mazes]
        solid = [r for r in rows if r and r["ok"]]
        scores = sorted(r["score"] for r in solid)
        n = len(solid)
        avg = lambda k: sum(r[k] for r in solid) / n if n else 0
        med = scores[n // 2] if n else 0
        print(f"{rname:16s} {len(mazes):5d} {n:5d} {avg('score'):9.2f} {med:9.2f} "
              f"{avg('total_dist'):8.1f} {avg('total_eff'):8.1f} "
              f"{avg('total_turns'):7.1f} {avg('trips'):7.1f} "
              f"{avg('replans'):7.1f} {avg('astar_exp'):8.1f} {avg('ms'):7.0f}")


if __name__ == "__main__":
    main()
