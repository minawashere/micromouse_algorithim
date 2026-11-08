# micromouse_algo

C++ micromouse maze solver and benchmark harness for the standard
16×16 competitive maze 

The same solver drives three frontends:

| Frontend | What it is |
|---|---|
| `build/run` | Offline command-line simulator: solve a maze from the corpus, print score, run statistics, and solver internals (`--json` for machine-readable output). |
| `build/mms` | Drop-in wrapper for the [mms](https://github.com/mackorone/mms) simulator GUI: speaks the mms stdin/stdout protocol, so the GUI's rendering & animation drive the same solver. |
| `build/test_core` | Unit smoke tests (maze parsing, A*/dijkstra, wall symmetry, end-to-end solve). |

## How the solver works

1. **Explore** — sense the 3 faces of the current cell, write them
   symmetrically into the wall map (learning one side of a wall
   immediately knows its twin), and seal confidently-dead ends out of
   planning.
2. **Plan** — weighted-A* over the known map with turn cost
   (`--turncost 70` = 0.275 cell-time per 90° turn) and 2×-weighted greed,
   replanning after every surprise. Optionally
   `--enable-diagonal` plans diagonal corridors as alternating L/R
   stair-steps priced as one sine-wave slalom (`--zigzag-cost`, default
   64 — never cheaper than a straight cell).
3. **Decide** — judge whether more exploration is worth the cells spent.
   By default exploration runs until the known sprint cost is *provably
   unbeatable* (exact best-path certificate; the `cert_*` JSON fields).
4. **Sprint** — execute the best run found. Goal cells absorb: a run ends
   at the first center cell it touches — never wandering inside the goal
   region.

Three personalities cover the spectrum: the **exact 4-phase machine**
(default), a **flood-fill mood** (`--floodfill` — exploration descends a
directional kinematic flood, so step costs include turn and u-turn
penalties, with a curiosity weight on unexplored faces), and the
competitive **5-stage mix-mode machine** (`--mix-mode`: SearchCenter →
OptimizedReturn → Sprint1 → DeepExplore → Sprint2), which locks the map
and sprints early, then re-opens exploration only inside an improvement
band.

## Repository layout

```
include/mm/    solver core (header-only): solver, A*, worlds, heuristics
src/apps/      run.cpp (CLI benchmark), mms_main.cpp (mms GUI protocol wrapper)
src/lib/       maze file loader
tests/         test_core.cpp smoke tests
mazes/         maze file collection (git submodule → micromouseonline/mazefiles)
scripts/       bench.py (corpus benchmark), mms_wrap.sh (protocol-logging wrapper)
```

## Build

### Configure

```sh
cmake -S . -B build                    # Release is the default build type
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug   # or explicitly Debug
```

### Build all targets at once

```sh
cmake --build build -j4                # or -j$(nproc)
```

### Build individual targets

Each frontend is its own CMake target; build only what you need:

| Target | Command | Produces |
|---|---|---|
| `mm_core` | `cmake --build build --target mm_core` | `libmm_core` static library (maze loader) that every frontend links |
| `run` | `cmake --build build --target run` | `build/run` — offline benchmark CLI |
| `mms` | `cmake --build build --target mms` | `build/mms` — mms GUI protocol wrapper |
| `test_core` | `cmake --build build --target test_core` | `build/test_core` — unit smoke tests |

For example, to build just the mms wrapper:

```sh
cmake --build build --target mms -j4
```

### Clean / full rebuild

```sh
cmake --build build --target clean   # remove build products, keep cache
rm -rf build && cmake -S . -B build  # from-scratch reconfigure
```

After building, verify with `./build/test_core` (must print `ALL TESTS
PASSED`).

## mazes (submodule only)

No maze files are committed to this repository. `mazes/mazefiles` is
referenced *exclusively* as a git submodule of
[github.com/micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles)
(525 classic mazes plus half-size variants): the recorded gitlink is the
only maze-related artifact in git, and the files materialize under
`mazes/mazefiles/` at clone time via the submodule checkout.

```sh
git clone --recurse-submodules https://github.com/minawashere/micromouse_algorithim
# ... or, in an existing clone:
git submodule update --init
# refresh the maze corpus to the latest upstream master:
git submodule update --remote mazes/mazefiles
```

## `run` — offline benchmark CLI

```sh
# Exact 4-phase machine (optimal-trip planner; the default mode)
./build/run --maze mazes/mazefiles/classic/tricky.txt

# Winner-style run: nearest-candidate sweep exploration + diagonal slalom
# planning with a discounted slalom-turn price
./build/run --maze mazes/mazefiles/classic/japan2023-student.txt --sweep --enable-diagonal --zigzag-cost 64

# Flood-fill mood, the benchmark configuration
./build/run --maze mazes/mazefiles/classic/taisho.txt --floodfill --ff-straight --ff-legs 2

# Competitive 5-stage mix-mode machine
./build/run --maze mazes/mazefiles/classic/killer.txt --mix-mode --ff-straight

# Machine-readable output
./build/run --maze mazes/mazefiles/classic/killer.txt --floodfill --ff-straight --json
```

Modes (mutually exclusive):

| Mode | Description |
|---|---|
| *(default)* | Exact planner: weighted-A* optimal trips with sense-and-replan, normal & reverse situations. |
| `--floodfill` | Flood-fill mood: exploration descends a directional kinematic flood; best discovered run becomes the sprint. |
| `--mix [--mix-mode]` | Competitive 5-stage machine: SearchCenter → OptimizedReturn → Sprint1 (locked map, no exploration) → DeepExplore (improvement-band trips) → Sprint2 (from start). |

### Parameters

| Flag | Default | Meaning |
|---|---|---|
| `--maze <file>` | *(required)* | Maze file (`MAZE` text format, from the mazes corpus). |
| `--json` | off | Machine-readable one-line JSON result. |
| `--check` | off | Validate a maze file: print parse dump + hash, then exit. |
| `--verbose` | off | Live console visualization of the run. |
| `--p1heur K` | `opt` | Phase 1 heuristic: `manhattan` \| `true` \| `opt`. |
| `--p2heur K` | `opt` | Phase 2 heuristic kind (same values). |
| `--turncost N` | 70 | A* turn cost (70 = 0.275 cell-time per 90° turn). |
| `--enable-diagonal` | off | Slalom planning: diagonal corridors are planned as alternating L/R stair-steps whose turn pair is charged `--zigzag-cost` (the motor controller drives the pair as one continuous sine-wave). Applies inside every mode (exact, flood, mix). Mice execute plain orthogonal moves. |
| `--zigzag-cost N` | 64 | Price of ONE alternating slalom turn; clamped ≥ 0. `< turnCost` ranks stair-steps over 90° zig-zags, `> 0` keeps them never cheaper than a straight (staircases can't beat a straight line of equal endpoints). |
| `--sweep` | off | Nearest-candidate sweep exploration: per pass, plans start→center on the optimistic map (lower bound `LIM`) and pessimistic map (executable `bestP`). Chases qualifying unknown faces within the optimistic bound, ordered by driving distance from current pose. Certifies optimal sprint when `bestP.cost == LIM` or falls back to return-home servicing. |
| `--floodfill` | off | Enable the flood-fill mood (see modes above). |
| `--ff-straight` | off | Flood walker tie-break: prefer going straight. |
| `--ff-avoid` | off | Flood walker tie-break: avoid (secondary preference). |
| `--ff-curio N` | 64 | Flood curiosity weight on unexplored faces (0 = optimistic flood), for center-bound legs. |
| `--ff-legs N` | 2 | Exploration legs; even ≥ 2 (odd coerced down). Ends the walk at the start so the sprint begins immediately. |
| `--mix` / `--mix-mode` | off | Enable the 5-stage mix-mode machine. |
| `--look-ahead` | **off** | Real-hardware feature: 2-cell IR rig (one cell ahead via front sensors + diagonal reads). Default OFF — keep OFF for simulators; enables extra sensor reads in the solver loop. |
| `--look-ahead-diag` | **off** | Diagonal channels of that rig: map the remote cell's left/right faces while the cell ahead is open. Default OFF. |
| `--debug` | off | Firmware-style critical-leg debug prints. |
| `--trace` / `--trace2` | off | Extra solver trace output. |

Unknown arguments are an error (exit code 2); missing `--maze` prints usage.

### Output

Text mode prints `score`, `total`, `best-run`, `sprint`, and A* statistics.
`--json` prints one line with fields `ok`, `score`, `total_dist`,
`total_eff`, `total_turns`, `best_run_dist`, `best_run_eff`,
`best_run_turns`, `sprint_dist`, `sprint_turns`, `astar_calls`,
`astar_exp`, `trips`, `replans`, `cert_at_cells`, `cert_known_cost`,
`cert_bound`. Score accounting is done by the offline simulator (the same
numbers a real mms-style GUI reports); benchmark comparisons in this repo
use `best_run_eff + best_run_turns`.

### Reference results (classic corpus)

| Maze | exact | exact + diag | floodfill (ff-legs 2) | ff + diag | mix-mode | mix + diag |
|---|---|---|---|---|---|---|
| `tricky.txt` | 211.65 | 211.65 | 190.45 | 189.15 | 223.35 | 223.95 |
| `taisho.txt` | 94.50 | 89.45 | 65.80 | 68.85 | 106.35 | 99.30 |
| `killer.txt` | 184.80 | 184.20 | 173.50 | 167.10 | 205.95 | 198.95 |
| `allamerica2013.txt` | 103.35 | 102.80 | 99.60 | 99.60 | 111.50 | 110.95 |

All cells measured with the shipped defaults of this revision: exact
and mix columns inherit the new `--turncost 70`, which slightly
re-ranks turn-heavy routes vs the older tc64 snapshot; flood rows are
tc-independent. Diagonal cells add `--enable-diagonal --zigzag-cost
64`. The discount is a per-maze tuning knob: net corpus effect is
mildly positive (523-maze mean −0.30), with individual mazes above or
below parity.

## `mms` — mms simulator GUI wrapper

[Build the mms simulator](https://github.com/mackorone/mms#building), set its
maze file, and point **Run command** at this binary (all tuning flags are
identical to `run`; the maze itself comes from the GUI over stdin/stdout):

```
Run command: <abs path>/build/mms --floodfill --ff-straight --ff-curio 64
```

or for mix-mode: `./build/mms --mix-mode`.

Note: the mms terminal protocol exposes only the standard 3-face rig, so
`--look-ahead` / `--look-ahead-diag` are accepted but inert there
(no extra protocol commands are issued).

## Tests

```sh
./build/test_core
```

Maze parsing, A*/dijkstra correctness, shared-wall symmetry, and an end-to-end
spiral solve. Assertions are compiled live even in Release (deliberate
`#undef NDEBUG` in `tests/test_core.cpp` — do not remove).

## Scripts

- `scripts/bench.py` — batch-run `./build/run --json` over a maze directory
  and summarize.
- `scripts/mms_wrap.sh` — convenience wrapper for mms runs that also logs the
  full protocol to files for offline debugging.

## License

MIT — see [LICENSE](LICENSE). The maze corpus in the `mazes/mazefiles`
submodule comes from its own upstream repository
([micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles));
see that repository for maze file terms.

## Development notes

`HANDOFF.md` (not tracked) documents the solver invariants, flood
implementation internals, validation harnesses, and the current
performance baseline for continued development.
