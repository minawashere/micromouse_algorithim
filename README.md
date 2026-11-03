# micromouse_algo

C++17 micromouse maze solver: weighted-A* trip planning, directional
kinematic flood-fill exploration, dead-end sealing, and a competitive
5-stage mix-mode machine for a 16×16 maze with the center 2×2 as the goal.
Runs in three places: an offline benchmark CLI (`run`), the
[mms](https://github.com/mackorone/mms) simulator GUI (`mms`), and the unit
smoke test (`test_core`).

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

Requires only a C++17 toolchain and CMake ≥ 3.16.

```sh
cmake -S . -B build          # Release is the default build type
cmake --build build -j4      # or -j$(nproc)
./build/test_core            # must print: ALL TESTS PASSED
```

Produces `build/run`, `build/mms`, and `build/test_core`.

## mazes submodule

`mazes/mazefiles` is a git submodule of
[github.com/micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles)
(525 mazes: `classic/`, `halfsize/`).

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
| `--p1eps N` | 2 | Weighted-A* epsilon, phase 1 trips (1 = strict optimal). |
| `--p2eps N` | 2 | Weighted-A* epsilon, phase 2 trips. |
| `--p1heur K` | `opt` | Phase 1 heuristic: `manhattan` \| `true` \| `opt`. |
| `--p2heur K` | `opt` | Phase 2 heuristic kind (same values). |
| `--turncost N` | 64 | A* turn cost (64 = 0.25 cell-time per 90° turn). |
| `--vband X` | 10 | Mix-mode improvement band for DeepExplore trips. |
| `--floodfill` | off | Enable the flood-fill mood (see modes above). |
| `--ff-straight` | off | Flood walker tie-break: prefer going straight. |
| `--ff-avoid` | off | Flood walker tie-break: avoid (secondary preference). |
| `--ff-curio N` | 64 | Flood curiosity weight on unexplored faces (0 = optimistic flood). |
| `--ff-legs N` | 2 | Exploration legs; even ≥ 2 (odd coerced down). Ends the walk at the start so the sprint begins immediately. |
| `--mix` / `--mix-mode` | off | Enable the 5-stage mix-mode machine. |
| `--look-ahead` | **off** | Real-hardware feature: 2-cell IR rig (one cell ahead via front sensors + diagonal reads). Default OFF — keep OFF for simulators; enables extra sensor reads in the solver loop. |
| `--look-ahead-diag` | **off** | Diagonal channels of that rig: map the remote cell's left/right faces while the cell ahead is open. Default OFF. |
| `--trip-worth X` | −1 | Phase-2 trip economics ratio (−1 = exact best-path certificate; 1.0 = legacy `--allow-lazy`). |
| `--edge-prior P` | 1.0 | Open-wall prior applied to potential savings when trip-worth is active. |
| `--cluster-radius R` | 0 | Frontier tie-break neighborhood radius (0 = legacy same-cell tie-break). |
| `--allow-lazy` | — | Shorthand for `--trip-worth 1.0`. |
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

| Maze | exact | floodfill (ff-legs 2) | mix-mode |
|---|---|---|---|
| `tricky.txt` | 217.30 | 190.45 | 223.35 |
| `taisho.txt` | 89.45 | 65.80 | 105.20 |
| `killer.txt` | 183.40 | 173.50 | 205.35 |
| `allamerica2013.txt` | 103.35 | 99.60 | 111.50 |

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
