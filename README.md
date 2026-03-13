# Subastral — GPU-Accelerated Bundle Adjustment & SLAM

A from-scratch GPU-accelerated Bundle Adjustment and Pose-Graph SLAM system in C++ with
CUDA. All core math (Jacobians, Schur complement, sparse Cholesky, LM solver, loss
functions, SE(3) Lie group operations) is implemented from the ground up — no
Ceres/GTSAM wrapping.

## Build

Requires Docker with NVIDIA CUDA 11.8 support. Uses Bazel 7.4.1 with Clang/LLD.

```bash
make build TARGET=//subastral:subastral
```

## Usage

```bash
make run TARGET=//subastral:subastral ARGS="<path_to_dataset> [options]"
```

### Options

| Flag | Description | Default |
|---|---|---|
| `--pose-graph` | Pose-Graph SLAM mode (g2o format) | off (BA mode) |
| `--cpu` | Use CPU solver instead of GPU | GPU |
| `--lie` | Use SE(3) Lie group parameterization for poses | off (angle-axis) |
| `--loss <type>` | Loss function: `trivial`, `huber`, `cauchy` | `trivial` |
| `--loss-param <v>` | Loss parameter (delta for Huber, c for Cauchy) | `1.0` |
| `--max-iter <n>` | Maximum LM iterations | `50` |
| `--lambda <v>` | Initial LM damping | `1e-3` |
| `--quiet` | Suppress per-iteration output | off |
| `--viz <path>` | Save before/after visualization PNG | off |
| `--rerun-save <p>` | Save Rerun recording to .rrd file | off |
| `--rerun-connect [addr]` | Stream to Rerun viewer via gRPC | off |

### Examples

```bash
# Bundle Adjustment — GPU solver on a BAL dataset
make run TARGET=//subastral:subastral ARGS="/data/se/bal_datasets/Dubrovnik/problem-16-22106-pre.txt --max-iter 50"

# Bundle Adjustment — CPU solver with Huber loss
make run TARGET=//subastral:subastral ARGS="/data/se/bal_datasets/Ladybug/problem-49-7776-pre.txt --cpu --loss huber --loss-param 1.0"

# Bundle Adjustment — GPU solver with SE(3) Lie group parameterization
make run TARGET=//subastral:subastral ARGS="/data/se/bal_datasets/Dubrovnik/problem-88-64298-pre.txt --lie"

# Pose-Graph SLAM — GPU solver on a g2o dataset
make run TARGET=//subastral:subastral ARGS="/data/se/g2o_datasets/parking-garage.g2o --pose-graph"

# Pose-Graph SLAM — with Rerun visualization
make run TARGET=//subastral:subastral ARGS="/data/se/g2o_datasets/torus3D.g2o --pose-graph --rerun-connect"
```

## Tests

```bash
make test TARGET=//subastral/...
```

All 13 test targets (109 individual tests) cover:
- Analytical Jacobians: CPU finite-difference validation (6 tests) + GPU-vs-CPU consistency (3 tests)
- Lie Jacobians: CPU finite-difference validation (4 tests) + GPU-vs-CPU consistency (2 tests)
- SO(3) exp/log/hat/vee/leftJacobian: CPU (16 tests) + GPU-vs-CPU (3 tests)
- SE(3) exp/log/hat/vee/adjoint/compose: CPU (18 tests) + GPU-vs-CPU (4 tests)
- Schur complement: JtJ accumulation, Schur-vs-dense solve, symmetry (3 tests)
- LM solver: convergence, cost decrease, no-obs edge case, larger problems (7 CPU + 12 GPU tests incl. Lie)
- Pose-Graph solver: Torus3D, parking-garage, fixed-vertex (3 GPU tests)
- g2o loader: file parsing, memory consistency, info matrix permutation (6 tests)
- Loss functions: Trivial, Huber, Cauchy properties (14 tests)
- Data structures: Camera, Point, Observation memory layout (4 tests)

## Architecture

### Phases Completed

1. **Analytical Jacobians** — Fused residual + Jacobian computation via chain rule through
   Rodrigues rotation, translation, perspective divide, and radial distortion. CPU reference
   implementation validated against central finite differences, GPU kernel validated against CPU.

2. **Schur Complement + LM Solver** — Block-sparse normal equations (U, V, W blocks),
   Schur complement point elimination to dense camera-only system, dense Cholesky solve,
   Nielsen lambda update. Full GPU pipeline: GPU Jacobians, GPU normal equation
   accumulation, GPU Schur complement formation, GPU dense Cholesky via cuSOLVER
   (`cusolverDnDpotrf` + `cusolverDnDpotrs`). When Cholesky fails (matrix not SPD at
   low damping), lambda is increased and retried entirely on GPU — no CPU fallback.

3. **Robustification** — TrivialLoss, HuberLoss, CauchyLoss with IRLS (Iteratively
   Reweighted Least Squares) weighting on both CPU and GPU.

4. **SE(3) Lie Group Infrastructure** — Full SO(3) and SE(3) exp/log maps, hat/vee
   operators, adjoint, left/right Jacobians, and their inverses. CPU reference
   implementations validated with 41 unit tests, GPU device functions validated against
   CPU. Lie-group Jacobians use left-perturbation model `T_new = Exp(delta_xi) * T_old`,
   giving simpler pose columns (`-[P_cam]x` instead of Rodrigues chain rule). Camera
   parameter update uses SE(3) composition on the pose (left-multiply by `Exp(delta_xi)`)
   with additive update on intrinsics. Integrated into the full LM solver via `--lie` flag.

5. **Pose-Graph SLAM** — Full GPU pipeline for 3D pose-graph optimization. Loads g2o
   format datasets (`VERTEX_SE3:QUAT` + `EDGE_SE3:QUAT`). Poses stored as 7 doubles
   `(x, y, z, qx, qy, qz, qw)` with Hamilton quaternion convention. Jacobians use the
   correct `J_r^{-1}` formula with SE(3) adjoint. The Hessian is assembled directly as a
   sparse CSR matrix (lower triangle only, ~half the nnz of the full matrix) and solved
   via `cusolverSpDcsrlsvchol` (GPU sparse Cholesky). Pose updates use the SE(3)
   exponential map. No Schur complement is needed for pure pose-graphs since there is no
   bipartite pose-landmark structure. Rerun visualization shows initial/optimized poses,
   edges colored by per-edge residual error, and trajectory line strips.

### Parameterization

**Bundle Adjustment — Default (angle-axis):**
- Cameras: angle-axis (3) + translation (3) + focal length + k1 + k2 = **9 parameters**
- Points: X, Y, Z = **3 parameters**
- Update: additive `cam += delta`, `pt += delta`

**Bundle Adjustment — Lie group (`--lie` flag):**
- Same 9-parameter storage, but pose update uses SE(3) composition:
  1. Reconstruct `T_old = (Exp_SO3(omega), t)` from stored angle-axis + translation
  2. Compose `T_new = Exp_SE3(delta_xi) * T_old` (left perturbation, `delta_xi` is the 6D pose delta)
  3. Write back `Log_SO3(R_new)` and `t_new` to camera parameters
- Intrinsics (`f, k1, k2`) and points still use additive update
- Jacobian uses `d(P_cam)/d(delta_phi) = -[P_cam]x` (simpler than Rodrigues chain rule)

**Pose-Graph SLAM:**
- Poses: `(x, y, z, qx, qy, qz, qw)` = **7 parameters** (position + Hamilton quaternion)
- 6-DOF tangent space per pose (3 rotation + 3 translation)
- Update: `T_new = Exp_SE3(delta_xi) * T_old` (left perturbation)
- Jacobians: `dE/dxi_i = -J_r^{-1}(e) * Ad(T_j^{-1})`, `dE/dxi_j = J_r^{-1}(e) * Ad(T_j^{-1})`
- First pose is fixed (gauge freedom anchor)

### Key Design Decisions

- **Solver-local GPU state** — `common.h` stays CUDA-free; the GPU solver owns a
  `GPUSolverState` struct with `thrust::device_vector`s, uploads problem data once,
  runs the full LM loop on-GPU, syncs back at the end.
- **ODR-safe compilation** — CPU projection functions live in `.cpp` files compiled only
  by clang. GPU kernels live in `.cu` files compiled only by nvcc. CUDA-free headers
  (`.h`) alongside CUDA headers (`.cuh`) prevent ABI mismatches.
- **AVX alignment safety** — `EIGEN_MAX_ALIGN_BYTES=16` is forced globally to prevent
  alignment mismatches between clang (AVX2, 32-byte) and nvcc (SSE, 16-byte) TUs.

## Benchmark Results

Tested on BAL (Bundle Adjustment in the Large) datasets. Hardware: NVIDIA GPU (sm_75)
+ x86_64 CPU in Docker container.

### GPU with cuSOLVER (current)

Full GPU pipeline: Jacobians, normal equations, Schur complement, and dense Cholesky
solve all on GPU via cuSOLVER.

| Dataset | Cameras | Points | Observations | Wall Time | Iterations | Cost Reduction | Final RMS (px) |
|---|---|---|---|---|---|---|---|
| Ladybug-49 | 49 | 7,776 | 31,843 | **0.29s** | 4 | 86.5% | 2.68 |
| Ladybug-73 | 73 | 11,032 | 46,122 | **0.23s** | 9 | 98.2% | 0.86 |
| Ladybug-138 | 138 | 19,878 | 85,217 | **0.90s** | 22 | 97.1% | 1.21 |
| Dubrovnik-16 | 16 | 22,106 | 83,718 | **0.16s** | 16 | 99.6% | 0.66 |
| Dubrovnik-88 | 88 | 64,298 | 383,937 | **0.67s** | 9 | 99.0% | 1.24 |

### Speedup vs Previous (GPU + Eigen CPU LDLT)

Moving the dense Cholesky solve from CPU Eigen LDLT to GPU cuSOLVER eliminated the
main bottleneck for larger problems.

| Dataset | GPU+Eigen | GPU+cuSOLVER | cuSOLVER Speedup | vs CPU-only |
|---|---|---|---|---|
| Ladybug-49 | 3.3s | 0.29s | **11x** | **48x** (vs 13.8s) |
| Ladybug-73 | 9.1s | 0.23s | **40x** | **209x** (vs 48.0s) |
| Ladybug-138 | 116.9s | 0.90s | **130x** | **118x** (vs 106.5s) |
| Dubrovnik-16 | 0.4s | 0.16s | **2.5x** | **495x** (vs 79.2s) |
| Dubrovnik-88 | 51.5s | 0.67s | **77x** | — |

### GPU with Lie Group Parameterization (`--lie`)

Same accuracy as the default solver, with faster wall times due to simpler Jacobian
computation (no Rodrigues chain rule needed for pose columns).

| Dataset | Cameras | Points | Observations | Wall Time | Iterations | Cost Reduction | Final RMS (px) |
|---|---|---|---|---|---|---|---|
| Ladybug-49 | 49 | 7,776 | 31,843 | **1.59s** | 50 | 98.4% | 0.92 |
| Ladybug-73 | 73 | 11,032 | 46,122 | **0.59s** | 23 | 98.2% | 0.86 |
| Dubrovnik-88 | 88 | 64,298 | 383,937 | **5.59s** | 50 | 99.1% | 1.23 |

### Performance Characteristics

- **cuSOLVER eliminates the dense solve bottleneck.** The previous GPU pipeline was
  limited by CPU Eigen LDLT on the Schur complement. With cuSOLVER, even Ladybug-138
  (1242x1242 Schur complement) solves in under 1 second.

- **Largest speedups on medium-to-large camera counts.** Ladybug-138 went from 117s to
  0.9s (130x) because the O(n_cam^3) Cholesky now runs on GPU. Small problems like
  Dubrovnik-16 (144x144 Schur complement) see smaller gains (2.5x) since the dense
  solve was already fast.

- **Cholesky failure handling is robust.** When the Schur complement is not SPD at low
  damping (common near convergence), the solver increases lambda and retries on GPU.
  No data transfer to CPU is needed.

### Pose-Graph SLAM (GPU Sparse Cholesky)

Full GPU pipeline: per-edge residuals/Jacobians, CSR Hessian assembly, sparse Cholesky
solve via `cusolverSpDcsrlsvchol`, SE(3) pose updates — all on GPU. Tested on standard
g2o 3D pose-graph datasets.

| Dataset | Poses | Edges | Loop Closures | Initial Cost | Final Cost | Reduction | Iterations | Wall Time | Status |
|---|---|---|---|---|---|---|---|---|---|
| parking-garage | 1,661 | 6,275 | 4,615 | 8,364 | 0.634 | 99.99% | 13 | **19.1s** | Converged |
| torus3D | 5,000 | 9,048 | 4,049 | 2,400,499 | 29,958 | 98.75% | 50 | **68.9s** | Max iter |
| sphere-bignoise | 2,200 | 8,647 | 6,448 | 165,904,000 | 3,537,910 | 97.87% | 50 | **10.2s** | Max iter |
| grid3D | 8,000 | 22,236 | 14,237 | 95,303,200 | 249,040 | 99.74% | 50 | **468.6s** | Max iter |
| cubicle | 5,750 | 16,869 | 7,621 | 5,405,430 | 5,405,430 | 0.00% | 1 | 110.6s | Failed |
| rim | 10,195 | 29,743 | 13,475 | 63,831,900 | 63,831,900 | 0.00% | 1 | 220.5s | Failed |

### Pose-Graph Performance Notes

- **parking-garage converges fully** to near-zero cost in 13 iterations — a well-conditioned
  dataset with dense loop closures relative to its size.

- **torus3D, sphere-bignoise, and grid3D** achieve large cost reductions (97-99%) but
  hit the max iteration limit. The cost is still decreasing slowly; more iterations or
  a preconditioned solver would help.

- **cubicle and rim fail** on the first iteration — the sparse Cholesky factorization
  (`cusolverSpDcsrlsvchol`) fails because the Hessian is not positive definite even with
  damping. These datasets have large initial noise and require either a more robust
  factorization (e.g., sparse LDL^T or iterative PCG) or a better initialization strategy.

- **Wall time is dominated by `cusolverSpDcsrlsvchol`** — the sparse Cholesky is called
  once per iteration. For grid3D (47,994 DOF, ~1.3M nnz), each solve takes ~9s. The
  kernel/assembly phases are fast by comparison.

## Project Structure

```
subastral/
  main.cpp                          CLI entry point
  pipeline.hpp                      Abstract Pipeline interface
  ba_pipeline.{hpp,cpp}             Bundle Adjustment pipeline
  pose_graph_pipeline.{hpp,cpp}     Pose-Graph SLAM pipeline
  subastral.{hpp,cpp}               Top-level Subastral class (factory + delegation)
  backend/
    common.h                        Umbrella header (includes graph/)
    graph/
      memory_map.h                  FactorGraphMemoryMap + stride constants
      graph_entity.h                GraphEntity abstract base class
      ba_types.h                    Camera, Point, Observation, BAProblem
      pose_graph_types.h            Pose, PoseEdge, PoseGraphProblem
      landmark_types.h              Landmark, PoseLandmarkEdge
    solver/
      lm_solver.{hpp,cpp}           CPU LM solver (BA)
      lm_solver_gpu.{cuh,cu}        GPU LM solver (BA, dense Cholesky)
      pose_graph_solver.{cuh,cu}    GPU LM solver (pose-graph, sparse Cholesky)
      schur.hpp                     Schur complement elimination
      loss_function.hpp             Trivial, Huber, Cauchy loss
    ops/                            Jacobians, projection, error computation
    lie/                            SO(3) and SE(3) Lie group implementations
  loader/
    bal_loader.{h,cpp}              BAL dataset parser
    g2o_loader.{h,cpp}              g2o dataset parser (SE3:QUAT)
  viz/                              OpenCV before/after visualizer
viz/rerun/
  ba_visualizer.{hpp,cpp}           Rerun BA visualizer
  pg_visualizer.{hpp,cpp}           Rerun pose-graph visualizer
```

## Roadmap

- [x] Phase 1: Analytical Jacobians
- [x] Phase 2: Schur Complement + LM Solver
- [x] Phase 3: Robustification (Loss Functions)
- [x] cuSOLVER Integration (GPU dense Cholesky solve)
- [x] Phase 4: SE(3) Lie Group Infrastructure
- [x] Phase 5: Pose-Graph SLAM (GPU sparse Cholesky, g2o loader, Rerun visualization)
- [x] Rerun Integration (3D visualization for BA and pose-graph)
- [x] Pipeline Refactoring (Strategy pattern for BA / Pose-Graph)
- [ ] PCG Solver — cubicle/rim fail because `cusolverSpDcsrlsvchol` requires SPD; PCG handles indefinite systems and avoids O(n^3) factorization bottleneck on large graphs
- [ ] Robust Loss for Pose-Graph — Huber/Cauchy IRLS is implemented for BA but not wired into pose-graph; outlier loop closures can corrupt the solution
- [ ] Sparse LDL^T Factorization — direct-solve alternative to PCG for non-SPD Hessians (cubicle/rim datasets)
- [ ] Landmark-Based SLAM — extend pose-graph with pose-landmark edges (types defined in `landmark_types.h`); requires Schur complement on combined pose-landmark Hessian
- [ ] 2D Pose-Graph Support — add `VERTEX_SE2`/`EDGE_SE2` g2o parsing and 3-DOF solver; many standard benchmarks (Intel, MIT, Manhattan) are 2D
- [ ] Phase 6: Visual Front-End — feature detection/matching (ORB/FAST), triangulation, PnP for going from offline optimizer to online SLAM
- [ ] IMU Preintegration — fuse IMU between keyframes (Forster et al.) for visual-inertial SLAM
- [ ] Phase 7: Incremental SLAM — online graph construction with sliding window or iSAM2-style incremental updates for real-time operation
- [ ] Phase 8: ROS Integration
- [ ] GPU Profiling & Optimization — Nsight profiling, optimize CSR assembly, reduce host-device transfers, benchmark against g2o/GTSAM

---

## Appendix: Human-in-the-Loop — Where Manual Input Mattered

This project was built collaboratively between a human engineer and an AI coding agent
(Claude). This appendix documents where human domain expertise made a material difference
to correctness and design quality, vs where the AI agent's defaults were sufficient.

### Algorithmic Corrections

These are cases where the AI's initial implementation was mathematically wrong or would
have produced a solver that compiles and runs but gives incorrect results.

**1. Pose-Graph Jacobian Formula (Critical)**

The AI initially implemented:
```
dE/dxi_i = -J_l^{-1}(e) * Ad(T_j^{-1} * T_i)
dE/dxi_j =  J_l^{-1}(e)
```

The correct formula (provided by the human) is:
```
dE/dxi_i = -J_r^{-1}(e) * Ad(T_j^{-1})
dE/dxi_j =  J_r^{-1}(e) * Ad(T_j^{-1})
```

Key differences: `J_r^{-1}` not `J_l^{-1}`, and the adjoint is of `T_j^{-1}` alone
(not `T_j^{-1} * T_i`). The derivation comes from applying the BCH approximation when
converting a left perturbation to a right perturbation of the error term. Wrong Jacobians
cause the solver to either diverge or converge to an incorrect solution.

**2. CSR Matrix Format for cusolverSpDcsrlsvchol**

The human specified:
- Must use `CUSPARSE_MATRIX_TYPE_GENERAL` (not `SYMMETRIC`)
- With `reorder=0`, the solver reads only the lower triangle automatically
- With `reorder=1`, the full matrix is required (we use `reorder=0`)
- Lower-triangle-only CSR halves the nnz storage and assembly work

Getting the matrix type or reorder flag wrong would cause silent numerical errors or
crashes. These are underdocumented cuSOLVER API details that require hands-on experience.

**3. No Schur Complement for Pure Pose-Graphs**

The AI's BA solver uses the Schur complement to eliminate landmark variables (bipartite
pose-landmark structure). The human clarified that pure pose-graphs have no bipartite
structure — there are no landmarks to eliminate. The Hessian is one sparse symmetric
matrix, solved directly with sparse Cholesky. Applying a Schur complement here would
have been fundamentally wrong.

**4. g2o Information Matrix Ordering**

The human identified that g2o stores the 6x6 information matrix in (translation, rotation)
block ordering, but the solver's internal convention is (rotation, translation). The
loader must permute the rows and columns. Without this, the information matrix weights
would be applied to the wrong residual components, producing silently incorrect results.

### Design-Level Decisions

These shaped the architecture and were provided as explicit requirements.

**5. GPU-First Architecture**

The human specified "full GPU pipeline — maximum GPU utilization, GPU is the main use
case, not CPU." This shaped the entire solver: CSR assembly on GPU, sparse Cholesky on
GPU, pose updates on GPU. Without this directive, the AI would likely have built a
CPU solver with optional GPU acceleration.

**6. Pose Storage Convention**

The human specified poses as 7 doubles `(x, y, z, qx, qy, qz, qw)` — Hamilton
quaternion with w last, matching g2o convention. Quaternion convention (w-first vs w-last)
is a classic source of silent sign errors in robotics code.

**7. Rerun Visualization Design**

Multiple specific visualization decisions were human-directed:
- Each edge must be a **separate line segment** (not batched strips) with per-edge
  color and per-edge label showing the error value
- Color ramp: odometry edges blue-to-yellow, loop closure edges green-to-red
- **Sqrt normalization** for error-to-color mapping (better visual contrast than linear)
- Initial and optimized states as **separate static entities** (both visible, togglable)

Without these inputs, the AI would have likely batched all edges into one `LineStrips3D`
with uniform color — functional but far less informative for debugging.

### Structural Decisions

These came up during code review and refactoring sessions.

**8. Pipeline Split into Separate Files**

The AI initially placed both `BAPipeline` and `PoseGraphPipeline` in a single
`subastral.cpp`. The human rejected this and requested separate files
(`ba_pipeline.{hpp,cpp}` + `pose_graph_pipeline.{hpp,cpp}`). This gives each pipeline
independent compilation and its own minimal dependency set.

**9. Graph Types Directory**

The human suggested splitting the monolithic `common.h` (495 lines) into
`subastral/backend/graph/` with separate headers per type family. The AI designed the
internal split (5 headers + umbrella), and the human chose the backward-compatible
umbrella approach over updating all 24 consumers.

### Summary

| Category | Decision | Source | Severity if Wrong |
|---|---|---|---|
| Math | Jacobian formula (`J_r^{-1}` + adjoint) | Human | Critical — wrong convergence |
| Math | Info matrix (trans,rot) to (rot,trans) permutation | Human | High — silent wrong results |
| Math | No Schur complement for pure pose-graphs | Human | High — wrong decomposition |
| GPU API | CSR format: GENERAL type, reorder=0, lower-triangle | Human | High — crash or wrong results |
| Architecture | GPU-first solver design | Human | Medium — different product |
| Convention | Quaternion w-last (Hamilton, g2o-compatible) | Human | Medium — silent sign errors |
| Visualization | Per-edge coloring, sqrt normalization, separate entities | Human | Low — usability |
| Structure | Separate pipeline files | Human | Low — code organization |
| Structure | Graph types directory + umbrella header | Both | Low — code organization |
| Implementation | BUILD targets, dependency analysis, 5-header split | AI | Low — code organization |
| Implementation | All kernel/solver code, test harnesses, CLI wiring | AI | N/A — primary output |

The pattern: human inputs were highest-impact on **mathematical correctness** and
**GPU API details** — areas where errors produce code that compiles and runs but gives
wrong answers. The AI handled **implementation volume** (kernels, tests, build system,
visualization code) reliably once the mathematical and API foundations were correct.
