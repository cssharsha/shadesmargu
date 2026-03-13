#include <pthread.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "subastral/subastral.hpp"

// =============================================================================
// Usage
// =============================================================================
//
// Basic:
//   subastral <path_to_dataset>
//
// With options:
//   subastral <path_to_dataset> [options]
//
// Options:
//   --pose-graph       Pose-Graph SLAM mode (g2o format, default: BA)
//   --cpu              Use CPU solver instead of GPU (default: GPU)
//   --lie              Use SE(3) Lie group parameterization
//   --loss <type>      Loss function: trivial, huber, cauchy (default: trivial)
//   --loss-param <v>   Loss parameter: delta for Huber, c for Cauchy
//   (default: 1.0)
//   --max-iter <n>     Maximum LM iterations (default: 50)
//   --lambda <v>       Initial LM damping (default: 1e-3)
//   --quiet            Suppress per-iteration output
//   --viz <path>       Save before/after visualization PNG to <path>
//   --rerun-save <p>   Save Rerun recording to .rrd file
//   --rerun-connect [addr]  Stream to Rerun viewer (default:
//   rerun+http://127.0.0.1:9876/proxy)
//
// =============================================================================

static void printUsage(const char* prog) {
  std::cerr
      << "Usage: " << prog << " <dataset> [options]\n"
      << "\nModes:\n"
      << "  (default)          Bundle Adjustment (BAL format)\n"
      << "  --pose-graph       Pose-Graph SLAM (g2o format)\n"
      << "\nOptions:\n"
      << "  --cpu              Use CPU solver (default: GPU)\n"
      << "  --lie              Use SE(3) Lie group parameterization\n"
      << "  --loss <type>      trivial|huber|cauchy (default: trivial)\n"
      << "  --loss-param <v>   Loss parameter (default: 1.0)\n"
      << "  --max-iter <n>     Max LM iterations (default: 50)\n"
      << "  --lambda <v>       Initial lambda (default: 1e-3)\n"
      << "  --quiet            Suppress per-iteration output\n"
      << "  --viz <path>       Save before/after PNG (default: off)\n"
      << "  --rerun-save <p>   Save Rerun recording to .rrd file\n"
      << "  --rerun-connect <addr>  Stream to Rerun viewer via gRPC\n"
      << "                     (default: rerun+http://127.0.0.1:9876/proxy)\n";
}

// -----------------------------------------------------------------------------
// Thread trampoline for running the solver with a large stack.
//
// On Linux, setrlimit(RLIMIT_STACK) does NOT grow the main thread's stack —
// the kernel maps it at exec time and the limit only affects new threads /
// child processes.  The CPU solver's Eigen-heavy call chains (dense LDLT on
// the Schur complement, back-substitution, etc.) can exceed the default 8 MB
// stack on real BAL datasets.  We solve this by running the actual work on a
// pthread with an explicitly large stack (64 MB).
// -----------------------------------------------------------------------------

struct WorkerArgs {
  int argc;
  char** argv;
  int result;
};

static void* workerThread(void* arg) {
  auto* wa = static_cast<WorkerArgs*>(arg);
  int argc = wa->argc;
  char** argv = wa->argv;

  if (argc < 2) {
    printUsage(argv[0]);
    wa->result = 1;
    return nullptr;
  }

  std::string filename = argv[1];

  // Check for --help before treating first arg as filename
  if (filename == "--help" || filename == "-h") {
    printUsage(argv[0]);
    wa->result = 0;
    return nullptr;
  }

  substral::Subastral subastral;
  bool pose_graph = false;

  // Parse optional arguments
  for (int i = 2; i < argc; ++i) {
    if (std::strcmp(argv[i], "--pose-graph") == 0) {
      pose_graph = true;
    } else if (std::strcmp(argv[i], "--cpu") == 0) {
      subastral.config().use_gpu = false;
    } else if (std::strcmp(argv[i], "--lie") == 0) {
      subastral.config().solver.use_lie = true;
    } else if (std::strcmp(argv[i], "--loss") == 0 && i + 1 < argc) {
      ++i;
      if (std::strcmp(argv[i], "huber") == 0) {
        subastral.config().solver.loss_type =
            substral::backend::solver::LossType::HUBER;
      } else if (std::strcmp(argv[i], "cauchy") == 0) {
        subastral.config().solver.loss_type =
            substral::backend::solver::LossType::CAUCHY;
      } else if (std::strcmp(argv[i], "trivial") == 0) {
        subastral.config().solver.loss_type =
            substral::backend::solver::LossType::TRIVIAL;
      } else {
        std::cerr << "Unknown loss type: " << argv[i] << std::endl;
        wa->result = 1;
        return nullptr;
      }
    } else if (std::strcmp(argv[i], "--loss-param") == 0 && i + 1 < argc) {
      subastral.config().solver.loss_param = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--max-iter") == 0 && i + 1 < argc) {
      subastral.config().solver.max_iterations = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--lambda") == 0 && i + 1 < argc) {
      subastral.config().solver.initial_lambda = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--quiet") == 0) {
      subastral.config().solver.verbose = false;
    } else if (std::strcmp(argv[i], "--viz") == 0 && i + 1 < argc) {
      subastral.config().viz_output_path = argv[++i];
    } else if (std::strcmp(argv[i], "--rerun-save") == 0 && i + 1 < argc) {
      subastral.config().rerun.rrd_path = argv[++i];
    } else if (std::strcmp(argv[i], "--rerun-connect") == 0) {
      if (i + 1 < argc && argv[i + 1][0] != '-') {
        subastral.config().rerun.connect_addr = argv[++i];
      } else {
        // Default Rerun viewer address
        subastral.config().rerun.connect_addr =
            "rerun+http://127.0.0.1:9876/proxy";
      }
    } else {
      std::cerr << "Unknown option: " << argv[i] << std::endl;
      printUsage(argv[0]);
      wa->result = 1;
      return nullptr;
    }
  }

  // Create the appropriate pipeline
  if (pose_graph) {
    subastral.createPipeline("pose-graph");
  }

  // Load dataset
  if (!subastral.load(filename)) {
    std::cerr << "Failed to load dataset: " << filename << std::endl;
    wa->result = 1;
    return nullptr;
  }

  // Run optimization
  subastral.run();

  wa->result = 0;
  return nullptr;
}

int main(int argc, char** argv) {
  // ---------------------------------------------------------------------------
  // Spawn the real work on a pthread with a 64 MB stack.
  //
  // The default 8 MB stack is insufficient for the CPU solver on real BAL
  // datasets — deep Eigen template call chains (LDLT, triangularSolve, etc.)
  // accumulate stack frames beyond 8 MB.  setrlimit(RLIMIT_STACK) cannot
  // grow the main thread's stack on Linux, so we use pthread_create with
  // pthread_attr_setstacksize instead.
  // ---------------------------------------------------------------------------
  constexpr size_t kStackSize = 64ULL * 1024 * 1024;  // 64 MB

  WorkerArgs wa{argc, argv, 0};

  pthread_t thread;
  pthread_attr_t attr;
  pthread_attr_init(&attr);

  if (pthread_attr_setstacksize(&attr, kStackSize) != 0) {
    std::cerr << "Warning: failed to set worker thread stack size\n";
  }

  int rc = pthread_create(&thread, &attr, workerThread, &wa);
  pthread_attr_destroy(&attr);

  if (rc != 0) {
    std::cerr << "Failed to create worker thread (rc=" << rc << ")\n";
    return 1;
  }

  pthread_join(thread, nullptr);
  return wa.result;
}
