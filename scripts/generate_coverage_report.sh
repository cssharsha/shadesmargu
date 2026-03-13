#!/usr/bin/env bash
#
# Generate HTML coverage report from Bazel's combined lcov output.
# Runs inside the Docker container after `bazel coverage`.
#
set -euo pipefail

LCOV_FILE="$(bazel info output_path)/_coverage/_coverage_report.dat"

if [ ! -f "$LCOV_FILE" ]; then
    echo "Warning: No coverage report found at $LCOV_FILE"
    echo "Checking for individual coverage.dat files..."
    find "$(bazel info output_path)" -name "coverage.dat" 2>/dev/null | head -10
    exit 0
fi

echo "Found coverage report: $LCOV_FILE"
echo "Size: $(wc -c < "$LCOV_FILE") bytes"
echo ""

# Check if report has actual content
if [ ! -s "$LCOV_FILE" ]; then
    echo "Warning: Coverage report is empty"
    exit 0
fi

# Output directory — use /tmp if /workspace isn't writable
OUT_DIR="/workspace"
if [ ! -w "$OUT_DIR" ]; then
    OUT_DIR="/tmp"
    echo "Note: /workspace not writable, using $OUT_DIR for output"
fi

# Generate HTML report
if command -v genhtml &>/dev/null; then
    mkdir -p "$OUT_DIR/coverage_report"
    genhtml "$LCOV_FILE" \
        --output-directory "$OUT_DIR/coverage_report" \
        --title "Shadesmar Coverage" \
        --legend --branch-coverage 2>&1 || \
    echo "Warning: genhtml failed (non-fatal)"
fi

# Print summary
echo ""
echo "=== Coverage Summary ==="
lcov --summary "$LCOV_FILE" 2>&1 || true

# Copy lcov data for artifact upload
cp "$LCOV_FILE" "$OUT_DIR/coverage.lcov"
echo ""
echo "Coverage report: $OUT_DIR/coverage_report/index.html"
echo "Raw lcov data:   $OUT_DIR/coverage.lcov"
