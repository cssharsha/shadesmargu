#!/usr/bin/env bash

# Generate .clangd configuration by discovering external dependencies from inside the container
# This script should be run inside the Docker container

set -e

WORKSPACE_DIR="/workspace"
cd "$WORKSPACE_DIR"

echo "Discovering Bazel external dependencies..."

# Get the Bazel output base
BAZEL_OUTPUT_BASE=$(bazel info output_base 2>/dev/null)
EXTERNAL_DIR="$BAZEL_OUTPUT_BASE/external"

if [ ! -d "$EXTERNAL_DIR" ]; then
    echo "Error: External directory not found at $EXTERNAL_DIR"
    exit 1
fi

echo "Found external directory: $EXTERNAL_DIR"

# Find all include directories in external dependencies (search deeper to find nested includes)
INCLUDE_PATHS=()
while IFS= read -r -d '' include_dir; do
    # Convert to relative path from external dir
    rel_path="${include_dir#$EXTERNAL_DIR/}"
    INCLUDE_PATHS+=("$rel_path")
done < <(find "$EXTERNAL_DIR" -maxdepth 6 -type d -name "include" -print0 2>/dev/null)

echo "Found ${#INCLUDE_PATHS[@]} include directories"

# Also search for subdirectories within include/ that contain headers
# (like include/python3.10/ for Python)
echo "Searching for header subdirectories within include/ directories..."
for include_path in "${INCLUDE_PATHS[@]}"; do
    full_include_path="$EXTERNAL_DIR/$include_path"
    for subdir in "$full_include_path"/*; do
        if [ -d "$subdir" ]; then
            # Check if this subdirectory has .h or .hpp files
            if find "$subdir" -maxdepth 1 -type f \( -name "*.h" -o -name "*.hpp" \) -print -quit | grep -q .; then
                rel_path="${subdir#$EXTERNAL_DIR/}"
                # Only add if not already in INCLUDE_PATHS
                if [[ ! " ${INCLUDE_PATHS[@]} " =~ " ${rel_path} " ]]; then
                    INCLUDE_PATHS+=("$rel_path")
                    echo "  Found headers in subdirectory: $rel_path"
                fi
            fi
        fi
    done
done

echo "Total ${#INCLUDE_PATHS[@]} include paths after adding subdirectories"

# Also find external dependencies that have header files directly in their root
# (not in an include/ subdirectory, like matplotlib_cpp)
echo "Searching for external dependencies with root-level headers..."
for dep_dir in "$EXTERNAL_DIR"/*; do
    if [ -d "$dep_dir" ]; then
        # Check if this directory has .h or .hpp files directly in it
        if find "$dep_dir" -maxdepth 1 -type f \( -name "*.h" -o -name "*.hpp" \) -print -quit | grep -q .; then
            rel_path="${dep_dir#$EXTERNAL_DIR/}"
            # Only add if not already in INCLUDE_PATHS
            if [[ ! " ${INCLUDE_PATHS[@]} " =~ " ${rel_path} " ]]; then
                INCLUDE_PATHS+=("$rel_path")
                echo "  Found root-level headers in: $rel_path"
            fi
        fi
    fi
done

echo "Total ${#INCLUDE_PATHS[@]} include paths after adding root-level headers"

# Add specific system libraries (OpenCV, Eigen, GTSAM)
echo "Searching for system library headers..."
SYSTEM_LIBRARY_PATHS=(
    "/usr/include/opencv4"
    "/usr/include/eigen3"
    "/usr/include/gtsam"
)

for lib_dir in "${SYSTEM_LIBRARY_PATHS[@]}"; do
    if [ -d "$lib_dir" ]; then
        INCLUDE_PATHS+=("SYSTEM:$lib_dir")
        echo "  Found system library headers: $lib_dir"
    fi
done

# Add Rerun SDK headers (pre-built in Docker at /opt/rerun_sdk)
RERUN_INCLUDE="/opt/rerun_sdk/include"
if [ -d "$RERUN_INCLUDE" ]; then
    INCLUDE_PATHS+=("SYSTEM:$RERUN_INCLUDE")
    echo "  Found Rerun SDK headers: $RERUN_INCLUDE"
fi

# Also add system Python package includes (like numpy)
echo "Searching for system Python package headers..."
SYSTEM_PYTHON_PATHS=(
    "/usr/lib/python3/dist-packages"
    "/usr/local/lib/python3.10/dist-packages"
    "/usr/lib/python3.10/site-packages"
)

for python_pkg_dir in "${SYSTEM_PYTHON_PATHS[@]}"; do
    if [ -d "$python_pkg_dir" ]; then
        # Look for packages with include directories (like numpy/core/include or numpy/_core/include)
        for pkg_include in "$python_pkg_dir"/*/core/include "$python_pkg_dir"/*/_core/include "$python_pkg_dir"/*/include; do
            if [ -d "$pkg_include" ]; then
                # Use absolute path for system directories (not relative to EXTERNAL_DIR)
                INCLUDE_PATHS+=("SYSTEM:$pkg_include")
                echo "  Found system package headers: $pkg_include"
            fi
        done
    fi
done

echo "Total ${#INCLUDE_PATHS[@]} include paths after adding system packages"

# Generate the config with container paths first
cat > /tmp/clangd_includes.txt <<EOF
# External include paths (container paths)
BAZEL_EXTERNAL=$EXTERNAL_DIR
EOF

for path in "${INCLUDE_PATHS[@]}"; do
    if [[ "$path" == SYSTEM:* ]]; then
        # System paths are already absolute, remove SYSTEM: prefix
        echo "SYSTEM:${path#SYSTEM:}" >> /tmp/clangd_includes.txt
    else
        # Bazel external paths are relative
        echo "INCLUDE:$path" >> /tmp/clangd_includes.txt
    fi
done

cat /tmp/clangd_includes.txt
