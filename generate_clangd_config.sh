#!/usr/bin/env bash

# Generate .clangd configuration for host by converting container paths to host paths
# This script runs on the host after docker/generate_clangd_config.sh runs in the container

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Generating .clangd config from container paths..."

# Run the discovery script inside the container
docker compose -f docker/docker-compose.yml exec -T slam_course bash /workspace/docker/generate_clangd_config.sh > /tmp/clangd_includes_raw.txt

# Parse the output and convert container paths to host paths
BAZEL_EXTERNAL=""
INCLUDE_PATHS=()
SYSTEM_PATHS=()

while IFS= read -r line; do
    if [[ "$line" =~ ^BAZEL_EXTERNAL=(.*)$ ]]; then
        CONTAINER_BAZEL_EXTERNAL="${BASH_REMATCH[1]}"
        # Convert /home/developer to /home/sree (or actual $HOME)
        HOST_BAZEL_EXTERNAL="${CONTAINER_BAZEL_EXTERNAL/\/home\/developer/$HOME}"
        BAZEL_EXTERNAL="$HOST_BAZEL_EXTERNAL"
    elif [[ "$line" =~ ^INCLUDE:(.*)$ ]]; then
        INCLUDE_PATHS+=("${BASH_REMATCH[1]}")
    elif [[ "$line" =~ ^SYSTEM:(.*)$ ]]; then
        SYSTEM_PATHS+=("${BASH_REMATCH[1]}")
    fi
done < /tmp/clangd_includes_raw.txt

if [ -z "$BAZEL_EXTERNAL" ]; then
    echo "Error: Could not determine Bazel external directory"
    exit 1
fi

echo "Bazel external on host: $BAZEL_EXTERNAL"
echo "Found ${#INCLUDE_PATHS[@]} include paths"
echo "Found ${#SYSTEM_PATHS[@]} system paths"

# Fix bazel-out symlink on host
BAZEL_OUTPUT_BASE="$(dirname "$BAZEL_EXTERNAL")"
BAZEL_OUT_HOST="$BAZEL_OUTPUT_BASE/execroot/slam_course/bazel-out"
if [ -d "$BAZEL_OUT_HOST" ]; then
    echo "Updating bazel-out symlink to: $BAZEL_OUT_HOST"
    rm -f bazel-out
    ln -s "$BAZEL_OUT_HOST" bazel-out
else
    echo "Warning: Could not find host bazel-out at $BAZEL_OUT_HOST"
fi

# Generate .clangd config
cat > .clangd <<EOF
CompileFlags:
  Add:
EOF

# Add all discovered include paths
for include_path in "${INCLUDE_PATHS[@]}"; do
    full_path="$BAZEL_EXTERNAL/$include_path"
    if [ -d "$full_path" ]; then
        echo "    - -I$full_path" >> .clangd
        echo "  Adding: $full_path"
    fi
done

# Add system paths if they exist on the host
for system_path in "${SYSTEM_PATHS[@]}"; do
    if [ -d "$system_path" ]; then
        echo "    - -I$system_path" >> .clangd
        echo "  Adding system: $system_path"
    else
        found=false

        # Check for system libraries (OpenCV, Eigen, GTSAM, Rerun) and copy from container if missing
        if [[ "$system_path" == *"/opencv4" ]] || [[ "$system_path" == *"/eigen3" ]] || [[ "$system_path" == *"/gtsam" ]] || [[ "$system_path" == *"/rerun_sdk/include" ]]; then
             CACHE_DIR=".cache/system_includes"

             # Rerun uses a deeper path — use "rerun_sdk" as cache name
             if [[ "$system_path" == *"/rerun_sdk/include" ]]; then
                 BASE_NAME="rerun_sdk"
                 LOCAL_CACHE="$CACHE_DIR/$BASE_NAME"
             else
                 BASE_NAME=$(basename "$system_path")
                 LOCAL_CACHE="$CACHE_DIR/$BASE_NAME"
             fi
             
             if [ ! -d "$LOCAL_CACHE" ]; then
                 echo "System path $system_path not found on host. Attempting to copy from container..."
                 mkdir -p "$CACHE_DIR"
                 if docker compose -f docker/docker-compose.yml ps -q slam_course > /dev/null; then
                     CONTAINER_ID=$(docker compose -f docker/docker-compose.yml ps -q slam_course)
                     if [[ "$BASE_NAME" == "rerun_sdk" ]]; then
                         # Copy the include dir contents into cache/rerun_sdk/
                         docker cp "${CONTAINER_ID}:${system_path}" "$LOCAL_CACHE"
                     else
                         docker cp "${CONTAINER_ID}:${system_path}" "$LOCAL_CACHE"
                     fi
                     if [ -d "$LOCAL_CACHE" ]; then
                         echo "Successfully copied $BASE_NAME to cache."
                     else
                         echo "Warning: Failed to copy $system_path"
                     fi
                 else
                     echo "Warning: Container not running, cannot copy $system_path"
                 fi
             fi
             
             if [ -d "$LOCAL_CACHE" ]; then
                 # Special handling for GTSAM to allow <gtsam/...> includes
                 if [[ "$BASE_NAME" == "gtsam" ]]; then
                     echo "    - -I$SCRIPT_DIR/$CACHE_DIR" >> .clangd
                     echo "  Adding cached system path (parent): $SCRIPT_DIR/$CACHE_DIR"
                 else
                     echo "    - -I$SCRIPT_DIR/$LOCAL_CACHE" >> .clangd
                     echo "  Adding cached system path: $SCRIPT_DIR/$LOCAL_CACHE"
                 fi
                 found=true
             fi
        fi

        # Try to find equivalent path with different Python version
        if [ "$found" = false ] && [[ "$system_path" =~ numpy/(.*core)/include ]]; then
            # Try to find numpy with any Python version and either core or _core
            for alt_path in /usr/lib/python3*/site-packages/numpy/core/include \
                            /usr/lib/python3*/site-packages/numpy/_core/include \
                            /usr/lib/python3/dist-packages/numpy/core/include \
                            /usr/lib/python3/dist-packages/numpy/_core/include \
                            /usr/local/lib/python3*/site-packages/numpy/core/include \
                            /usr/local/lib/python3*/site-packages/numpy/_core/include; do
                if [ -d "$alt_path" ]; then
                    echo "    - -I$alt_path" >> .clangd
                    echo "  Adding system (found alternative): $alt_path"
                    found=true
                    break
                fi
            done
        fi

        if [ "$found" = false ]; then
            echo "  Warning: System path not found on host (skipping): $system_path"
        fi
    fi
done

# Check and copy CUDA headers if missing
if [ ! -d ".cache/cuda/include" ]; then
    echo "Local CUDA cache not found. Attempting to copy from container..."
    mkdir -p .cache/cuda
    
    # Check if container is running
    if docker compose -f docker/docker-compose.yml ps -q slam_course > /dev/null; then
        # Try to find CUDA include path in container
        CUDA_INCLUDE_PATH=$(docker compose -f docker/docker-compose.yml exec -T slam_course readlink -f /usr/local/cuda/include 2>/dev/null || echo "")
        
        if [ -n "$CUDA_INCLUDE_PATH" ]; then
            echo "Found CUDA in container at $CUDA_INCLUDE_PATH"
            # Get the container ID
            CONTAINER_ID=$(docker compose -f docker/docker-compose.yml ps -q slam_course)
            
            # Copy headers
            docker cp "${CONTAINER_ID}:${CUDA_INCLUDE_PATH}" .cache/cuda/
            # Rename the copied directory (it copies as 'include')
            if [ -d ".cache/cuda/include" ]; then
                echo "Successfully copied CUDA headers."
            else
                # In case it copied with a different name or structure
                echo "Warning: CUDA copy might have failed or has unexpected structure."
                ls -F .cache/cuda/
            fi
        else
            echo "Warning: Could not locate CUDA include path in container."
        fi
    else
        echo "Warning: cppgrad container is not running. Cannot copy CUDA headers."
    fi
fi

# Add local CUDA cache if it exists
if [ -d ".cache/cuda/include" ]; then

    echo "    - -I$SCRIPT_DIR/.cache/cuda/include
    - -I$SCRIPT_DIR
    - --cuda-path=$SCRIPT_DIR/.cache/cuda
    - -nocudalib
    - -nocudainc
    - -x
    - c++" >> .clangd
    echo "  Adding local CUDA cache: $SCRIPT_DIR/.cache/cuda/include"
fi

# Complete the config
cat >> .clangd <<EOF
  Remove:
    # Remove problematic symlink paths that point to container locations
    - -iquote*external/*
    - -isystem*external/*

Diagnostics:
  UnusedIncludes: None
  MissingIncludes: Strict
EOF

echo ""
echo ".clangd config generated successfully!"
