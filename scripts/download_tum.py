import os
import tarfile
import requests
import argparse
import sys

# TUM RGB-D Benchmark datasets for visual SLAM:
# https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download
#
# These are standard benchmarks for monocular/RGB-D SLAM evaluation.
# Each sequence contains: rgb/, depth/, rgb.txt, depth.txt, groundtruth.txt
#
# TUM-VI Visual-Inertial datasets:
# https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset
#
# EuRoC format: mav0/cam0/, mav0/imu0/, mav0/state_groundtruth_estimate0/
# Stereo fisheye cameras (1024x1024 or 512x512) + 6-axis IMU at 200Hz

RGBD_BASE_URL = "https://cvg.cit.tum.de/rgbd/dataset"
TUMVI_BASE_URL = "http://vision.in.tum.de/tumvi/exported/euroc/512_16"

DATASETS = {
    # --- TUM RGB-D ---
    "rgbd_dataset_freiburg1_xyz": {
        "url": f"{RGBD_BASE_URL}/freiburg1/rgbd_dataset_freiburg1_xyz.tgz",
        "description": "fr1/xyz: Simple translation on desk (30s, 0.47GB)",
        "category": "tum-rgbd",
        "output_subdir": "tum_datasets",
    },
    "rgbd_dataset_freiburg1_desk": {
        "url": f"{RGBD_BASE_URL}/freiburg1/rgbd_dataset_freiburg1_desk.tgz",
        "description": "fr1/desk: Handheld SLAM around desk (23s, 0.36GB)",
        "category": "tum-rgbd",
        "output_subdir": "tum_datasets",
    },
    "rgbd_dataset_freiburg1_room": {
        "url": f"{RGBD_BASE_URL}/freiburg1/rgbd_dataset_freiburg1_room.tgz",
        "description": "fr1/room: Full room traversal with loops (49s, 0.83GB)",
        "category": "tum-rgbd",
        "output_subdir": "tum_datasets",
    },
    "rgbd_dataset_freiburg2_desk": {
        "url": f"{RGBD_BASE_URL}/freiburg2/rgbd_dataset_freiburg2_desk.tgz",
        "description": "fr2/desk: Slow handheld SLAM (99s, 2.01GB)",
        "category": "tum-rgbd",
        "output_subdir": "tum_datasets",
    },
    # --- TUM-VI (EuRoC format, 512x512 16-bit) ---
    "dataset-room1_512_16": {
        "url": f"{TUMVI_BASE_URL}/dataset-room1_512_16.tar",
        "description": "TUM-VI room1: Indoor with full GT (147m, ~3.5GB)",
        "category": "tum-vi",
        "output_subdir": "tumvi_datasets",
    },
    "dataset-room2_512_16": {
        "url": f"{TUMVI_BASE_URL}/dataset-room2_512_16.tar",
        "description": "TUM-VI room2: Indoor with full GT (142m, ~3.4GB)",
        "category": "tum-vi",
        "output_subdir": "tumvi_datasets",
    },
    "dataset-room3_512_16": {
        "url": f"{TUMVI_BASE_URL}/dataset-room3_512_16.tar",
        "description": "TUM-VI room3: Indoor with full GT (135m, ~3.2GB)",
        "category": "tum-vi",
        "output_subdir": "tumvi_datasets",
    },
}


def download_file(url, filepath):
    """Downloads a file from a URL to a local path with a progress indicator."""
    try:
        with requests.get(url, stream=True, allow_redirects=True) as r:
            r.raise_for_status()
            total_size = int(r.headers.get("content-length", 0))

            with open(filepath, "wb") as f:
                downloaded = 0
                for chunk in r.iter_content(chunk_size=8192):
                    f.write(chunk)
                    downloaded += len(chunk)
                    if total_size > 0:
                        percent = int(100 * downloaded / total_size)
                        mb_done = downloaded / (1024 * 1024)
                        mb_total = total_size / (1024 * 1024)
                        sys.stdout.write(
                            f"\r  Downloading: {percent}% ({mb_done:.1f}/{mb_total:.1f} MB)"
                        )
                        sys.stdout.flush()
        print("")
        return True
    except requests.exceptions.RequestException as e:
        print(f"\n  Error downloading {url}: {e}")
        return False


def extract_archive(source_path, dest_dir):
    """Extracts a .tgz or .tar file to the destination directory."""
    try:
        print(f"  Extracting to {dest_dir}...")
        mode = "r:gz" if source_path.endswith(".tgz") else "r"
        with tarfile.open(source_path, mode) as tar:
            tar.extractall(path=dest_dir)
        return True
    except Exception as e:
        print(f"  Error extracting {source_path}: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Download TUM RGB-D and TUM-VI datasets for visual SLAM."
    )
    parser.add_argument(
        "output_dir",
        type=str,
        help="Base directory (datasets go into tum_datasets/ and tumvi_datasets/ subdirs).",
    )
    parser.add_argument(
        "--dataset",
        type=str,
        choices=list(DATASETS.keys()),
        help="Download only a specific dataset (default: all).",
    )
    parser.add_argument(
        "--category",
        type=str,
        choices=["tum-rgbd", "tum-vi"],
        help="Download only datasets from a specific category.",
    )
    args = parser.parse_args()

    datasets_to_download = DATASETS
    if args.dataset:
        datasets_to_download = {args.dataset: DATASETS[args.dataset]}
    elif args.category:
        datasets_to_download = {
            k: v for k, v in DATASETS.items() if v["category"] == args.category
        }

    print(f"Base output: {os.path.abspath(args.output_dir)}\n")

    for name, info in datasets_to_download.items():
        subdir = info.get("output_subdir", "tum_datasets")
        base_output_dir = os.path.join(args.output_dir, subdir)
        os.makedirs(base_output_dir, exist_ok=True)

        sequence_dir = os.path.join(base_output_dir, name)

        # Check if already extracted
        # TUM RGB-D: look for rgb.txt; TUM-VI: look for mav0/
        if info["category"] == "tum-vi":
            check_path = os.path.join(sequence_dir, "mav0")
        else:
            check_path = os.path.join(sequence_dir, "rgb.txt")

        if os.path.exists(check_path):
            print(f"Skipping {name} (already extracted) - {info['description']}")
            continue

        # Determine archive extension from URL
        url = info["url"]
        ext = ".tar" if url.endswith(".tar") else ".tgz"
        archive_path = os.path.join(base_output_dir, f"{name}{ext}")

        # Download if not already present
        if not os.path.exists(archive_path):
            print(f"Target: {name} - {info['description']}")
            success = download_file(url, archive_path)
            if not success:
                if os.path.exists(archive_path):
                    os.remove(archive_path)
                continue
        else:
            print(f"Target: {name} (already downloaded) - {info['description']}")

        # Extract
        success = extract_archive(archive_path, base_output_dir)
        if success:
            size_mb = os.path.getsize(archive_path) / (1024 * 1024)
            print(f"  Saved: {sequence_dir} (archive: {size_mb:.1f} MB)")
        else:
            if os.path.exists(sequence_dir):
                import shutil
                shutil.rmtree(sequence_dir)

    print("\nAll operations completed.")


if __name__ == "__main__":
    main()
