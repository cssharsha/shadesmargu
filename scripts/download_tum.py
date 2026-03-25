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
# Camera intrinsics per sensor:
#   Freiburg 1: fx=517.3 fy=516.5 cx=318.6 cy=255.3
#   Freiburg 2: fx=520.9 fy=521.0 cx=325.1 cy=249.7
#   Freiburg 3: fx=535.4 fy=539.2 cx=320.1 cy=247.6 (undistorted)

BASE_URL = "https://cvg.cit.tum.de/rgbd/dataset"

DATASETS = {
    "rgbd_dataset_freiburg1_xyz": {
        "url": f"{BASE_URL}/freiburg1/rgbd_dataset_freiburg1_xyz.tgz",
        "description": "fr1/xyz: Simple translation on desk (30s, 0.47GB)",
        "category": "testing",
    },
    "rgbd_dataset_freiburg1_desk": {
        "url": f"{BASE_URL}/freiburg1/rgbd_dataset_freiburg1_desk.tgz",
        "description": "fr1/desk: Handheld SLAM around desk (23s, 0.36GB)",
        "category": "handheld",
    },
    "rgbd_dataset_freiburg1_room": {
        "url": f"{BASE_URL}/freiburg1/rgbd_dataset_freiburg1_room.tgz",
        "description": "fr1/room: Full room traversal with loops (49s, 0.83GB)",
        "category": "handheld",
    },
    "rgbd_dataset_freiburg2_desk": {
        "url": f"{BASE_URL}/freiburg2/rgbd_dataset_freiburg2_desk.tgz",
        "description": "fr2/desk: Slow handheld SLAM (99s, 2.01GB)",
        "category": "handheld",
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


def extract_tgz(source_path, dest_dir):
    """Extracts a .tgz file to the destination directory."""
    try:
        print(f"  Extracting to {dest_dir}...")
        with tarfile.open(source_path, "r:gz") as tar:
            tar.extractall(path=dest_dir)
        return True
    except Exception as e:
        print(f"  Error extracting {source_path}: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Download TUM RGB-D benchmark datasets for visual SLAM."
    )
    parser.add_argument(
        "output_dir",
        type=str,
        help="The directory where the 'tum_datasets' folder will be created.",
    )
    parser.add_argument(
        "--dataset",
        type=str,
        choices=list(DATASETS.keys()),
        help="Download only a specific dataset (default: all).",
    )
    args = parser.parse_args()

    base_output_dir = os.path.join(args.output_dir, "tum_datasets")

    if not os.path.exists(base_output_dir):
        os.makedirs(base_output_dir)

    print(f"Starting download of TUM datasets to: {os.path.abspath(base_output_dir)}\n")

    datasets_to_download = DATASETS
    if args.dataset:
        datasets_to_download = {args.dataset: DATASETS[args.dataset]}

    for name, info in datasets_to_download.items():
        sequence_dir = os.path.join(base_output_dir, name)

        # Check if already extracted (look for rgb.txt as indicator)
        if os.path.exists(os.path.join(sequence_dir, "rgb.txt")):
            print(f"Skipping {name} (already extracted) - {info['description']}")
            continue

        tgz_path = os.path.join(base_output_dir, f"{name}.tgz")

        # Download if not already present
        if not os.path.exists(tgz_path):
            print(f"Target: {name} - {info['description']}")
            success = download_file(info["url"], tgz_path)
            if not success:
                if os.path.exists(tgz_path):
                    os.remove(tgz_path)
                continue
        else:
            print(f"Target: {name} (already downloaded) - {info['description']}")

        # Extract
        success = extract_tgz(tgz_path, base_output_dir)
        if success:
            size_mb = os.path.getsize(tgz_path) / (1024 * 1024)
            print(f"  Saved: {sequence_dir} (archive: {size_mb:.1f} MB)")
            # Optionally remove the .tgz to save space
            # os.remove(tgz_path)
        else:
            # Remove partial extraction
            if os.path.exists(sequence_dir):
                import shutil

                shutil.rmtree(sequence_dir)

    print("\nAll operations completed.")


if __name__ == "__main__":
    main()
