import os
import requests
import argparse
import sys

# 3D Pose Graph datasets from Luca Carlone's page:
# https://lucacarlone.mit.edu/datasets/
#
# These are standard benchmarks for 3D pose-graph SLAM optimization.
# File format: g2o (VERTEX_SE3:QUAT + EDGE_SE3:QUAT)

DATASETS = {
    "sphere_bignoise_vertex3.g2o": {
        "url": "https://www.dropbox.com/s/ej5hb1ckcp3x42u/sphere_bignoise_vertex3.g2o?dl=1",
        "description": "Sphere-a: ~2500 poses, large noise",
    },
    "torus3D.g2o": {
        "url": "https://www.dropbox.com/s/o95o2lbvww1100r/torus3D.g2o?dl=1",
        "description": "Torus: ~800 poses, 3D torus trajectory",
    },
    "grid3D.g2o": {
        "url": "https://www.dropbox.com/s/gjw9xl3t632gbrk/grid3D.g2o?dl=1",
        "description": "Cube/Grid: ~800 poses, 3D grid",
    },
    "parking-garage.g2o": {
        "url": "https://www.dropbox.com/s/zu23p8d522qccor/parking-garage.g2o?dl=1",
        "description": "Parking Garage: ~1661 poses, multi-floor spiral",
    },
    "cubicle.g2o": {
        "url": "https://www.dropbox.com/s/twpqdfphdw4md94/cubicle.g2o?dl=1",
        "description": "Cubicle: ~5750 poses, indoor environment",
    },
    "rim.g2o": {
        "url": "https://www.dropbox.com/s/25qijwvfpmzh257/rim.g2o?dl=1",
        "description": "Rim: ~10195 poses, large-scale",
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
                        sys.stdout.write(f"\r  Downloading: {percent}%")
                        sys.stdout.flush()
        print("")
        return True
    except requests.exceptions.RequestException as e:
        print(f"\n  Error downloading {url}: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Download 3D pose-graph g2o datasets (Luca Carlone)."
    )
    parser.add_argument(
        "output_dir",
        type=str,
        help="The directory where the 'g2o_datasets' folder will be created.",
    )
    args = parser.parse_args()

    base_output_dir = os.path.join(args.output_dir, "g2o_datasets")

    if not os.path.exists(base_output_dir):
        os.makedirs(base_output_dir)

    print(f"Starting download of g2o datasets to: {os.path.abspath(base_output_dir)}\n")

    for filename, info in DATASETS.items():
        filepath = os.path.join(base_output_dir, filename)

        if os.path.exists(filepath):
            print(f"Skipping {filename} (already exists) - {info['description']}")
            continue

        print(f"Target: {filename} - {info['description']}")
        success = download_file(info["url"], filepath)
        if success:
            size_kb = os.path.getsize(filepath) // 1024
            print(f"  Saved: {filepath} ({size_kb} KB)")
        else:
            # Remove partial download
            if os.path.exists(filepath):
                os.remove(filepath)

    print("\nAll operations completed.")


if __name__ == "__main__":
    main()
