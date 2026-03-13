import os
import requests
import bz2
import argparse
import sys
from urllib.parse import urljoin
from bs4 import BeautifulSoup

# Base URL for the BAL project
BASE_URL = "https://grail.cs.washington.edu/projects/bal/"

DATASET_PAGES = [
    "ladybug.html",
    "trafalgar.html",
    "dubrovnik.html",
    "venice.html",
    "final.html",
]


def download_file(url, filepath):
    """Downloads a file from a URL to a local path with a progress indicator."""
    try:
        with requests.get(url, stream=True) as r:
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


def extract_bz2(source_path, dest_path):
    """Decompresses a .bz2 file to the destination path."""
    try:
        print(f"  Extracting to {os.path.basename(dest_path)}...")
        with bz2.open(source_path, "rb") as source, open(dest_path, "wb") as dest:
            for data in iter(lambda: source.read(1024 * 1024), b""):
                dest.write(data)
        return True
    except Exception as e:
        print(f"  Error extracting {source_path}: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Download and extract BAL datasets.")
    parser.add_argument(
        "output_dir",
        type=str,
        help="The directory where the 'bal_datasets' folder will be created.",
    )
    args = parser.parse_args()

    # Define the base output path based on user input
    # We append 'bal_datasets' to keep things clean, so it doesn't scatter files
    # directly into a generic folder like ~/Downloads
    base_output_dir = os.path.join(args.output_dir, "bal_datasets")

    if not os.path.exists(base_output_dir):
        os.makedirs(base_output_dir)

    print(f"Starting download of BAL datasets to: {os.path.abspath(base_output_dir)}\n")

    for page in DATASET_PAGES:
        page_url = urljoin(BASE_URL, page)
        category_name = page.replace(".html", "").capitalize()

        category_dir = os.path.join(base_output_dir, category_name)
        if not os.path.exists(category_dir):
            os.makedirs(category_dir)

        print(f"--- Processing Category: {category_name} ---")

        try:
            response = requests.get(page_url)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            print(f"Failed to access page {page_url}: {e}")
            continue

        soup = BeautifulSoup(response.content, "html.parser")
        links = [
            a["href"]
            for a in soup.find_all("a", href=True)
            if a["href"].endswith(".bz2")
        ]

        if not links:
            print("No datasets found on this page.")
            continue

        print(f"Found {len(links)} datasets.")

        for link in links:
            file_url = urljoin(BASE_URL, link)
            filename = os.path.basename(link)

            file_path_bz2 = os.path.join(category_dir, filename)
            file_path_txt = os.path.join(category_dir, filename.replace(".bz2", ""))

            if os.path.exists(file_path_txt):
                print(f"Skipping {filename} (already extracted).")
                continue

            if not os.path.exists(file_path_bz2):
                print(f"Target: {filename}")
                success = download_file(file_url, file_path_bz2)
                if not success:
                    continue
            else:
                print(f"Target: {filename} (already downloaded)")

            extract_bz2(file_path_bz2, file_path_txt)
            # os.remove(file_path_bz2) # Uncomment to delete bz2 files after extraction

    print("\nAll operations completed.")


if __name__ == "__main__":
    main()
