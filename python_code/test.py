import tensorflow_datasets as tfds
import os

DATASET_NAME = "piper_dataset_builder"
DATASET_DIR = os.path.expanduser("~/tensorflow_datasets")

ds = tfds.load(DATASET_NAME, split="train", data_dir=DATASET_DIR)

def print_structure(d, prefix=""):
    if isinstance(d, dict):
        for k, v in d.items():
            print_structure(v, prefix + "/" + k)
    else:
        print(prefix, d.shape, d.dtype)

for ep in ds.take(1):
    print("\n===== EPISODE STRUCTURE =====")
    print_structure(ep)