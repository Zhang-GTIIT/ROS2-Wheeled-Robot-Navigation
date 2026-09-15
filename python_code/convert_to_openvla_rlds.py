import os
import tensorflow as tf
import tensorflow_datasets as tfds
import rlds

import sys
sys.path.append("/home/agv/wzb/python_code")
import piper_dataset_builder.piper_dataset_builder


DATASET_NAME = "piper_dataset_builder"
DATASET_DIR = os.path.expanduser("~/tensorflow_datasets")
OUTPUT_DIR = os.path.expanduser("~/tensorflow_datasets/piper_openvla_rlds")


def step_map_fn(step):
    return {
        rlds.OBSERVATION: {
            "image": step["observation"]["image"],
            "state": step["observation"]["state"],
            "language_instruction": step["language_instruction"],
        },
        rlds.ACTION: step["action"],
        rlds.REWARD: tf.constant(0.0, tf.float32),
        rlds.DISCOUNT: tf.constant(1.0, tf.float32),
        rlds.IS_FIRST: step["is_first"],
        rlds.IS_LAST: step["is_last"],
        rlds.IS_TERMINAL: step["is_terminal"],
    }


def episode_map_fn(ep):
    steps_ds = ep[rlds.STEPS].map(step_map_fn, num_parallel_calls=tf.data.AUTOTUNE)
    return {rlds.STEPS: steps_ds}


print("📦 Loading TFDS RLDS dataset...")
ds = tfds.load(DATASET_NAME, split="train", data_dir=DATASET_DIR)

print("🔄 Converting episodes to OpenVLA RLDS format...")
ds = ds.map(episode_map_fn, num_parallel_calls=tf.data.AUTOTUNE)

print("💾 Saving dataset in RLDS-compatible format...")
tf.data.experimental.save(ds, OUTPUT_DIR, compression="GZIP")

print("✅ Done! OpenVLA-ready RLDS dataset at:", OUTPUT_DIR)