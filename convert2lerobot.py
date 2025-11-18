#!/usr/bin/env python3

"""
Convert the Oracle raw dataset located under /home/data/Dataset/oracle_dataset_raw
into a single LeRobot v3 dataset stored under /home/data/Dataset/rheovla_dataset_lerobot.
Each episode is stored in its own parquet and per-camera MP4 files (one episode per file).

The raw dataset is expected to be organized as:

oracle_dataset_raw/
  <task_name>/
    episode_00000/
      img/
        <camera_name>/
          00001.jpg
          ...
      state/
        00001.log
        ...

Each `.log` file contains a single whitespace separated row of floats describing the
robot state/action for that frame. Two RGB camera streams named `fisheye_rgb` and
`realsense_rgb` store synchronized frames for every episode. All sequences across
modalities must have the same number of frames.
"""

from __future__ import annotations

import argparse
import logging
import shutil
from pathlib import Path
from typing import Sequence

import numpy as np
from PIL import Image

from lerobot.datasets.lerobot_dataset import LeRobotDataset

RAW_ROOT = Path("/home/data/Dataset/oracle_dataset_raw")
OUTPUT_DATASET_PATH = Path("/home/data/Dataset/rheovla_dataset_lerobot")
DEFAULT_REPO_ID = "rheovla_dataset_lerobot"
DEFAULT_FPS = 10
DEFAULT_ACTION_CHUNK = 10


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert Oracle dataset to LeRobot v3 format.")
    parser.add_argument("--raw-root", type=Path, default=RAW_ROOT, help="Path to oracle_dataset_raw.")
    parser.add_argument("--output-path", type=Path, default=OUTPUT_DATASET_PATH, help="Where to store LeRobot dataset.")
    parser.add_argument("--repo-id", type=str, default=DEFAULT_REPO_ID, help="Repo id recorded in metadata.")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Recording FPS used in metadata.")
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Delete the existing output folder before converting. Use with care.",
    )
    parser.add_argument(
        "--action-chunk-size",
        type=int,
        default=DEFAULT_ACTION_CHUNK,
        help="Number of future states to pack into each action vector (>=1).",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity.",
    )
    return parser.parse_args()


def iter_task_episodes(task_dir: Path) -> list[Path]:
    return [episode for episode in sorted(task_dir.glob("episode_*")) if episode.is_dir()]


def load_state_sequence(state_dir: Path) -> np.ndarray:
    files = sorted(state_dir.glob("*.log"))
    if not files:
        raise ValueError(f"No state frames found in {state_dir}")
    frames = []
    for state_file in files:
        content = state_file.read_text().strip()
        if not content:
            continue
        frames.append(np.fromstring(content, sep=" ", dtype=np.float32))
    if not frames:
        raise ValueError(f"No valid state frames found in {state_dir}")
    return np.stack(frames, axis=0)


def compute_action_chunks(states: np.ndarray, chunk_size: int) -> np.ndarray:
    if chunk_size <= 0:
        raise ValueError("action chunk size must be >= 1")

    num_frames, state_dim = states.shape
    offsets = np.arange(1, chunk_size + 1, dtype=np.int32)
    indices = np.clip(np.arange(num_frames)[:, None] + offsets, 0, num_frames - 1)
    return states[indices]


def discover_cameras(episode_dir: Path) -> dict[str, tuple[int, int, int]]:
    img_root = episode_dir / "img"
    if not img_root.is_dir():
        raise FileNotFoundError(f"Missing img directory in {episode_dir}")
    camera_shapes: dict[str, tuple[int, int, int]] = {}
    for cam_dir in sorted(img_root.iterdir()):
        if not cam_dir.is_dir():
            continue
        sample = next((p for p in cam_dir.glob("*.jpg")), None)
        if sample is None:
            continue
        with Image.open(sample) as img:
            img = img.convert("RGB")
            width, height = img.size
        camera_shapes[cam_dir.name] = (height, width, 3)
    if not camera_shapes:
        raise ValueError(f"No camera folders with JPEG images found under {img_root}")
    return camera_shapes


def infer_state_dim(episode_dir: Path) -> int:
    state_files = sorted((episode_dir / "state").glob("*.log"))
    if not state_files:
        raise FileNotFoundError(f"No .log files inside {episode_dir/'state'}")
    sample_state = state_files[0]
    with sample_state.open("r", encoding="utf-8") as fp:
        parts = fp.read().strip().split()
    if not parts:
        raise ValueError(f"Empty state file {sample_state}")
    return len(parts)


def build_features(
    state_dim: int,
    camera_shapes: dict[str, tuple[int, int, int]],
    use_videos: bool,
    action_chunk_size: int,
) -> dict:
    state_names = [f"dim_{idx}" for idx in range(state_dim)]
    if action_chunk_size <= 0:
        raise ValueError("action_chunk_size must be >= 1")

    features: dict[str, dict] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (state_dim,),
            "names": state_names,
        },
        "action": {
            "dtype": "float32",
            "shape": (action_chunk_size, state_dim),
            "names": ["chunk", "dim"],
        },
    }
    dtype = "video" if use_videos else "image"
    for camera_name, (height, width, channels) in camera_shapes.items():
        features[f"observation.images.{camera_name}"] = {
            "dtype": dtype,
            "shape": (height, width, channels),
            "names": ["height", "width", "channels"],
        }
    return features


def load_camera_frame(image_path: Path) -> np.ndarray:
    with Image.open(image_path) as img:
        img = img.convert("RGB")
        return np.asarray(img)


def make_task_text(task_dir: Path) -> str:
    return task_dir.name.replace("_", " ").strip()


def gather_episode_paths(episode_dir: Path, camera_names: Sequence[str]) -> dict[str, list[Path]]:
    image_paths: dict[str, list[Path]] = {}
    for camera_name in camera_names:
        paths = sorted((episode_dir / "img" / camera_name).glob("*.jpg"))
        if not paths:
            raise ValueError(f"No frames for camera {camera_name} in {episode_dir}")
        image_paths[camera_name] = paths
    return image_paths


def convert(args: argparse.Namespace) -> None:
    logging.basicConfig(level=getattr(logging, args.log_level), format="%(levelname)s - %(message)s")

    if not args.raw_root.is_dir():
        raise FileNotFoundError(f"Raw dataset not found at {args.raw_root}")

    if args.output_path.exists():
        if args.overwrite:
            shutil.rmtree(args.output_path)
        else:
            raise FileExistsError(
                f"{args.output_path} already exists. Pass --overwrite if you really want to delete it first."
            )

    task_dirs = [d for d in sorted(args.raw_root.iterdir()) if d.is_dir()]
    if not task_dirs:
        raise FileNotFoundError(f"No task folders inside {args.raw_root}")

    episodes_per_task: list[tuple[Path, list[Path]]] = []
    first_episode = None
    for task_dir in task_dirs:
        episodes = iter_task_episodes(task_dir)
        if episodes:
            episodes_per_task.append((task_dir, episodes))
            if first_episode is None:
                first_episode = episodes[0]
    if first_episode is None:
        raise FileNotFoundError("No episodes found in any task folder.")

    use_videos = True
    camera_shapes = discover_cameras(first_episode)
    camera_names = tuple(camera_shapes)
    state_dim = infer_state_dim(first_episode)
    features = build_features(
        state_dim,
        camera_shapes,
        use_videos=use_videos,
        action_chunk_size=args.action_chunk_size,
    )

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=features,
        root=args.output_path,
        use_videos=use_videos,
    )
    dataset.batch_encoding_size = 1

    logging.info("Converting dataset with cameras: %s", ", ".join(camera_names))
    total_eps = 0
    for task_dir, episodes in episodes_per_task:
        task_text = make_task_text(task_dir)
        for episode_dir in episodes:
            states = load_state_sequence(episode_dir / "state")
            actions = compute_action_chunks(states, args.action_chunk_size)
            image_paths = gather_episode_paths(episode_dir, camera_names)
            num_frames = states.shape[0]
            for cam in camera_names:
                if len(image_paths[cam]) != num_frames:
                    raise ValueError(
                        f"Camera {cam} has {len(image_paths[cam])} frames but state sequence has {num_frames} frames."
                    )

            logging.info("Processing %s (task: %s, %d frames)", episode_dir.name, task_text, num_frames)

            for frame_idx in range(num_frames):
                frame = {
                    "task": task_text,
                    "observation.state": states[frame_idx],
                    "action": actions[frame_idx],
                }
                for camera_name in camera_names:
                    frame[f"observation.images.{camera_name}"] = load_camera_frame(
                        image_paths[camera_name][frame_idx]
                    )
                dataset.add_frame(frame)

            dataset.save_episode()
            total_eps += 1

    dataset.finalize()
    logging.info("Finished conversion. Episodes: %d, output path: %s", total_eps, args.output_path)


def main() -> None:
    args = parse_args()
    convert(args)


if __name__ == "__main__":
    main()
