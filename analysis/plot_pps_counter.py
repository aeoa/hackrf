#!/usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot metadata words embedded at the start of each USB chunk."
    )
    parser.add_argument("iq_file", type=Path, help="Path to raw HackRF capture file")
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=0x4000,
        help="Stride in bytes between successive metadata words (default: 0x4000).",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=10e6,
        help="Sample rate in samples per second (default: 10e6).",
    )
    parser.add_argument(
        "--metadata-bytes",
        type=int,
        default=4,
        help="Metadata bytes at start of each chunk (default: 4 = SCT_COUNT).",
    )
    parser.add_argument(
        "--bytes-per-sample",
        type=int,
        default=2,
        help="Bytes per complex sample in the capture (default: 2 for int8 IQ).",
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="Optional path to save the figure instead of showing it interactively.",
    )
    parser.add_argument(
        "--fields",
        type=int,
        nargs="+",
        help="Optional list of metadata word indices to plot (default: all words).",
    )
    parser.add_argument(
        "--print-delta",
        action="store_true",
        help="Print per-chunk SCT_COUNT deltas for debugging.",
    )
    return parser.parse_args()


def load_metadata(path: Path, chunk_size: int, metadata_bytes: int) -> np.ndarray:
    data = np.memmap(path, dtype=np.uint8, mode="r")
    chunk_count = data.size // chunk_size
    if chunk_count == 0:
        raise ValueError("File does not contain a full chunk.")

    trimmed = data[: chunk_count * chunk_size]
    chunks = trimmed.reshape(chunk_count, chunk_size)
    if metadata_bytes % 4 != 0:
        raise ValueError("metadata_bytes must be a multiple of 4.")
    if metadata_bytes > chunk_size:
        raise ValueError("metadata_bytes cannot exceed chunk size.")

    metadata = chunks[:, :metadata_bytes].copy()
    word_count = metadata_bytes // 4
    return metadata.view("<u4").reshape(chunk_count, word_count)


def build_time_axis(
    count: int,
    chunk_size: int,
    metadata_bytes: int,
    bytes_per_sample: int,
    sample_rate: float,
) -> np.ndarray:
    payload_bytes = chunk_size - metadata_bytes
    if payload_bytes <= 0:
        raise ValueError("Chunk size must exceed metadata bytes.")
    samples_per_chunk = payload_bytes / bytes_per_sample
    seconds_per_chunk = samples_per_chunk / sample_rate
    return np.arange(count, dtype=np.float64) * seconds_per_chunk


def main():
    args = parse_args()
    metadata = load_metadata(args.iq_file, args.chunk_size, args.metadata_bytes)
    word_count = metadata.shape[1]

    if args.fields is None:
        fields = list(range(word_count))
    else:
        fields = []
        for idx in args.fields:
            if not (0 <= idx < word_count):
                raise ValueError(f"field index {idx} out of range (0..{word_count-1}).")
            fields.append(idx)
    if not fields:
        raise ValueError("No metadata fields selected to plot.")

    times = build_time_axis(
        metadata.shape[0],
        args.chunk_size,
        args.metadata_bytes,
        args.bytes_per_sample,
        args.sample_rate,
    )

    rows = int(np.ceil(len(fields) / 3))
    cols = min(len(fields), 3)
    fig, axes = plt.subplots(rows, cols, sharex=True, figsize=(8, 2.5 * rows))
    if isinstance(axes, np.ndarray):
        axes = axes.flatten()
    else:
        axes = [axes]
    axes = axes[:len(fields)]

    labels = {
        0: "SCT_COUNT",
    }

    for ax, field_idx in zip(axes, fields):
        ax.plot(times, metadata[:, field_idx], marker=".", linestyle="-")
        ax.set_ylabel(labels.get(field_idx, f"Word {field_idx}"))
        ax.grid(True)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(f"Metadata words vs time ({args.iq_file.name})", y=0.99)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    if args.print_delta and metadata.shape[0] > 1:
        deltas = np.diff(metadata[:, 0], prepend=metadata[0, 0]) & 0xFFFFFFFF
        unique_deltas = np.unique(deltas)
        print(
            "SCT_COUNT deltas (mod 2^32): min={}, max={}, sample={}".format(
                deltas.min(),
                deltas.max(),
                unique_deltas[:8],
            )
        )

    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches="tight")
    else:
        plt.show()


if __name__ == "__main__":
    main()
