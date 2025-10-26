import argparse
import numpy as np

parser = argparse.ArgumentParser(description="Dump SCT diagnostics from capture")
parser.add_argument("iq_file")
parser.add_argument("--metadata-bytes", type=int, default=20)
parser.add_argument("--chunk-size", type=int, default=0x4000)
args = parser.parse_args()

mm = np.memmap(args.iq_file, dtype=np.uint8, mode='r')
chunks = mm.size // args.chunk_size
meta = mm[:chunks*args.chunk_size].reshape(chunks, args.chunk_size)
meta = meta[:, :args.metadata_bytes].copy().view('<u4').reshape(chunks, -1)

sct_count = meta[:,0]
print("Chunks:", chunks)
print("Unique counts delta (mod 2^32):", np.unique(np.diff(sct_count, prepend=sct_count[0]) & 0xFFFFFFFF)[:16])
print("Unique SCT_INPUT values:", np.unique(meta[:,1])[:16])
print("First few lines:")
for i in range(min(10, chunks)):
    print(i, meta[i])
