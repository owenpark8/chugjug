import wave
import sys
import argparse
from pathlib import Path


def convert(input_path: Path, output_path: Path):
    with wave.open(str(input_path), "rb") as f:
        if f.getsampwidth() != 2:
            print(f"Error: expected 16-bit PCM, got {f.getsampwidth() * 8}-bit", file=sys.stderr)
            sys.exit(1)
        print(f"Sample rate: {f.getframerate()} Hz, channels: {f.getnchannels()}, depth: {f.getsampwidth() * 8}-bit")
        frames = f.readframes(f.getnframes())

    output_path.write_bytes(frames)
    print(f"Wrote {len(frames)} bytes ({len(frames)//2} samples) -> {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Convert WAV file to raw binary for STM32 flash audio")
    parser.add_argument("input", type=Path, help="Input .wav file")
    parser.add_argument("output", type=Path, nargs="?", help="Output file name (default: input with .bin extension)")
    args = parser.parse_args()

    if not args.input.exists():
        print(f"Error: {args.input} not found", file=sys.stderr)
        sys.exit(1)

    output = args.output or args.input.with_suffix(".bin")
    convert(args.input, output)


if __name__ == "__main__":
    main()
