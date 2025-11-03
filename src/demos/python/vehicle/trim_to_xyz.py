#!/usr/bin/env python3
import csv
import os
import sys


def trim_csv_to_xyz(path: str) -> None:
    tmp_path = path + ".tmp"
    with open(path, "r", newline="") as fin, open(tmp_path, "w", newline="") as fout:
        reader = csv.reader(fin)
        writer = csv.writer(fout)

        try:
            header = next(reader)
        except StopIteration:
            raise RuntimeError(f"Empty CSV: {path}")

        normalized = [h.strip().lower() for h in header]
        wanted = ["x", "y", "z"]
        indices = []
        for w in wanted:
            if w in normalized:
                indices.append(normalized.index(w))

        if len(indices) != 3:
            if len(header) >= 3:
                indices = [0, 1, 2]
            else:
                raise RuntimeError(f"CSV {path} does not have at least 3 columns")

        writer.writerow(["x", "y", "z"])
        for row in reader:
            if not row:
                continue
            out = []
            for i in indices:
                out.append(row[i].strip() if i < len(row) else "")
            writer.writerow(out)

    os.replace(tmp_path, path)


def main():
    if len(sys.argv) > 1:
        targets = sys.argv[1:]
    else:
        targets = [
            "sph_10_1_0.1_0.005.csv",
            "boundary_10_1_0.1_0.005.csv",
        ]

    for t in targets:
        if not os.path.exists(t):
            print(f"Skipping missing file: {t}")
            continue
        print(f"Trimming to x,y,z: {t}")
        trim_csv_to_xyz(t)
        print(f"Done: {t}")


if __name__ == "__main__":
    main()

