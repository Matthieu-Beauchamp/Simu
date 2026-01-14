#!/usr/bin/env python3

import csv
import math
import sys
from collections import defaultdict
from dataclasses import dataclass, field

# ----------------------------
# Configuration
# ----------------------------
Z_THRESHOLD = 2.0  # statistical threshold
REL_THRESHOLD = 5.0  # percent change threshold


# ----------------------------
# Parsing
# ----------------------------

@dataclass
class BenchmarkEntry:
    name: str
    count: int
    mean: float
    stddev: float
    cv: float
    min: float
    max: float


NULL_ENTRY = BenchmarkEntry("", 0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass
class Benchmark:
    name: str
    num_frames: int
    entries: dict[str, dict[str, BenchmarkEntry]] = field(default_factory=dict)

    def __getitem__(self, item):
        return self.entries[item]


# ----------------------------
# Helpers
# ----------------------------

# For two samples z-test
# Gives the number of standard deviations away the mean is from its expected value
#
# Used for its simplicity of interpretation
def z_score(old_entry: BenchmarkEntry, new_entry: BenchmarkEntry, n1: int, n2: int):
    denom = math.sqrt((old_entry.stddev * old_entry.stddev) / n1 + (new_entry.stddev * new_entry.stddev) / n2)
    if denom == 0:
        return 0.0
    return (new_entry.mean - old_entry.mean) / denom


def z_label(z):
    az = abs(z)
    if az < 1:
        return "noise"
    if az < 2:
        return "weak"
    if az < 3:
        return "likely"
    return "strong"


def cohen_d(old_entry: BenchmarkEntry, new_entry: BenchmarkEntry, n1: int, n2: int):
    weighted_var_sum = (old_entry.stddev * old_entry.stddev) * (n1 - 1) + (new_entry.stddev * new_entry.stddev) * (n2 - 1)
    pooled_stddev = math.sqrt(weighted_var_sum / (n1 + n2 - 2))
    if pooled_stddev == 0:
        return 0.0
    return (new_entry.mean - old_entry.mean) / pooled_stddev


def split_scope(name):
    scope, metric = name.split(" ", 1)
    return scope, metric


def load_csv(path) -> Benchmark:
    rows = {}
    frames = math.inf
    with (open(path, newline="") as f):
        reader = csv.DictReader(f)
        for r in reader:
            entry = BenchmarkEntry(
                r["name"],
                int(r["count"]),
                float(r["mean"]),
                float(r["stddev"]),
                float(r["cv"]),
                float(r["min"]),
                float(r["max"])
            )

            if entry.name == "frames":
                frames = entry.count
            else:
                scope, metric = split_scope(entry.name)
                if scope not in rows:
                    rows[scope] = {}

                rows[scope][metric] = entry

    return Benchmark(path, frames, rows)


@dataclass
class EntryDiff:
    old: BenchmarkEntry
    new: BenchmarkEntry
    name: str
    delta: float
    rel: float
    z: float
    d: float
    meaningful: bool


def make_diff_entry(name: str, old_entry: BenchmarkEntry, new_entry: BenchmarkEntry, num_frames_old: int,
                    num_frames_new: int) -> EntryDiff:
    delta = new_entry.mean - old_entry.mean
    rel = (delta / old_entry.mean * 100) if old_entry.mean != 0 else 0.0
    z = z_score(old_entry, new_entry, num_frames_old, num_frames_new)
    d = cohen_d(old_entry, new_entry, num_frames_old, num_frames_new)
    meaningful = abs(z) >= Z_THRESHOLD and abs(rel) >= REL_THRESHOLD
    return EntryDiff(old_entry, new_entry, name, delta, rel, z, d, meaningful)


# ----------------------------
# Main
# ----------------------------

def main(old_path, new_path):
    old = load_csv(old_path)
    new = load_csv(new_path)

    scopes: dict[str, dict[str, tuple[BenchmarkEntry, BenchmarkEntry]]] = defaultdict(dict)

    diff_entries = []

    all_scopes = set(old.entries.keys()) | set(new.entries.keys())
    for scope in all_scopes:
        all_metrics = set(old.entries.get(scope, {}).keys()) | set(new.entries.get(scope, {}).keys())
        for metric in all_metrics:
            old_entry = old.entries.get(scope, {metric: NULL_ENTRY}).get(metric)
            new_entry = new.entries.get(scope, {metric: NULL_ENTRY}).get(metric)
            scopes[scope][metric] = (old_entry, new_entry)

            diff = make_diff_entry(f"{scope} {metric}", old_entry, new_entry, old.num_frames, new.num_frames)
            diff_entries.append(diff)

    print("# Performance Comparison Report\n")

    print(f"Baseline: {old_path}\n")
    print()
    print(f"New: {new_path}\n")

    print("## Summary\n")

    print("| Metric | Baseline | New | Change | Z-score | Cohen's d |")
    print("|--------|----------|-----|--------|---------|-----------|")
    for entry in sorted(diff_entries, key=lambda e: abs(e.d), reverse=True)[:10]:
        print(
            f"| {entry.old.name} | {entry.old.mean:.4f} | {entry.new.mean:.4f} | {entry.delta:+.4f} ({entry.rel:+.2f})% | {entry.z:.2f} ({z_label(entry.z)}) | {entry.d:.2f} |")

    print("---\n")

    for scope in sorted(scopes):
        print(f"## {scope}\n")

        for metric, (old_entry, new_entry) in scopes[scope].items():
            diff = make_diff_entry(metric, old_entry, new_entry, old.num_frames, new.num_frames)

            m1, s1, n1 = old_entry.mean, old_entry.stddev, old.num_frames
            m2, s2, n2 = new_entry.mean, new_entry.stddev, new.num_frames

            print(f"### {metric}\n")
            print("| Metric | Baseline | New | Change |")
            print("|-------|----------|-----|--------|")
            print(f"| Mean | {m1:.4g} | {m2:.4g} | {diff.delta:.4g} ({diff.rel:+.2f}%) |")
            print(f"| Stddev | {s1:.4g} | {s2:.4g} | |")
            print(f"| CV | {old_entry.cv:.3g} | {new_entry.cv:.3g} | |")
            print(f"| Samples | {n1} | {n2} | |")

            print(f"- Z-score: `{diff.z:.2f}` ({z_label(diff.z)})")
            print(f"- Cohen's d: `{diff.d:.2f}`")

            print()

            print("---\n")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: perf_diff_z.py baseline.csv new.csv")
        sys.exit(1)

    main(sys.argv[1], sys.argv[2])
