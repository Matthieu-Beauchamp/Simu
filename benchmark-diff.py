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
def z_score(m1, s1, n1, m2, s2, n2):
    denom = math.sqrt((s1 * s1) / n1 + (s2 * s2) / n2)
    if denom == 0:
        return 0.0
    return (m2 - m1) / denom


def z_label(z):
    az = abs(z)
    if az < 1:
        return "noise"
    if az < 2:
        return "weak"
    if az < 3:
        return "likely"
    return "strong"


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


# ----------------------------
# Main
# ----------------------------
def main(old_path, new_path):
    old = load_csv(old_path)
    new = load_csv(new_path)

    scopes = defaultdict(dict)

    all_scopes = set(old.entries.keys()) | set(new.entries.keys())
    for scope in all_scopes:
        all_metrics = set(old.entries.get(scope, {}).keys()) | set(new.entries.get(scope, {}).keys())
        for metric in all_metrics:
            # TODO: Wrong
            old_entry = old.entries.get(scope, {}).get(metric, 0)
            new_entry = new.entries.get(scope, {}).get(metric, 0)
            scopes[scope][metric] = (old_entry, new_entry)

    print("# Performance Comparison Report (Z-score based)\n")

    for scope in sorted(scopes):
        print(f"## {scope}\n")

        for metric, (o, n) in scopes[scope].items():
            m1, s1, n1 = o["mean"], o["stddev"], old.num_frames
            m2, s2, n2 = n["mean"], n["stddev"], new.num_frames

            delta = m2 - m1
            rel = (delta / m1 * 100) if m1 != 0 else 0.0
            z = z_score(m1, s1, n1, m2, s2, n2)

            meaningful = abs(z) >= Z_THRESHOLD and abs(rel) >= REL_THRESHOLD

            print(f"### {metric}\n")
            print("| Metric | Baseline | New | Change |")
            print("|-------|----------|-----|--------|")
            print(f"| Mean | {m1:.4g} | {m2:.4g} | {rel:+.2f}% |")
            print(f"| Stddev | {s1:.4g} | {s2:.4g} | |")
            print(f"| CV | {o['cv']:.3g} | {n['cv']:.3g} | |")
            print(f"| Samples | {n1} | {n2} | |")

            print("\n**Significance**")
            print(f"- Z-score: `{z:.2f}` ({z_label(z)})")
            print(f"- Relative change: `{rel:+.2f}%`")
            print(f"- Verdict: **{'MEANINGFUL CHANGE' if meaningful else 'not significant'}**")

            if metric == "time":
                if meaningful:
                    if rel > 0:
                        print("- ⚠️ Slower execution")
                    else:
                        print("- ✅ Faster execution")
                else:
                    print("- ℹ️ Change within noise bounds")

            print()

            print("---\n")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: perf_diff_z.py baseline.csv new.csv")
        sys.exit(1)

    main(sys.argv[1], sys.argv[2])
