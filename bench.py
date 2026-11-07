"""Headless comparison: ours vs central-hub baseline (rules agent, repeatable per seed).

    python bench.py --seeds 5
"""
import argparse
import time

from sim import benchmark


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--start", type=int, default=1)
    a = p.parse_args()
    t0 = time.time()
    rows = benchmark(a.seeds, a.start)
    hdr = f"{'scenario':<22}{'mode':<10}{'found':>7}{'t_find':>8}{'t_all':>8}{'cover':>8}{'dup':>7}"
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['scenario']:<22}{r['mode']:<10}{r['found']:>4}/{r['runs']:<2}"
              f"{str(r['avg_time_to_find']):>8}{str(r['avg_all_know']):>8}"
              f"{r['avg_coverage']:>8.0%}{r['avg_duplicated']:>7.0%}")
    print(f"\n{time.time() - t0:.1f}s")


if __name__ == "__main__":
    main()
