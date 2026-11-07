"""Headless benchmarks (rules agents, repeatable per seed). Saves bench_results.json,
which the web page shows instantly.

    python bench.py --seeds 10      # run and save
    python bench.py --show          # print the saved results
"""
import argparse
import json
import time
from pathlib import Path

from sim import benchmark, coordination_breakdown, resilience_curve

RESULTS = Path(__file__).with_name("bench_results.json")


def run_all(seeds=5, start_seed=1):
    t0 = time.time()
    out = {
        "seeds": seeds,
        "breakdown": coordination_breakdown(seeds, start_seed),
        "curve": resilience_curve(seeds, start_seed),
        "scenarios": benchmark(seeds, start_seed),
    }
    out["seconds"] = round(time.time() - t0, 1)
    return out


def _fmt(v, pct=False):
    if v is None:
        return "-"
    return f"{v:.0%}" if pct else str(v)


def print_results(r):
    print(f"\nWHAT TEAMWORK IS WORTH (radio perfect, {r['seeds']} maps)")
    print(f"{'setup':<38}{'found':>7}{'t_find':>8}{'t_all':>8}{'twice':>7}{'flown':>8}")
    for x in r["breakdown"]:
        print(f"{x['setup']:<38}{x['found']:>4}/{x['runs']:<2}{_fmt(x['avg_time_to_find']):>8}"
              f"{_fmt(x['avg_all_know']):>8}{_fmt(x['avg_duplicated'], True):>7}{x['avg_flight_ticks']:>8}")

    print("\nRESILIENCE: hiker found (%) vs packet loss")
    print(f"{'mode':<10}" + "".join(f"{int(pt['loss'] * 100):>7}%" for pt in r["curve"]["ours"]))
    for mode, pts in r["curve"].items():
        print(f"{mode:<10}" + "".join(f"{pt['found'] / pt['runs']:>8.0%}" for pt in pts))

    print("\nFAILURE SCENARIOS (with battery use)")
    hdr = (f"{'scenario':<26}{'mode':<10}{'found':>7}{'t_find':>8}{'t_all':>8}{'cover':>7}{'dup':>6}"
           f"{'flown':>7}{'swaps':>7}{'low_avg':>8}{'low_min':>8}{'flat':>5}")
    print(hdr)
    print("-" * len(hdr))
    for x in r["scenarios"]:
        print(f"{x['scenario']:<26}{x['mode']:<10}{x['found']:>4}/{x['runs']:<2}"
              f"{_fmt(x['avg_time_to_find']):>8}{_fmt(x['avg_all_know']):>8}"
              f"{x['avg_coverage']:>7.0%}{x['avg_duplicated']:>6.0%}{x['avg_flight_ticks']:>7}{x['avg_swaps']:>7}"
              f"{x['avg_lowest_battery']:>8.0%}{x['worst_lowest_battery']:>8.0%}{x['ran_flat']:>5}")
    print("\nflown = total drone flight time (ticks, all drones); swaps = battery swaps per run;")
    print("low_avg / low_min = lowest charge any drone reached (average run / worst run); flat = drones that ran out")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--start", type=int, default=1)
    p.add_argument("--show", action="store_true", help="print the saved results without re-running")
    a = p.parse_args()
    if a.show:
        print_results(json.loads(RESULTS.read_text(encoding="utf-8")))
        return
    r = run_all(a.seeds, a.start)
    RESULTS.write_text(json.dumps(r, indent=1), encoding="utf-8")
    print_results(r)
    print(f"\n{r['seconds']}s, saved bench_results.json")


if __name__ == "__main__":
    main()
