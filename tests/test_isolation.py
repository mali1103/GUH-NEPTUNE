"""Proves the three drones are independent: they share information only over the
simulated radio, and each LLM agent sees only its own drone's knowledge.

    python tests/test_isolation.py        (no extra packages needed)
    pytest tests/                          (also works if you have pytest)

No API key or network needed: the LLM is replaced by a stand-in.
"""
import json
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import agent  # noqa: E402
import drone  # noqa: E402
import sim  # noqa: E402
from world import COMMS_RANGE, GRID_SIZE, World, dist  # noqa: E402

# The test's own references, captured at import so a bug in the sim can't fool the check.
_LOS = World.line_of_sight
_RANGE = COMMS_RANGE


def _run(s, llm=False):
    while s.status not in ("complete", "failed"):
        if llm:
            s.poll_agents()
            if s.agents_busy():
                time.sleep(0.001)
                continue
        s.step()
    return s


def test_line_of_sight_is_symmetric():
    """If A can hear B, B can hear A."""
    w, r = World(1), random.Random(0)
    for _ in range(20000):
        a = (r.randrange(GRID_SIZE), r.randrange(GRID_SIZE))
        b = (r.randrange(GRID_SIZE), r.randrange(GRID_SIZE))
        assert w.line_of_sight(a, b) == w.line_of_sight(b, a), f"LOS differs {a}<->{b}"


def test_radio_off_means_no_shared_knowledge():
    """With the radio off for a whole run, each drone knows only what its own sensors saw."""
    saw_it = set()
    orig = drone.Drone.move_and_sense

    def spy(self, tick):
        r = orig(self, tick)
        if r == "FOUND":
            saw_it.add(self.id)
        return r

    drone.Drone.move_and_sense = spy
    try:
        for seed in range(1, 11):
            saw_it.clear()
            s = sim.Sim(seed, "ours")
            s.comms_enabled = False
            _run(s)
            assert s.metrics["msgs_sent"] == 0
            for d in s.drones:
                assert d.searched == d.my_cells, f"seed {seed} drone {d.id}: map has cells it never swept"
                for j, t in d.teammates.items():
                    assert t["last_heard"] == 0, f"seed {seed} drone {d.id} heard from {j}"
                    assert t["position"] == s.world.base, f"seed {seed} drone {d.id} knows {j}'s real position"
                if d.hiker_found:
                    assert d.id in saw_it, f"seed {seed} drone {d.id} knows about a hiker it never saw"
    finally:
        drone.Drone.move_and_sense = orig


def test_every_message_is_physically_possible():
    """Every delivered message: radio on, both drones alive, within range, clear line of sight.
    And nobody hears from a drone after it is killed."""
    orig = drone.Drone.receive
    ctx = {}
    count = [0]

    def spy(self, msg, tick):
        s = ctx["sim"]
        sender = s.drones[msg["sender"]]
        count[0] += 1
        assert s.comms_enabled, "delivered while comms disabled"
        assert sender.alive and self.alive, "delivered to/from a dead drone"
        assert dist(sender.pos, self.pos) <= _RANGE, "delivered out of range"
        assert _LOS(s.world, sender.pos, self.pos), "delivered through a mountain"
        return orig(self, msg, tick)

    drone.Drone.receive = spy
    try:
        for seed in range(1, 21):
            s = sim.Sim(seed, "ours")
            s.packet_loss = 0.5
            s.scenario = {"kill_at": 60, "kill_id": 2}
            ctx["sim"] = s
            _run(s)
            for d in s.drones[:2]:
                assert d.teammates[2]["last_heard"] <= 60, f"seed {seed}: heard from drone 2 after it died"
    finally:
        drone.Drone.receive = orig
    assert count[0] > 1000, "too few deliveries to be a meaningful test"


def test_each_llm_call_sees_only_its_own_drone():
    """Each agent call carries its own drone's knowledge and memory, never the raw map."""
    payloads = []
    orig_call, orig_avail = agent._call_llm, agent.llm_available

    def fake(payload, n, timeout):
        payloads.append(payload)
        return 0, "stand-in", {"in": 0, "out": 0}, 0.0

    agent._call_llm, agent.llm_available = fake, (lambda: True)
    try:
        for seed in range(1, 7):
            s = sim.Sim(seed, "ours")
            s.agent_mode, s.wait_for_agents = "llm", True
            start = len(payloads)
            _run(s, llm=True)
            for p in payloads[start:]:
                me = p["drone"]
                assert "searched_cells" not in json.dumps(p), "raw searched cells sent to the LLM"
                own_ticks = {e["tick"] for e in s.log if e["drone"] == me}
                for m in p["my_recent_decisions"]:
                    assert m["tick"] <= p["tick"], "memory from the future"
                    assert m["tick"] in own_ticks, f"drone {me} was given another drone's memory"
    finally:
        agent._call_llm, agent.llm_available = orig_call, orig_avail
    assert len(payloads) > 30, "too few LLM calls to be a meaningful test"


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        t0 = time.time()
        try:
            t()
            print(f"PASS  {t.__name__}  ({time.time() - t0:.1f}s)")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
