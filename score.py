"""
score.py — MUTABLE scoring function for autoresearch-drone.

The agent may modify this file to reshape the optimization landscape.

Takes raw run data from prepare.py, returns a single float. LOWER IS BETTER.

Hard constraint: the global optimum must correspond to the fastest complete
lap with all gates passed and no crash.
"""


def compute(results: dict) -> float:
    """
    Compute a composite score from raw run results.

    Args:
        results: dict with keys:
            - lap_time: float (seconds, 0.0 if DNF)
            - gates_passed: int
            - total_gates: int
            - crashed: bool
            - timeout: bool
            - total_elapsed: float (seconds)
            - gate_details: list of gate passage records

    Returns:
        float: composite score, lower is better. 9999.0 for total failure.
    """
    gates_passed = results["gates_passed"]
    total_gates = results["total_gates"]
    lap_time = results["lap_time"]
    crashed = results["crashed"]
    timeout = results["timeout"]
    max_time = 480.0  # 8 minute max

    # Case 1: Complete lap (all gates passed, no crash)
    # Score is just the lap time — this is what we're optimizing.
    if gates_passed == total_gates and lap_time > 0:
        return lap_time

    # Case 2: Partial completion (some gates passed but didn't finish)
    # Score = base penalty - credit for gates passed.
    # This makes passing 7/8 gates much better than passing 2/8,
    # creating a smooth gradient toward completion.
    gate_fraction = gates_passed / total_gates
    base_penalty = max_time + 100.0  # worse than any complete lap

    # Each gate passed reduces the score significantly
    gate_credit = gate_fraction * 200.0

    # Small bonus for how quickly gates were passed (encourages speed even on partials)
    if gates_passed > 0 and results["gate_details"]:
        last_gate_time = results["gate_details"][-1]["time"] - (
            results["total_elapsed"] - results["total_elapsed"]
        )
        # Approximate: use total elapsed as proxy
        time_credit = max(0, (max_time - results["total_elapsed"]) / max_time) * 20.0
    else:
        time_credit = 0.0

    score = base_penalty - gate_credit - time_credit

    # Case 3: Crash penalty (on top of partial completion score)
    if crashed:
        score += 50.0

    # Case 4: Total failure (no gates passed)
    if gates_passed == 0:
        score = 9999.0

    return round(score, 3)
