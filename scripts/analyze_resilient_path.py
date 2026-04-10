#!/usr/bin/env python3
"""Analyze a WPILog to verify ResilientFollowPathCommand pause/resume behavior."""

import struct
import sys

import matplotlib.pyplot as plt
from wpiutil.log import DataLogReader

LOG_PATH = sys.argv[1] if len(sys.argv) > 1 else (
    "/home/drew/dev/Robot2026/logs/FRC_20260409_174304.wpilog"
)

# ── Read log ──────────────────────────────────────────────────────────

reader = DataLogReader(LOG_PATH)

# Build entry ID -> name map
entry_meta = {}
for record in reader:
    if record.isStart():
        d = record.getStartData()
        entry_meta[d.entry] = (d.name, d.type)

# Entries we care about
NAMES = {
    "NT:ResilientPath/IsPaused": "is_paused",
    "NT:ResilientPath/TranslationErrorMeters": "error",
    "NT:ResilientPath/RealTrackingErrorMeters": "real_error",
    "NT:ResilientPath/VirtualTimeSeconds": "virtual_time",
    "NT:ResilientPath/TargetPose": "target_pose",
    "NT:/DriveState/Pose": "robot_pose",
    "NT:/Sim/DisturbRobot": "disturb",
}

# Reverse map: entry_id -> short key
eid_to_key = {}
for eid, (name, typ) in entry_meta.items():
    if name in NAMES:
        eid_to_key[eid] = NAMES[name]

# ── Extract time series ───────────────────────────────────────────────

ts_error = ([], [])
ts_real_error = ([], [])
ts_paused = ([], [])
ts_virtual = ([], [])
ts_disturb = ([], [])
ts_robot_x = ([], [])
ts_robot_y = ([], [])
ts_target_x = ([], [])
ts_target_y = ([], [])


def decode_pose2d(raw_bytes):
    """Decode a WPILib struct:Pose2d (3 doubles: x, y, rotation)."""
    if len(raw_bytes) >= 24:
        x, y, _rot = struct.unpack("<ddd", raw_bytes[:24])
        return x, y
    return None, None


reader2 = DataLogReader(LOG_PATH)
for record in reader2:
    if record.isStart() or record.isFinish() or record.isSetMetadata() or record.isControl():
        continue

    eid = record.getEntry()
    key = eid_to_key.get(eid)
    if key is None:
        continue

    t = record.getTimestamp() / 1e6  # microseconds -> seconds

    if key == "error":
        ts_error[0].append(t)
        ts_error[1].append(record.getDouble())
    elif key == "real_error":
        ts_real_error[0].append(t)
        ts_real_error[1].append(record.getDouble())
    elif key == "is_paused":
        ts_paused[0].append(t)
        ts_paused[1].append(1.0 if record.getBoolean() else 0.0)
    elif key == "virtual_time":
        ts_virtual[0].append(t)
        ts_virtual[1].append(record.getDouble())
    elif key == "disturb":
        ts_disturb[0].append(t)
        ts_disturb[1].append(1.0 if record.getBoolean() else 0.0)
    elif key == "robot_pose":
        x, y = decode_pose2d(record.getRaw())
        if x is not None:
            ts_robot_x[0].append(t)
            ts_robot_x[1].append(x)
            ts_robot_y[0].append(t)
            ts_robot_y[1].append(y)
    elif key == "target_pose":
        x, y = decode_pose2d(record.getRaw())
        if x is not None:
            ts_target_x[0].append(t)
            ts_target_x[1].append(x)
            ts_target_y[0].append(t)
            ts_target_y[1].append(y)

# ── Analysis ──────────────────────────────────────────────────────────

print(f"Log: {LOG_PATH}")
print(f"  Error samples:        {len(ts_error[0])}")
print(f"  Real error samples:   {len(ts_real_error[0])}")
print(f"  IsPaused samples:     {len(ts_paused[0])}")
print(f"  VirtualTime samples:  {len(ts_virtual[0])}")
print(f"  DisturbRobot samples: {len(ts_disturb[0])}")
print(f"  Robot pose samples:   {len(ts_robot_x[0])}")
print(f"  Target pose samples:  {len(ts_target_x[0])}")

# Find pause events
pause_starts = []
pause_ends = []
was_paused = False
for i, val in enumerate(ts_paused[1]):
    if val > 0.5 and not was_paused:
        pause_starts.append(ts_paused[0][i])
        was_paused = True
    elif val < 0.5 and was_paused:
        pause_ends.append(ts_paused[0][i])
        was_paused = False

# Find disturb events
disturb_times = []
was_disturbed = False
for i, val in enumerate(ts_disturb[1]):
    if val > 0.5 and not was_disturbed:
        disturb_times.append(ts_disturb[0][i])
        was_disturbed = True
    elif val < 0.5:
        was_disturbed = False

print(f"\n--- Disturb triggers: {len(disturb_times)} ---")
for dt in disturb_times:
    print(f"  t = {dt:.3f}s")

print(f"\n--- Pause events: {len(pause_starts)} ---")
for i, ps in enumerate(pause_starts):
    pe = pause_ends[i] if i < len(pause_ends) else None
    duration = f"{pe - ps:.3f}s" if pe else "still paused at log end"
    pe_str = f"{pe:.3f}" if pe else "N/A"
    print(f"  Pause #{i+1}: start={ps:.3f}s  end={pe_str}s  duration={duration}")

    # Max error during pause
    max_err = 0.0
    for j, t in enumerate(ts_error[0]):
        bound = pe if pe else (ts_error[0][-1] if ts_error[0] else ps)
        if ps <= t <= bound:
            max_err = max(max_err, ts_error[1][j])
    print(f"    Max error during pause: {max_err:.3f}m")

    # Check virtual time froze
    vt_during = []
    for j, t in enumerate(ts_virtual[0]):
        bound = pe if pe else (ts_virtual[0][-1] if ts_virtual[0] else ps)
        if ps <= t <= bound:
            vt_during.append(ts_virtual[1][j])
    if len(vt_during) >= 2:
        drift = abs(vt_during[-1] - vt_during[0])
        if drift < 0.05:
            print(f"    Virtual time: FROZEN (drift={drift:.4f}s) ✓")
        else:
            print(f"    Virtual time: DRIFTED {drift:.3f}s ✗")
    else:
        print(f"    Virtual time: not enough samples ({len(vt_during)})")

# Real tracking error stats (excluding paused periods where sim offset inflates it)
if ts_real_error[1]:
    # Filter to only non-paused periods
    running_errors = []
    pause_idx = 0
    for j, t in enumerate(ts_real_error[0]):
        in_pause = False
        for pi, ps in enumerate(pause_starts):
            pe = pause_ends[pi] if pi < len(pause_ends) else 1e20
            if ps <= t <= pe:
                in_pause = True
                break
        if not in_pause:
            running_errors.append(ts_real_error[1][j])
    if running_errors:
        print(f"\n--- Real tracking error (while running, no sim offset) ---")
        print(f"  Mean:  {sum(running_errors)/len(running_errors):.4f}m")
        print(f"  Max:   {max(running_errors):.4f}m")
        print(f"  P95:   {sorted(running_errors)[int(len(running_errors)*0.95)]:.4f}m")
        print(f"  P99:   {sorted(running_errors)[int(len(running_errors)*0.99)]:.4f}m")

# Verify causality: disturb -> pause
print("\n--- Causality check ---")
for dt in disturb_times:
    found = None
    for ps in pause_starts:
        if 0 <= ps - dt <= 2.0:
            found = ps
            break
    if found:
        print(f"  Disturb at {dt:.3f}s -> pause at {found:.3f}s (delay={found-dt:.3f}s) ✓")
    else:
        print(f"  Disturb at {dt:.3f}s -> NO pause within 2s ✗")

# ── Plot ──────────────────────────────────────────────────────────────

# Normalize time to start of first data
t0 = min(
    ts_error[0][0] if ts_error[0] else 1e20,
    ts_paused[0][0] if ts_paused[0] else 1e20,
    ts_virtual[0][0] if ts_virtual[0] else 1e20,
)


def shift(times):
    return [t - t0 for t in times]


fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
fig.suptitle("ResilientFollowPathCommand Analysis", fontsize=14)

# Panel 1: Translation error + thresholds
ax = axes[0]
ax.plot(shift(ts_error[0]), ts_error[1], "b-", linewidth=1, label="Error (w/ sim offset)")
if ts_real_error[0]:
    ax.plot(
        shift(ts_real_error[0]), ts_real_error[1], "c-", linewidth=1, alpha=0.7,
        label="Real Tracking Error",
    )
ax.axhline(y=0.5, color="r", linestyle="--", alpha=0.7, label="Pause Threshold (0.5m)")
ax.axhline(y=0.15, color="g", linestyle="--", alpha=0.7, label="Resume Threshold (0.15m)")
for ps in pause_starts:
    ax.axvline(x=ps - t0, color="r", alpha=0.3, linewidth=2)
for pe in pause_ends:
    ax.axvline(x=pe - t0, color="g", alpha=0.3, linewidth=2)
for dt in disturb_times:
    ax.axvline(x=dt - t0, color="orange", alpha=0.5, linewidth=2, linestyle=":")
ax.set_ylabel("Error (m)")
ax.legend(loc="upper right", fontsize=8)
ax.set_title("Translation Error vs Thresholds")
ax.grid(True, alpha=0.3)

# Panel 2: IsPaused
ax = axes[1]
ax.fill_between(shift(ts_paused[0]), ts_paused[1], step="post", alpha=0.4, color="red")
ax.step(shift(ts_paused[0]), ts_paused[1], "r-", where="post", linewidth=1)
for dt in disturb_times:
    ax.axvline(x=dt - t0, color="orange", alpha=0.5, linewidth=2, linestyle=":")
ax.set_ylabel("Is Paused")
ax.set_yticks([0, 1])
ax.set_yticklabels(["Running", "Paused"])
ax.set_title("Pause State (orange = disturb trigger)")
ax.grid(True, alpha=0.3)

# Panel 3: Virtual time vs wall time
ax = axes[2]
if ts_virtual[0]:
    wall_shifted = shift(ts_virtual[0])
    ax.plot(wall_shifted, ts_virtual[1], "b-", linewidth=1, label="Virtual Time")
    vt0 = ts_virtual[1][0]
    ax.plot(
        wall_shifted,
        [vt0 + (w - wall_shifted[0]) for w in wall_shifted],
        "k--",
        alpha=0.3,
        label="Wall Time (if never paused)",
    )
for ps in pause_starts:
    ax.axvline(x=ps - t0, color="r", alpha=0.3, linewidth=2)
for pe in pause_ends:
    ax.axvline(x=pe - t0, color="g", alpha=0.3, linewidth=2)
ax.set_ylabel("Time (s)")
ax.legend(loc="upper left", fontsize=8)
ax.set_title("Virtual Time vs Wall Time (flat = paused)")
ax.grid(True, alpha=0.3)

# Panel 4: Robot XY vs Target XY
ax = axes[3]
if ts_robot_x[0]:
    ax.plot(shift(ts_robot_x[0]), ts_robot_x[1], "b-", linewidth=1, alpha=0.7, label="Robot X")
    ax.plot(shift(ts_robot_y[0]), ts_robot_y[1], "b--", linewidth=1, alpha=0.7, label="Robot Y")
if ts_target_x[0]:
    ax.plot(shift(ts_target_x[0]), ts_target_x[1], "r-", linewidth=1, alpha=0.7, label="Target X")
    ax.plot(shift(ts_target_y[0]), ts_target_y[1], "r--", linewidth=1, alpha=0.7, label="Target Y")
for ps in pause_starts:
    ax.axvline(x=ps - t0, color="r", alpha=0.3, linewidth=2)
for pe in pause_ends:
    ax.axvline(x=pe - t0, color="g", alpha=0.3, linewidth=2)
ax.set_ylabel("Position (m)")
ax.set_xlabel("Wall Time (s from start)")
ax.legend(loc="upper right", fontsize=8)
ax.set_title("Robot vs Target Position")
ax.grid(True, alpha=0.3)

plt.tight_layout()
out_path = "/home/drew/dev/Robot2026/logs/resilient_path_analysis.png"
plt.savefig(out_path, dpi=150)
print(f"\nPlot saved to: {out_path}")
plt.show()
