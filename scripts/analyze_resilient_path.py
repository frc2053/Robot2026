#!/usr/bin/env python3
"""Analyze a WPILog to verify ResilientFollowPathCommand pause/resume behavior."""

import struct
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from wpiutil.log import DataLogReader

SCRIPT_DIR = Path(__file__).parent.resolve()
LOGS_DIR = SCRIPT_DIR.parent / "logs"


def get_latest_log():
    """Find the most recent .wpilog file in the logs directory."""
    log_files = list(LOGS_DIR.glob("*.wpilog"))
    if not log_files:
        print(f"No .wpilog files found in {LOGS_DIR}")
        sys.exit(1)
    return str(max(log_files, key=lambda p: p.stat().st_mtime))


LOG_PATH = sys.argv[1] if len(sys.argv) > 1 else get_latest_log()

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
    # Also try alternate NT paths
    "NT:Sim/DisturbActive": "disturb_active",
    "NT:/Sim/DisturbActive": "disturb_active",
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
ts_disturb_active = ([], [])
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
    elif key == "disturb_active":
        ts_disturb_active[0].append(t)
        ts_disturb_active[1].append(1.0 if record.getBoolean() else 0.0)
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

print(f"\n{'='*60}")
print("LOG FILE INFO")
print(f"{'='*60}")
print(f"Log: {LOG_PATH}")

# List all available entries in the log
print(f"\n--- All log entries ({len(entry_meta)} total) ---")
resilient_entries = []
for eid, (name, typ) in sorted(entry_meta.items(), key=lambda x: x[1][0]):
    if "ResilientPath" in name or "Sim/" in name or "DriveState" in name:
        resilient_entries.append((name, typ))
        print(f"  {name} ({typ})")

print(f"\n--- Sample counts ---")
print(f"  Error samples:         {len(ts_error[0])}")
print(f"  Real error samples:    {len(ts_real_error[0])}")
print(f"  IsPaused samples:      {len(ts_paused[0])}")
print(f"  VirtualTime samples:   {len(ts_virtual[0])}")
print(f"  DisturbRobot samples:  {len(ts_disturb[0])}")
print(f"  DisturbActive samples: {len(ts_disturb_active[0])}")
print(f"  Robot pose samples:    {len(ts_robot_x[0])}")
print(f"  Target pose samples:   {len(ts_target_x[0])}")

# Time ranges for each data stream
print(f"\n--- Time ranges ---")
if ts_error[0]:
    print(f"  Error:        {ts_error[0][0]:.3f}s - {ts_error[0][-1]:.3f}s")
if ts_paused[0]:
    print(f"  IsPaused:     {ts_paused[0][0]:.3f}s - {ts_paused[0][-1]:.3f}s")
if ts_virtual[0]:
    print(f"  VirtualTime:  {ts_virtual[0][0]:.3f}s - {ts_virtual[0][-1]:.3f}s")
if ts_disturb[0]:
    print(f"  DisturbRobot: {ts_disturb[0][0]:.3f}s - {ts_disturb[0][-1]:.3f}s")
if ts_disturb_active[0]:
    print(f"  DisturbActiv: {ts_disturb_active[0][0]:.3f}s - {ts_disturb_active[0][-1]:.3f}s")
if ts_target_x[0]:
    print(f"  TargetPose:   {ts_target_x[0][0]:.3f}s - {ts_target_x[0][-1]:.3f}s")

# Sample rate analysis
print(f"\n--- Sample rates ---")
def calc_rate(times):
    if len(times) < 2:
        return 0, 0, 0
    deltas = [times[i+1] - times[i] for i in range(len(times)-1)]
    avg = sum(deltas) / len(deltas)
    return 1/avg if avg > 0 else 0, min(deltas), max(deltas)

if ts_error[0]:
    rate, min_d, max_d = calc_rate(ts_error[0])
    print(f"  Error:       {rate:.1f} Hz (gaps: {min_d*1000:.1f}ms - {max_d*1000:.1f}ms)")
if ts_paused[0]:
    rate, min_d, max_d = calc_rate(ts_paused[0])
    print(f"  IsPaused:    {rate:.1f} Hz (gaps: {min_d*1000:.1f}ms - {max_d*1000:.1f}ms)")
if ts_virtual[0]:
    rate, min_d, max_d = calc_rate(ts_virtual[0])
    print(f"  VirtualTime: {rate:.1f} Hz (gaps: {min_d*1000:.1f}ms - {max_d*1000:.1f}ms)")

# Gap detection - find periods where data stops coming
print(f"\n--- Gap detection (gaps > 100ms) ---")
def find_gaps(times, name, threshold=0.1):
    gaps = []
    for i in range(len(times)-1):
        delta = times[i+1] - times[i]
        if delta > threshold:
            gaps.append((times[i], times[i+1], delta))
    return gaps

if ts_error[0]:
    gaps = find_gaps(ts_error[0], "Error")
    if gaps:
        print(f"  Error gaps: {len(gaps)}")
        for start, end, dur in gaps[:5]:
            print(f"    {start:.3f}s - {end:.3f}s ({dur*1000:.0f}ms)")
        if len(gaps) > 5:
            print(f"    ... and {len(gaps)-5} more")
    else:
        print(f"  Error: no gaps > 100ms")

if ts_paused[0]:
    gaps = find_gaps(ts_paused[0], "IsPaused")
    if gaps:
        print(f"  IsPaused gaps: {len(gaps)}")
        for start, end, dur in gaps[:5]:
            print(f"    {start:.3f}s - {end:.3f}s ({dur*1000:.0f}ms)")
    else:
        print(f"  IsPaused: no gaps > 100ms")

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

# Find disturb events - try both disturb and disturb_active
disturb_times = []
was_disturbed = False

# Use whichever has data
disturb_source = ts_disturb if ts_disturb[0] else ts_disturb_active
disturb_source_name = "DisturbRobot" if ts_disturb[0] else "DisturbActive"

for i, val in enumerate(disturb_source[1]):
    if val > 0.5 and not was_disturbed:
        disturb_times.append(disturb_source[0][i])
        was_disturbed = True
    elif val < 0.5:
        was_disturbed = False

print(f"\nUsing disturb source: {disturb_source_name} ({len(disturb_source[0])} samples)")

print(f"\n{'='*60}")
print("DISTURBANCE ANALYSIS")
print(f"{'='*60}")

PAUSE_THRESHOLD = 0.254  # 10 inches
RESUME_THRESHOLD = 0.10  # ~4 inches

print(f"\nPause threshold: {PAUSE_THRESHOLD}m")
print(f"Resume threshold: {RESUME_THRESHOLD}m")

print(f"\n--- Disturb triggers: {len(disturb_times)} ---")
for idx, dt in enumerate(disturb_times):
    print(f"\n  === DISTURBANCE #{idx+1} at t={dt:.3f}s ===")

    # Find disturbance end (next time disturb goes false, or +2s)
    disturb_end = dt + 2.0
    for i, t in enumerate(ts_disturb[0]):
        if t > dt and ts_disturb[1][i] < 0.5:
            disturb_end = t
            break
    print(f"  Duration: {disturb_end - dt:.3f}s")

    # Sample counts during disturbance window
    print(f"\n  Sample counts during disturbance window [{dt:.3f}s - {disturb_end:.3f}s]:")
    error_count = sum(1 for t in ts_error[0] if dt <= t <= disturb_end)
    paused_count = sum(1 for t in ts_paused[0] if dt <= t <= disturb_end)
    virtual_count = sum(1 for t in ts_virtual[0] if dt <= t <= disturb_end)
    target_count = sum(1 for t in ts_target_x[0] if dt <= t <= disturb_end)
    print(f"    Error samples:       {error_count}")
    print(f"    IsPaused samples:    {paused_count}")
    print(f"    VirtualTime samples: {virtual_count}")
    print(f"    TargetPose samples:  {target_count}")

    # First and last sample times for each stream during disturbance
    print(f"\n  First/last sample timestamps during disturbance:")
    for name, times in [("Error", ts_error[0]), ("IsPaused", ts_paused[0]),
                         ("VirtualTime", ts_virtual[0]), ("TargetPose", ts_target_x[0])]:
        samples_in_window = [t for t in times if dt <= t <= disturb_end]
        if samples_in_window:
            first = min(samples_in_window)
            last = max(samples_in_window)
            print(f"    {name:12}: first={first:.3f}s last={last:.3f}s (delay from disturb start: {first-dt:.3f}s)")
        else:
            print(f"    {name:12}: NO SAMPLES during disturbance!")

    # Raw sample timeline during disturbance (interleaved)
    print(f"\n  Raw sample timeline (all streams interleaved):")
    all_samples = []
    for t in ts_error[0]:
        if dt - 0.1 <= t <= disturb_end + 0.5:
            all_samples.append((t, "Error", ts_error[1][ts_error[0].index(t)]))
    for i, t in enumerate(ts_paused[0]):
        if dt - 0.1 <= t <= disturb_end + 0.5:
            all_samples.append((t, "IsPaused", "TRUE" if ts_paused[1][i] > 0.5 else "false"))
    for i, t in enumerate(ts_virtual[0]):
        if dt - 0.1 <= t <= disturb_end + 0.5:
            all_samples.append((t, "VirtualTime", ts_virtual[1][i]))

    all_samples.sort(key=lambda x: x[0])
    if all_samples:
        prev_t = all_samples[0][0]
        for t, name, val in all_samples[:30]:  # Limit to 30 entries
            gap = t - prev_t
            gap_str = f"(+{gap*1000:.0f}ms)" if gap > 0.05 else ""
            if name == "Error":
                print(f"    t={t:.3f}s {gap_str:10} {name:12} = {val:.4f}m")
            elif name == "IsPaused":
                print(f"    t={t:.3f}s {gap_str:10} {name:12} = {val}")
            else:
                print(f"    t={t:.3f}s {gap_str:10} {name:12} = {val:.3f}s")
            prev_t = t
        if len(all_samples) > 30:
            print(f"    ... ({len(all_samples) - 30} more samples)")
    else:
        print(f"    NO SAMPLES near disturbance window!")

    # Show error values during this disturbance window
    print(f"\n  Error during disturbance (threshold={PAUSE_THRESHOLD}m):")
    errors_during = []
    for i, t in enumerate(ts_error[0]):
        if dt <= t <= disturb_end + 1.0:  # Include 1s after for recovery
            errors_during.append((t, ts_error[1][i]))

    if errors_during:
        max_err = max(e[1] for e in errors_during)
        print(f"    Max error: {max_err:.4f}m {'> threshold ✓' if max_err > PAUSE_THRESHOLD else '< threshold ✗'}")
        print(f"    Error timeline:")
        for t, err in errors_during[::max(1, len(errors_during)//10)]:  # Sample ~10 points
            marker = ">>>" if err > PAUSE_THRESHOLD else "   "
            print(f"      {marker} t={t:.3f}s: error={err:.4f}m")
    else:
        print(f"    NO ERROR DATA during disturbance!")

    # Show virtual time during disturbance
    print(f"\n  Virtual time during disturbance:")
    vt_during = []
    for i, t in enumerate(ts_virtual[0]):
        if dt <= t <= disturb_end + 1.0:
            vt_during.append((t, ts_virtual[1][i]))

    if vt_during:
        vt_start = vt_during[0][1]
        vt_end = vt_during[-1][1]
        vt_change = vt_end - vt_start
        wall_change = vt_during[-1][0] - vt_during[0][0]
        print(f"    Virtual time changed: {vt_start:.3f}s -> {vt_end:.3f}s (delta={vt_change:.3f}s)")
        print(f"    Wall time changed: {wall_change:.3f}s")
        if wall_change > 0.1:
            ratio = vt_change / wall_change
            print(f"    Ratio (vt/wall): {ratio:.2f} {'(paused!)' if ratio < 0.5 else '(running)'}")
    else:
        print(f"    NO VIRTUAL TIME DATA during disturbance!")

    # Show IsPaused during disturbance
    print(f"\n  IsPaused during disturbance:")
    paused_during = []
    for i, t in enumerate(ts_paused[0]):
        if dt <= t <= disturb_end + 1.0:
            paused_during.append((t, ts_paused[1][i]))

    if paused_during:
        any_paused = any(p[1] > 0.5 for p in paused_during)
        print(f"    Ever paused: {'YES ✓' if any_paused else 'NO ✗'}")
        for t, p in paused_during[::max(1, len(paused_during)//5)]:
            print(f"      t={t:.3f}s: IsPaused={'TRUE' if p > 0.5 else 'false'}")
    else:
        print(f"    NO ISPAUSED DATA during disturbance!")

    # Show robot pose vs target pose
    print(f"\n  Robot vs Target pose:")
    for i, t in enumerate(ts_robot_x[0]):
        if dt <= t <= disturb_end:
            # Find closest target pose
            target_x, target_y = None, None
            for j, tt in enumerate(ts_target_x[0]):
                if abs(tt - t) < 0.05:
                    target_x = ts_target_x[1][j]
                    target_y = ts_target_y[1][j]
                    break
            if target_x is not None and i < len(ts_robot_y[1]):
                robot_x = ts_robot_x[1][i]
                robot_y = ts_robot_y[1][i]
                dist = ((robot_x - target_x)**2 + (robot_y - target_y)**2)**0.5
                print(f"    t={t:.3f}s: robot=({robot_x:.2f},{robot_y:.2f}) target=({target_x:.2f},{target_y:.2f}) dist={dist:.3f}m")
            if i > 5:  # Limit output
                print(f"    ... (truncated)")
                break

print(f"\n{'='*60}")
print("PAUSE EVENTS")
print(f"{'='*60}")

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
LOGS_DIR.mkdir(exist_ok=True)
out_path = LOGS_DIR / "resilient_path_analysis.png"
plt.savefig(out_path, dpi=150)
print(f"\nPlot saved to: {out_path}")
plt.show()
