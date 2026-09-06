"""
Generate a synthetic .obd file matching log_data.c's binary format.

This is for self-testing the parser without needing a real mission file.
It writes the same byte layout that sim.exe produces on Windows / Linux.
"""
from __future__ import annotations

import struct
from pathlib import Path

import numpy as np


def write_synthetic_obd(path: str | Path, n_cycles: int = 200) -> Path:
    """Write a small synthetic .obd file simulating a yoyo dive.

    Returns the path written. Variables included:
        m_present_time(s), m_depth(m), m_pitch(rad), m_yaw(rad),
        m_pos_n(m), m_pos_e(m), water_temp(C), water_cond(S/m).
    """
    path = Path(path)
    sensors = [
        (1, "m_present_time", "s",   0.0),
        (5, "m_depth",        "m",   0.0),
        (6, "m_pitch",        "rad", 0.0),
        (7, "m_yaw",          "rad", 0.0),
        (10, "m_pos_n",       "m",   0.0),
        (11, "m_pos_e",       "m",   0.0),
        (20, "water_temp",    "C",   15.0),
        (21, "water_cond",    "S/m",  4.0),
    ]

    # Build synthetic time series — a simple yoyo + slow heading change
    t = np.linspace(0.0, n_cycles * 0.2, n_cycles)        # 5 Hz logging
    depth = 30.0 + 25.0 * np.sin(2 * np.pi * t / 60.0)    # 60 s yoyo, 5–55 m
    pitch = 0.4 * np.cos(2 * np.pi * t / 60.0)            # ±0.4 rad
    yaw = np.radians(45.0 + 0.5 * t)                      # slow turn
    pos_n = 1.5 * t * np.cos(yaw)
    pos_e = 1.5 * t * np.sin(yaw)
    temp = 18.0 - 0.15 * depth                            # mild thermocline
    cond = 4.0 - 0.005 * depth

    series = {
        1: t.astype(np.float32),
        5: depth.astype(np.float32),
        6: pitch.astype(np.float32),
        7: yaw.astype(np.float32),
        10: pos_n.astype(np.float32),
        11: pos_e.astype(np.float32),
        20: temp.astype(np.float32),
        21: cond.astype(np.float32),
    }

    with open(path, "wb") as f:
        # ASCII header
        f.write(b"MIT_SG_AUV_LAB_v2 SyntheticVehicle\n")
        f.write(b"2026/01/01 00:00:00 GMT     # OS-9 file creation time\n")
        f.write(b"2026/01/01 00:00:00 GMT     # Structure creation time\n")
        for idx, name, units, init in sensors:
            f.write(f"{idx} {name}({units}) {init}\n".encode("ascii"))
        f.write(b"9999 0.00 (2026/01/01)\n")

        # Binary section: per-cycle 10001 marker + per-sensor (idx, val) pairs.
        # Note log_data.c writes the 10001 first, then loops i=0..last_s
        # writing only sensors whose timestamp >= present_time. Here we
        # simply log every variable every cycle for simplicity.
        for c in range(n_cycles):
            f.write(struct.pack("<hf", 10001, 0.0))   # cycle marker + dummy elapsed
            for idx, name, units, init in sensors:
                f.write(struct.pack("<hf", idx, float(series[idx][c])))

    return path


if __name__ == "__main__":
    out = write_synthetic_obd("synthetic.obd")
    print(f"wrote {out} ({out.stat().st_size} bytes)")
