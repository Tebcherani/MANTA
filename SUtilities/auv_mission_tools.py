from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
from typing import Dict, Iterable, List, Sequence, Set, Tuple


SENSOR_RE = re.compile(r"^\s*sensor:\s+(\S+)")
BEHAVIOR_RE = re.compile(r"^\s*behavior:\s+(\S+)")
B_ARG_RE = re.compile(r"^\s*b_arg:\s+(\S+)")
OBD_HEADER_RE = re.compile(r"^\s*(\d+)\s+(\S+)")
OBD_NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(?:\(.*\))?$")
VEHICLE_EXTERN_RE = re.compile(r"^\s*extern\s+int\s+([A-Za-z_][A-Za-z0-9_]*)\s*;")


@dataclass
class MissionScan:
    sensor_count: int
    behavior_count: int
    b_arg_count: int
    missing_sensors: List[str]
    missing_behaviors: List[str]
    missing_b_args: List[str]


@dataclass
class TrimResult:
    text: str
    removed_sensors: List[str]
    removed_behaviors: List[str]
    removed_b_args: List[str]


@dataclass
class ObdHeaderEntry:
    index: int
    name: str


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_vehicle_h() -> Path:
    return repo_root() / "vehicle.h"


def default_behaviors_file() -> Path:
    return repo_root() / "behaviors"


def iter_input_files(folder: Path, suffix: str, recursive: bool = False) -> List[Path]:
    pattern = "**/*" + suffix if recursive else "*" + suffix
    return sorted(
        (p for p in folder.glob(pattern) if p.is_file()),
        key=lambda p: str(p).lower(),
    )


def base_name(name_with_units: str) -> str:
    return name_with_units.split("(", 1)[0].strip().lower()


def unique_in_order(values: Iterable[str]) -> List[str]:
    seen: Set[str] = set()
    out: List[str] = []
    for value in values:
        key = value.lower()
        if key not in seen:
            seen.add(key)
            out.append(value)
    return out


def parse_vehicle_variables(vehicle_h: Path) -> Set[str]:
    variables: Set[str] = set()
    with vehicle_h.open("r", encoding="latin-1") as f:
        for line in f:
            match = VEHICLE_EXTERN_RE.match(line)
            if match:
                variables.add(match.group(1).lower())
    return variables


def parse_behavior_defs(behaviors_file: Path) -> Dict[str, Set[str]]:
    behavior_defs: Dict[str, Set[str]] = {}
    current_behavior = ""

    behavior_line = re.compile(
        r'^\s*"([A-Za-z_][A-Za-z0-9_]*)"\s*,\s*0\s*,\s*-?1\.?\s*,\s*0\.?\s*,\s*0\s*,?\s*$'
    )
    quoted_name = re.compile(r'^\s*"([^"]*)"\s*,')

    with behaviors_file.open("r", encoding="latin-1") as f:
        for line in f:
            match = behavior_line.match(line)
            if match:
                current_behavior = match.group(1).lower()
                behavior_defs[current_behavior] = set()
                continue

            if not current_behavior:
                continue

            match = quoted_name.match(line)
            if not match:
                continue

            arg_name = match.group(1).strip()
            if arg_name:
                behavior_defs[current_behavior].add(base_name(arg_name))

    return behavior_defs


def scan_mission_text(
    text: str,
    vehicle_variables: Set[str],
    behavior_defs: Dict[str, Set[str]],
) -> MissionScan:
    missing_sensors: List[str] = []
    missing_behaviors: List[str] = []
    missing_b_args: List[str] = []
    current_behavior = ""
    sensor_count = 0
    behavior_count = 0
    b_arg_count = 0

    for line in text.splitlines():
        match = SENSOR_RE.match(line)
        if match:
            sensor_count += 1
            name = base_name(match.group(1))
            if name not in vehicle_variables:
                missing_sensors.append(name)
            continue

        match = BEHAVIOR_RE.match(line)
        if match:
            behavior_count += 1
            behavior_name = match.group(1)
            current_behavior = behavior_name.lower()
            if current_behavior not in behavior_defs:
                missing_behaviors.append(behavior_name)
            continue

        match = B_ARG_RE.match(line)
        if match:
            b_arg_count += 1
            arg_name = base_name(match.group(1))
            if current_behavior and current_behavior in behavior_defs:
                if arg_name not in behavior_defs[current_behavior]:
                    missing_b_args.append(current_behavior + "." + arg_name)
            elif not current_behavior:
                missing_b_args.append("<no_behavior>." + arg_name)

    return MissionScan(
        sensor_count=sensor_count,
        behavior_count=behavior_count,
        b_arg_count=b_arg_count,
        missing_sensors=unique_in_order(missing_sensors),
        missing_behaviors=unique_in_order(missing_behaviors),
        missing_b_args=unique_in_order(missing_b_args),
    )


def scan_mission_file(
    path: Path,
    vehicle_variables: Set[str],
    behavior_defs: Dict[str, Set[str]],
) -> MissionScan:
    return scan_mission_text(
        path.read_text(encoding="latin-1"),
        vehicle_variables,
        behavior_defs,
    )


def trim_mission_text(
    text: str,
    vehicle_variables: Set[str],
    behavior_defs: Dict[str, Set[str]],
    drop_missing_behaviors: bool = True,
    drop_missing_b_args: bool = True,
) -> TrimResult:
    output: List[str] = []
    removed_sensors: List[str] = []
    removed_behaviors: List[str] = []
    removed_b_args: List[str] = []
    current_behavior = ""
    current_behavior_known = True

    for line in text.splitlines(keepends=True):
        match = SENSOR_RE.match(line)
        if match:
            name = base_name(match.group(1))
            if name not in vehicle_variables:
                removed_sensors.append(name)
                continue
            output.append(line)
            continue

        match = BEHAVIOR_RE.match(line)
        if match:
            behavior_name = match.group(1)
            current_behavior = behavior_name.lower()
            current_behavior_known = current_behavior in behavior_defs
            if not current_behavior_known and drop_missing_behaviors:
                removed_behaviors.append(behavior_name)
                continue
            output.append(line)
            continue

        match = B_ARG_RE.match(line)
        if match:
            arg_name = base_name(match.group(1))
            if not current_behavior_known and drop_missing_behaviors:
                continue
            if (
                drop_missing_b_args
                and current_behavior
                and current_behavior in behavior_defs
                and arg_name not in behavior_defs[current_behavior]
            ):
                removed_b_args.append(current_behavior + "." + arg_name)
                continue
            output.append(line)
            continue

        output.append(line)

    return TrimResult(
        text="".join(output),
        removed_sensors=unique_in_order(removed_sensors),
        removed_behaviors=unique_in_order(removed_behaviors),
        removed_b_args=unique_in_order(removed_b_args),
    )


def extract_mission_from_slg_text(text: str) -> str:
    lines = text.splitlines(keepends=True)
    start = None

    for idx, line in enumerate(lines):
        if "mission file:" in line.lower():
            start = idx + 1
            break

    if start is None:
        for idx, line in enumerate(lines):
            if is_mission_line(line):
                start = idx
                break

    if start is None:
        return ""

    end = len(lines)
    for idx in range(start, len(lines)):
        if is_slg_mission_end(lines[idx]):
            end = idx
            break

    mission = "".join(lines[start:end]).strip()
    if not mission:
        return ""
    return mission + "\n"


def is_mission_line(line: str) -> bool:
    stripped = line.lstrip()
    return bool(
        stripped.startswith("Use")
        or stripped.startswith("Monitor")
        or stripped.startswith("Countdown")
        or stripped.startswith("InitDataFileSize")
        or stripped.startswith("state:")
        or stripped.startswith("sensor:")
        or stripped.startswith("behavior:")
        or stripped.startswith("b_arg:")
    )


def is_slg_mission_end(line: str) -> bool:
    stripped = line.strip()
    if not stripped:
        return False

    end_prefixes = (
        "Data module structure creation time",
        "vehicle.h structure creation time",
        "Data Logger",
        "Thruster ...",
        "Rudder ...",
        "Elevator ...",
        "Paroscientific",
        "KVH ",
        "Comms Handler",
        "EdgeTech",
    )
    return any(stripped.startswith(prefix) for prefix in end_prefixes)


def extract_mission_from_slg_file(path: Path) -> str:
    return extract_mission_from_slg_text(path.read_text(encoding="latin-1"))


def parse_obd_header_entries(
    path: Path,
    max_header_bytes: int = 1024 * 1024,
) -> Tuple[List[ObdHeaderEntry], bool]:
    with path.open("rb") as f:
        raw = f.read(max_header_bytes)

    entries: List[ObdHeaderEntry] = []
    found_end = False
    text = raw.decode("latin-1", errors="ignore")

    for line in text.splitlines():
        match = OBD_HEADER_RE.match(line)
        if not match:
            continue

        index = int(match.group(1))
        if index == 9999:
            found_end = True
            break

        name = match.group(2)
        if OBD_NAME_RE.match(name):
            entries.append(ObdHeaderEntry(index=index, name=name))

    return entries, found_end


def is_known_behavior_log_entry(
    variable_base_name: str,
    behavior_defs: Dict[str, Set[str]],
) -> bool:
    if variable_base_name in behavior_defs:
        return True

    for behavior_name in sorted(behavior_defs, key=len, reverse=True):
        prefix = behavior_name + "_"
        if variable_base_name.startswith(prefix):
            arg_name = variable_base_name[len(prefix):]
            return arg_name in behavior_defs[behavior_name]

    return False


def field_sensor_variables_from_obd(
    path: Path,
    behavior_defs: Dict[str, Set[str]],
) -> Tuple[List[str], bool]:
    entries, found_end = parse_obd_header_entries(path)
    variables: List[str] = []

    for entry in entries:
        name = base_name(entry.name)
        if is_known_behavior_log_entry(name, behavior_defs):
            continue
        variables.append(name)

    return unique_in_order(variables), found_end


def semicolon_join(values: Sequence[str]) -> str:
    return ";".join(values)
