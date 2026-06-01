from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import re
from typing import Iterable


BEHAVIOR_START_RE = re.compile(
    r'^\s*"([A-Za-z_][A-Za-z0-9_]*)"\s*,\s*0\s*,\s*-?1\.?\s*,\s*0\.?\s*,\s*0\s*,?\s*$'
)
QUOTED_VALUE_RE = re.compile(r'^\s*"([^"]*)"\s*,\s*([^,]+)\s*,?\s*$')
SENSOR_RE = re.compile(r"^\s*sensor:\s+(\S+)\s*(.*)$", re.I)
STATE_RE = re.compile(r"^\s*state:\s+(.+)$", re.I)
BEHAVIOR_RE = re.compile(r"^\s*behavior:\s+(\S+)\s+(\d+)", re.I)
B_ARG_RE = re.compile(r"^\s*b_arg:\s+(\S+)\s*(.*)$", re.I)
VEHICLE_EXTERN_RE = re.compile(r"^\s*extern\s+int\s+([A-Za-z_][A-Za-z0-9_]*)\s*;")
SENSOR_DEF_RE = re.compile(r'^\s*"([^"]+)"\s*,')


@dataclass
class BehaviorArgDef:
    name: str
    default: str = "0"


@dataclass
class BehaviorDef:
    name: str
    args: list[BehaviorArgDef] = field(default_factory=list)


@dataclass
class MissionSensor:
    name: str
    value: str = "0"


@dataclass
class MissionBehavior:
    name: str
    priority: int
    args: list[MissionSensor] = field(default_factory=list)


@dataclass
class Mission:
    filename: str = "mission.mi"
    header: str = "UseSim 1\nMonitorLayeredControl 2\nCountdown 0"
    state: str = "mission"
    sensors: list[MissionSensor] = field(
        default_factory=lambda: [MissionSensor("m_depth(m)", "0")]
    )
    behaviors: list[MissionBehavior] = field(default_factory=list)


@dataclass
class MissionCheck:
    severity: str
    message: str
    behavior: str = ""
    argument: str = ""


@dataclass
class FaultRule:
    trigger_variable: str = "m_present_time"
    operator: str = ">"
    trigger_value: str = "0"
    target_variable: str = "vs_xg"
    action_value: str = "0"


@dataclass
class TimingEstimate:
    behavior: str
    priority: int
    seconds: float | None
    start_depth: float
    end_depth: float
    note: str = ""


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def base_name(name: str) -> str:
    return name.split("(", 1)[0].strip().lower()


def unit_name(name: str) -> str:
    match = re.search(r"\(([^)]*)\)", name)
    return match.group(1).strip().lower() if match else ""


def numeric_value(value: object, default: float = 0.0) -> float:
    try:
        return float(str(value).strip())
    except ValueError:
        try:
            converted, _ = convert_value_expression("", str(value), None)
            return converted
        except ValueError:
            return default


def format_name(name: str, width: int = 30) -> str:
    return f"{name:<{width}}"


def load_behavior_catalog(path: Path | None = None) -> list[BehaviorDef]:
    path = path or repo_root() / "behaviors"
    catalog: list[BehaviorDef] = []
    current: BehaviorDef | None = None

    for line in path.read_text(encoding="latin-1").splitlines():
        match = BEHAVIOR_START_RE.match(line)
        if match:
            current = BehaviorDef(match.group(1), [])
            catalog.append(current)
            continue

        if current is None:
            continue

        match = QUOTED_VALUE_RE.match(line)
        if not match:
            continue
        name = match.group(1).strip()
        if name:
            current.args.append(BehaviorArgDef(name, match.group(2).strip()))

    return catalog


def load_vehicle_variables(path: Path | None = None) -> list[str]:
    path = path or repo_root() / "sensors"
    variables: list[str] = []
    for line in path.read_text(encoding="latin-1").splitlines():
        match = SENSOR_DEF_RE.match(line)
        if match:
            variables.append(base_name(match.group(1)))
    return variables


def load_vehicle_sensor_names(path: Path | None = None) -> list[str]:
    path = path or repo_root() / "sensors"
    names: list[str] = []
    for line in path.read_text(encoding="latin-1").splitlines():
        match = SENSOR_DEF_RE.match(line)
        if match:
            names.append(match.group(1))
    return names


def catalog_by_name(catalog: Iterable[BehaviorDef]) -> dict[str, BehaviorDef]:
    return {item.name: item for item in catalog}


def create_behavior(name: str, priority: int, catalog: dict[str, BehaviorDef]) -> MissionBehavior:
    definition = catalog.get(name)
    args = []
    if definition:
        args = [MissionSensor(arg.name, arg.default) for arg in definition.args]
    return MissionBehavior(name=name, priority=priority, args=args)


def set_arg(behavior: MissionBehavior, arg_base: str, value: object) -> None:
    for arg in behavior.args:
        if base_name(arg.name) == arg_base:
            arg.value = str(value)
            return
    behavior.args.append(MissionSensor(arg_base, str(value)))


def get_arg(behavior: MissionBehavior, arg_base: str, default: float = 0.0) -> float:
    for arg in behavior.args:
        if base_name(arg.name) == arg_base:
            return numeric_value(arg.value, default)
    return default


def get_sensor(mission: Mission, sensor_base: str, default: float = 0.0) -> float:
    for sensor in mission.sensors:
        if base_name(sensor.name) == sensor_base:
            return numeric_value(sensor.value, default)
    return default


def parse_arg(behavior: MissionBehavior, arg_base: str) -> float | None:
    for arg in behavior.args:
        if base_name(arg.name) == arg_base:
            try:
                return convert_value_expression(arg.name, arg.value, behavior)[0]
            except ValueError:
                return None
    return None


def speed_for_time_arg(behavior: MissionBehavior, arg_base: str) -> float:
    if behavior.name == "survey_dead_reckon":
        if arg_base == "trans_time":
            return get_arg(behavior, "transit_speed", 0.0)
        if arg_base in ("time_leg1", "time_leg2"):
            return get_arg(behavior, "survey_speed", 0.0)
    speed = get_arg(behavior, "speed", 0.0)
    if speed <= 0.0:
        speed = get_arg(behavior, "transit_speed", 0.0)
    if speed <= 0.0:
        speed = get_arg(behavior, "survey_speed", 0.0)
    return speed


def _parse_number_unit(text: str) -> tuple[float, str]:
    match = re.match(r"^\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)\s*([A-Za-z/_]+)?\s*$", text)
    if not match:
        raise ValueError(f"Cannot parse value '{text}'")
    return float(match.group(1)), (match.group(2) or "").lower()


def convert_value_expression(
    arg_name: str,
    text: str,
    behavior: MissionBehavior | None = None,
) -> tuple[float, str]:
    """Convert a user expression to the simulator-native value.

    Returns (value, note).  Supported conveniences include angle degrees for
    radian fields, distance/speed/time units, and distance expressions in a
    time field when the behavior has a speed argument.
    """
    raw = str(text).strip()
    if not raw:
        return 0.0, ""

    value, supplied_unit = _parse_number_unit(raw)
    expected = unit_name(arg_name)
    arg_base = base_name(arg_name)
    note = ""

    distance_units = {
        "m": 1.0,
        "meter": 1.0,
        "meters": 1.0,
        "km": 1000.0,
        "cm": 0.01,
        "mm": 0.001,
        "nm": 1852.0,
        "nmi": 1852.0,
        "ft": 0.3048,
    }
    time_units = {
        "s": 1.0,
        "sec": 1.0,
        "secs": 1.0,
        "second": 1.0,
        "seconds": 1.0,
        "min": 60.0,
        "mins": 60.0,
        "hr": 3600.0,
        "hrs": 3600.0,
        "h": 3600.0,
    }
    speed_units = {
        "m/s": 1.0,
        "mps": 1.0,
        "kn": 0.514444,
        "knot": 0.514444,
        "knots": 0.514444,
        "km/hr": 1000.0 / 3600.0,
        "km/h": 1000.0 / 3600.0,
    }

    if supplied_unit in ("deg", "degree", "degrees") and "rad" in expected:
        return math.radians(value), f"{raw} -> {math.radians(value):.6g} rad"
    if supplied_unit == "rad" and ("rad" in expected or arg_base in ("heading", "turn")):
        return value, ""

    distance_bases = (
        "depth", "top_depth", "bottom_depth", "end_depth", "max_depth",
        "min_depth", "cutoff_depth", "min_altitude", "m_depth",
        "m_altitude", "m_pos_n", "m_pos_e", "real_x", "real_y",
        "real_z", "vs_xg", "vs_yg", "vs_zg", "vs_xb", "vs_yb",
        "vs_zb",
    )
    time_bases = (
        "time", "max_time", "timeout", "duration", "m_present_time",
    )

    if supplied_unit in distance_units and ("m" in expected or arg_base in distance_bases):
        converted = value * distance_units[supplied_unit]
        return converted, f"{raw} -> {converted:.6g} m"

    if supplied_unit in speed_units and ("m/s" in expected or arg_base == "speed"):
        converted = value * speed_units[supplied_unit]
        return converted, f"{raw} -> {converted:.6g} m/s"

    if supplied_unit in time_units and ("s" in expected or "sec" in expected or arg_base in time_bases):
        converted = value * time_units[supplied_unit]
        return converted, f"{raw} -> {converted:.6g} s"

    if (
        supplied_unit in distance_units
        and behavior is not None
        and (arg_base in ("time", "max_time", "duration") or arg_base.endswith("_time") or arg_base.startswith("time_"))
    ):
        speed = speed_for_time_arg(behavior, arg_base)
        if speed <= 0:
            raise ValueError(f"Cannot convert {raw} to time without a positive speed")
        distance = value * distance_units[supplied_unit]
        converted = distance / speed
        return converted, f"{raw} at {speed:.3g} m/s -> {converted:.6g} s"

    if supplied_unit:
        raise ValueError(f"Unit '{supplied_unit}' is not valid for {arg_name}")
    return value, note


def normalize_behavior_values(behavior: MissionBehavior) -> list[str]:
    notes: list[str] = []
    for arg in behavior.args:
        try:
            converted, note = convert_value_expression(arg.name, arg.value, behavior)
        except ValueError:
            continue
        if note:
            arg.value = f"{converted:.8g}"
            notes.append(f"{behavior.name}.{base_name(arg.name)}: {note}")
    return notes


def parse_mission_text(text: str, filename: str = "imported.mi") -> Mission:
    mission = Mission(filename=filename, header="", state="", sensors=[], behaviors=[])
    header_lines: list[str] = []
    current: MissionBehavior | None = None

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue

        match = SENSOR_RE.match(line)
        if match:
            mission.sensors.append(MissionSensor(match.group(1), match.group(2).strip()))
            current = None
            continue

        match = STATE_RE.match(line)
        if match:
            mission.state = match.group(1).strip()
            current = None
            continue

        match = BEHAVIOR_RE.match(line)
        if match:
            current = MissionBehavior(match.group(1), int(match.group(2)), [])
            mission.behaviors.append(current)
            continue

        match = B_ARG_RE.match(line)
        if match and current is not None:
            current.args.append(MissionSensor(match.group(1), match.group(2).strip()))
            continue

        if current is None:
            header_lines.append(line)

    mission.header = "\n".join(header_lines).strip() or "UseSim 1\nMonitorLayeredControl 2\nCountdown 0"
    if not mission.state:
        mission.state = "mission"
    if not mission.sensors:
        mission.sensors.append(MissionSensor("m_depth(m)", "0"))
    mission.behaviors.sort(key=lambda item: item.priority)
    return mission


def build_mission_text(mission: Mission) -> str:
    lines: list[str] = []
    for line in mission.header.splitlines():
        line = line.strip()
        if line:
            lines.append(line)
    lines.append("")
    for sensor in mission.sensors:
        lines.append(f"sensor: {format_name(sensor.name)} {sensor.value}")
    lines.append("")
    lines.append(f"state: {mission.state.strip() or 'mission'}")
    lines.append("")

    for behavior in sorted(mission.behaviors, key=lambda item: item.priority):
        lines.append(f"behavior: {behavior.name} {behavior.priority}")
        for arg in behavior.args:
            if arg.name.strip():
                lines.append(f"  b_arg: {format_name(arg.name)} {arg.value}")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def renumber_behaviors(mission: Mission) -> None:
    for priority, behavior in enumerate(mission.behaviors, start=1):
        behavior.priority = priority


def behavior_fastest_time(
    behavior: MissionBehavior,
    start_depth: float,
) -> tuple[float | None, float, str]:
    name = behavior.name
    if name == "mission_timer":
        return 0.0, start_depth, "mission_timer is the limit, not a phase"
    if name in ("depth_envelope", "broadcast", "remote_delay", "acquire_heading"):
        return 0.0, start_depth, "background/safety behavior"

    if name == "setpoint":
        time = parse_arg(behavior, "time")
        cycles = parse_arg(behavior, "cycles")
        depth = parse_arg(behavior, "depth")
        if time is None or cycles is None:
            return None, start_depth, "invalid time or cycles"
        return max(0.0, time) * max(1.0, math.floor(cycles)), (
            depth if depth is not None else start_depth
        ), "time * cycles"

    if name == "setvector":
        time = parse_arg(behavior, "time")
        if time is None:
            return None, start_depth, "invalid time"
        return max(0.0, time), start_depth, "time"

    if name == "survey_dead_reckon":
        trans_time = parse_arg(behavior, "trans_time")
        leg1 = parse_arg(behavior, "time_leg1")
        leg2 = parse_arg(behavior, "time_leg2")
        cycles = parse_arg(behavior, "cycles")
        depth = parse_arg(behavior, "depth")
        if trans_time is None or leg1 is None or leg2 is None or cycles is None:
            return None, start_depth, "invalid transit/leg time or cycles"
        elapsed = 2.0 * max(0.0, cycles) * (max(0.0, leg1) + max(0.0, leg2))
        elapsed = elapsed - max(0.0, leg2) + max(0.0, trans_time)
        return max(0.0, elapsed), (
            depth if depth is not None else start_depth
        ), "transit plus survey cycles"

    if name == "yo_yo":
        max_time = parse_arg(behavior, "max_time")
        if max_time is None:
            return None, start_depth, "invalid max_time"
        return max(0.0, max_time), start_depth, "max_time"

    if name in ("descend", "ascend"):
        angle_name = "descent_angle" if name == "descend" else "ascent_angle"
        speed = parse_arg(behavior, "speed")
        angle = parse_arg(behavior, angle_name)
        end_depth = parse_arg(behavior, "end_depth")
        max_time = parse_arg(behavior, "max_time")
        if speed is None or angle is None or end_depth is None:
            return None, start_depth, f"invalid speed, {angle_name}, or end_depth"
        vertical_speed = abs(speed) * abs(math.sin(angle))
        depth_delta = end_depth - start_depth if name == "descend" else start_depth - end_depth
        if depth_delta <= 0.0:
            estimate = 0.0
        elif vertical_speed > 0.0:
            estimate = depth_delta / vertical_speed
        else:
            return None, start_depth, "zero vertical speed"
        if name == "descend" and max_time is not None and max_time > 0.0:
            estimate = min(estimate, max_time)
        return estimate, end_depth, "depth change / vertical speed"

    for arg_name in ("time", "duration", "timeout", "max_time"):
        value = parse_arg(behavior, arg_name)
        if value is not None and value > 0.0:
            return value, start_depth, arg_name
    return 0.0, start_depth, "no timing estimate"


def mission_planned_max_depth(mission: Mission) -> float:
    depths = [get_sensor(mission, "m_depth", 0.0)]
    for behavior in mission.behaviors:
        for arg_name in ("depth", "end_depth", "top_depth", "bottom_depth", "max_depth"):
            value = parse_arg(behavior, arg_name)
            if value is not None:
                depths.append(value)
    return max(depths) if depths else 0.0


def mission_timing_estimates(mission: Mission) -> tuple[list[TimingEstimate], list[str]]:
    total = 0.0
    estimates: list[TimingEstimate] = []
    problems: list[str] = []
    depth = get_sensor(mission, "m_depth", 0.0)
    planned_max_depth = mission_planned_max_depth(mission)
    ordered = sorted(mission.behaviors, key=lambda item: item.priority, reverse=True)
    skip_paired_yoyo: MissionBehavior | None = None

    for behavior in ordered:
        if behavior is skip_paired_yoyo:
            continue
        if behavior.name == "ascend":
            depth = planned_max_depth
        start_depth = depth
        if behavior.name in ("setpoint", "survey_dead_reckon", "waypoint_2d", "setvector"):
            yoyo = next(
                (candidate for candidate in mission.behaviors
                 if candidate.name == "yo_yo" and candidate.priority == behavior.priority - 1),
                None,
            )
            if yoyo is not None:
                setpoint_time, setpoint_depth, setpoint_note = behavior_fastest_time(behavior, depth)
                yoyo_time, _, yoyo_note = behavior_fastest_time(yoyo, depth)
                if setpoint_time is None or yoyo_time is None:
                    problems.append(f"{behavior.name}/{yoyo.name}: invalid paired timing")
                else:
                    paired_time = max(setpoint_time, yoyo_time)
                    total += paired_time
                    estimates.append(
                        TimingEstimate(
                            f"{behavior.name} + {yoyo.name}",
                            yoyo.priority,
                            paired_time,
                            start_depth,
                            setpoint_depth,
                            f"max of {setpoint_note}, {yoyo_note}",
                        )
                    )
                    depth = setpoint_depth
                skip_paired_yoyo = yoyo
                continue

        estimate, next_depth, note = behavior_fastest_time(behavior, depth)
        if estimate is None:
            problems.append(f"{behavior.name}: {note}")
            continue
        total += estimate
        estimates.append(TimingEstimate(
            behavior.name,
            behavior.priority,
            estimate,
            start_depth,
            next_depth,
            note,
        ))
        depth = next_depth
    return estimates, problems


def mission_fastest_time(mission: Mission) -> tuple[float, list[str], list[str]]:
    estimates, problems = mission_timing_estimates(mission)
    total = sum(item.seconds for item in estimates if item.seconds is not None)
    notes = [
        f"{item.behavior}: {item.seconds:.1f} s ({item.note})"
        for item in estimates
        if item.seconds is not None and item.seconds > 0.0
    ]
    return total, notes, problems


def run_mission_checks(
    mission: Mission,
    catalog: dict[str, BehaviorDef],
) -> list[MissionCheck]:
    checks: list[MissionCheck] = []
    names = {item.name for item in mission.behaviors}
    priorities: dict[int, list[MissionBehavior]] = {}
    for behavior in mission.behaviors:
        priorities.setdefault(behavior.priority, []).append(behavior)
        if behavior.name not in catalog:
            checks.append(MissionCheck("error", f"Unknown behavior '{behavior.name}'.", behavior.name))
            continue
        known = {base_name(arg.name) for arg in catalog[behavior.name].args}
        for arg in behavior.args:
            if base_name(arg.name) not in known:
                checks.append(MissionCheck("warning", f"Unknown argument '{arg.name}'.", behavior.name, arg.name))
            try:
                convert_value_expression(arg.name, arg.value, behavior)
            except ValueError as exc:
                checks.append(MissionCheck(
                    "error",
                    f"Cannot parse {behavior.name}.{base_name(arg.name)}='{arg.value}': {exc}",
                    behavior.name,
                    arg.name,
                ))

    for priority, behaviors in priorities.items():
        if len(behaviors) > 1:
            checks.append(MissionCheck("error", f"Priority {priority} is used by multiple behaviors."))

    if "mission_timer" not in names:
        checks.append(MissionCheck("warning", "No mission_timer behavior is present."))
    if "depth_envelope" not in names:
        checks.append(MissionCheck("warning", "No depth_envelope behavior is present."))

    timer = mission_timer_seconds(mission)
    fastest_time, timing_notes, timing_problems = mission_fastest_time(mission)
    for problem in timing_problems:
        behavior_name = problem.split(":", 1)[0].split("/", 1)[0]
        checks.append(MissionCheck("warning", f"Cannot estimate mission timing for {problem}", behavior_name))
    if timer is not None:
        if timer <= 0.0:
            checks.append(MissionCheck("error", "mission_timer.time must be positive.", "mission_timer", "time"))
        elif fastest_time > 0.0:
            if timer < fastest_time:
                checks.append(MissionCheck(
                    "error",
                    f"mission_timer.time={timer:g} s is shorter than estimated fastest completion time {fastest_time:.1f} s.",
                    "mission_timer",
                    "time",
                ))
            elif timer < fastest_time * 1.1:
                checks.append(MissionCheck(
                    "warning",
                    f"mission_timer.time={timer:g} s leaves less than 10% margin over estimated fastest completion time {fastest_time:.1f} s.",
                    "mission_timer",
                    "time",
                ))
            else:
                checks.append(MissionCheck(
                    "ok",
                    f"Estimated fastest mission completion time is {fastest_time:.1f} s; mission_timer is {timer:g} s.",
                    "mission_timer",
                    "time",
                ))
    elif fastest_time > 0.0:
        checks.append(MissionCheck(
            "warning",
            f"Estimated fastest mission completion time is {fastest_time:.1f} s, but no mission_timer is present.",
            "mission_timer",
            "time",
        ))

    envelope = next((b for b in mission.behaviors if b.name == "depth_envelope"), None)
    max_depth = get_arg(envelope, "max_depth", 0.0) if envelope else None
    min_depth = get_arg(envelope, "min_depth", 0.0) if envelope else None
    cutoff_depth = get_arg(envelope, "cutoff_depth", 0.0) if envelope else None

    def check_depth(behavior: MissionBehavior, arg: str, value: float) -> None:
        if max_depth is not None and max_depth > 0 and value > max_depth:
            checks.append(MissionCheck(
                "error",
                f"{behavior.name}.{arg}={value:g} m is deeper than depth_envelope.max_depth={max_depth:g} m.",
                behavior.name,
                arg,
            ))
        if min_depth is not None and value < min_depth:
            checks.append(MissionCheck(
                "warning",
                f"{behavior.name}.{arg}={value:g} m is shallower than depth_envelope.min_depth={min_depth:g} m.",
                behavior.name,
                arg,
            ))

    for behavior in mission.behaviors:
        for arg_name in ("depth", "end_depth", "top_depth", "bottom_depth"):
            value = get_arg(behavior, arg_name, None)  # type: ignore[arg-type]
            if value is not None:
                check_depth(behavior, arg_name, value)

        if behavior.name == "yo_yo":
            ordered = sorted(mission.behaviors, key=lambda item: item.priority)
            idx = ordered.index(behavior)
            companion = ordered[idx + 1] if idx + 1 < len(ordered) else None
            if companion is None:
                checks.append(MissionCheck("error", "yo_yo has no immediately lower-priority companion.", behavior.name))
            elif companion.priority != behavior.priority + 1:
                checks.append(MissionCheck("error", "yo_yo companion must be at the immediately lower priority.", behavior.name))
            elif companion.name not in ("setpoint", "survey_dead_reckon", "waypoint_2d", "setvector"):
                checks.append(MissionCheck("warning", f"yo_yo companion is {companion.name}; confirm it commands heading and speed.", behavior.name))

            top = get_arg(behavior, "top_depth", 0.0)
            bottom = get_arg(behavior, "bottom_depth", 0.0)
            if top >= bottom:
                checks.append(MissionCheck("error", "yo_yo top_depth should be shallower than bottom_depth.", behavior.name))

        if behavior.name in ("setpoint", "setvector"):
            time = get_arg(behavior, "time", 0.0)
            speed = get_arg(behavior, "speed", 0.0)
            if time <= 0:
                checks.append(MissionCheck("warning", f"{behavior.name} has non-positive time.", behavior.name, "time"))
            if speed <= 0:
                checks.append(MissionCheck("warning", f"{behavior.name} has non-positive speed.", behavior.name, "speed"))

        if behavior.name == "descend":
            pitch = abs(get_arg(behavior, "descent_angle", 0.0))
            speed = abs(get_arg(behavior, "speed", 0.0))
            if speed <= 0 or pitch <= 0:
                checks.append(MissionCheck("warning", "descend needs positive speed and descent_angle.", behavior.name))

        if behavior.name == "ascend":
            pitch = abs(get_arg(behavior, "ascent_angle", 0.0))
            speed = abs(get_arg(behavior, "speed", 0.0))
            if speed <= 0 or pitch <= 0:
                checks.append(MissionCheck("warning", "ascend needs positive speed and ascent_angle.", behavior.name))

    if cutoff_depth is not None and max_depth is not None and cutoff_depth and cutoff_depth <= max_depth:
        checks.append(MissionCheck("warning", "depth_envelope.cutoff_depth should normally be deeper than max_depth.", "depth_envelope"))

    if not checks:
        checks.append(MissionCheck("ok", "No mission-check issues found."))
    return checks


def build_fault_text(rules: list[FaultRule]) -> str:
    lines = [
        "# Odyssey fault injection file",
        "# Format: fault: <condition> -> <target> = <value>",
        "",
    ]
    for rule in rules:
        try:
            trigger_value = f"{convert_value_expression(rule.trigger_variable, rule.trigger_value, None)[0]:.8g}"
        except ValueError:
            trigger_value = str(rule.trigger_value).strip() or "0"
        try:
            action_value = f"{convert_value_expression(rule.target_variable, rule.action_value, None)[0]:.8g}"
        except ValueError:
            action_value = str(rule.action_value).strip() or "0"
        lines.append(
            f"fault: {rule.trigger_variable} {rule.operator} {trigger_value} "
            f"-> {rule.target_variable} = {action_value}"
        )
    return "\n".join(lines).rstrip() + "\n"


def mission_timer_seconds(mission: Mission) -> float | None:
    timer = next((b for b in mission.behaviors if b.name == "mission_timer"), None)
    if timer is None:
        return None
    return get_arg(timer, "time", None)  # type: ignore[arg-type]


def mission_max_depth(mission: Mission) -> float | None:
    depths: list[float] = []
    for behavior in mission.behaviors:
        if behavior.name == "depth_envelope":
            max_depth = get_arg(behavior, "max_depth", None)  # type: ignore[arg-type]
            if max_depth is not None and max_depth > 0:
                depths.append(max_depth)
        for arg_name in ("depth", "end_depth", "top_depth", "bottom_depth", "cutoff_depth"):
            value = get_arg(behavior, arg_name, None)  # type: ignore[arg-type]
            if value is not None:
                depths.append(value)
    return max(depths) if depths else None


def run_fault_checks(
    rules: list[FaultRule],
    mission: Mission,
    vehicle_variables: Iterable[str],
) -> list[MissionCheck]:
    checks: list[MissionCheck] = []
    variables = {v.lower() for v in vehicle_variables}
    timer = mission_timer_seconds(mission)
    max_depth = mission_max_depth(mission)

    for index, rule in enumerate(rules, start=1):
        label = f"fault {index}"
        trigger = rule.trigger_variable.strip()
        target = rule.target_variable.strip()

        if not trigger:
            checks.append(MissionCheck("error", "Missing trigger variable.", label))
        elif trigger.lower() not in variables:
            checks.append(MissionCheck("error", f"Unknown trigger variable '{trigger}'.", label))

        if not target:
            checks.append(MissionCheck("error", "Missing target variable.", label))
        elif target.lower() not in variables:
            checks.append(MissionCheck("error", f"Unknown target variable '{target}'.", label))

        if rule.operator not in (">", "<", ">=", "<=", "==", "!="):
            checks.append(MissionCheck("error", f"Unsupported operator '{rule.operator}'.", label))

        try:
            trigger_value, note = convert_value_expression(trigger, rule.trigger_value, None)
            if note:
                checks.append(MissionCheck("ok", note, label, trigger))
        except ValueError:
            checks.append(MissionCheck("error", f"Cannot parse trigger value '{rule.trigger_value}'.", label, trigger))
            trigger_value = None

        try:
            convert_value_expression(target, rule.action_value, None)
        except ValueError:
            checks.append(MissionCheck("error", f"Cannot parse action value '{rule.action_value}'.", label, target))

        if trigger_value is not None and base_name(trigger) == "m_present_time" and timer is not None:
            if rule.operator in (">", ">=", "==") and trigger_value > timer:
                checks.append(MissionCheck(
                    "warning",
                    f"Trigger time {trigger_value:g} s is beyond mission_timer {timer:g} s.",
                    label,
                    trigger,
                ))

        if trigger_value is not None and base_name(trigger) == "m_depth" and max_depth is not None:
            if rule.operator in (">", ">=", "==") and trigger_value > max_depth:
                checks.append(MissionCheck(
                    "warning",
                    f"Trigger depth {trigger_value:g} m is deeper than planned mission max depth {max_depth:g} m.",
                    label,
                    trigger,
                ))

    if not checks:
        checks.append(MissionCheck("ok", "No failure-script issues found."))
    return checks


def make_yoyo_reference_mission(catalog: dict[str, BehaviorDef]) -> Mission:
    mission = Mission(filename="reference_yoyo.mi", state="reference_yoyo")
    mission.header = "UseSim 1\nMonitorLayeredControl 2\nCountdown 0"
    mission.sensors = [MissionSensor("m_depth(m)", "0.0")]
    mission.behaviors = [
        create_behavior("mission_timer", 1, catalog),
        create_behavior("ascend", 2, catalog),
        create_behavior("depth_envelope", 3, catalog),
        create_behavior("yo_yo", 4, catalog),
        create_behavior("setpoint", 5, catalog),
        create_behavior("descend", 6, catalog),
    ]

    timer, ascend, envelope, yoyo, setpoint, descend = mission.behaviors
    set_arg(timer, "time", 1800)

    set_arg(ascend, "heading", 0)
    set_arg(ascend, "rudder_angle", 0)
    set_arg(ascend, "ascent_angle", 0.349)
    set_arg(ascend, "speed", 1.0)
    set_arg(ascend, "end_depth", 3)
    set_arg(ascend, "drop_descent_weight", 1)
    set_arg(ascend, "drop_ascent_weight", 0)
    set_arg(ascend, "abort_mission_active", 0)

    set_arg(envelope, "max_depth", 110)
    set_arg(envelope, "min_depth", 0)
    set_arg(envelope, "alt_env_active", 0)
    set_arg(envelope, "min_altitude", -1)
    set_arg(envelope, "depth_cutoff_active", 1)
    set_arg(envelope, "cutoff_depth", 120)

    set_arg(yoyo, "desired_pitch", -0.523599)
    set_arg(yoyo, "top_depth", 50)
    set_arg(yoyo, "bottom_depth", 100)
    set_arg(yoyo, "min_altitude", -1)
    set_arg(yoyo, "cycles", 10000)
    set_arg(yoyo, "max_time", 900)

    set_arg(setpoint, "heading", 0)
    set_arg(setpoint, "depth", 75)
    set_arg(setpoint, "speed", 1.5)
    set_arg(setpoint, "time", 900)
    set_arg(setpoint, "cycles", 1)
    set_arg(setpoint, "turn", 0)
    set_arg(setpoint, "desired_pitch", 0)

    set_arg(descend, "heading", 0)
    set_arg(descend, "rudder_angle", 0.174533)
    set_arg(descend, "descent_angle", 0.349066)
    set_arg(descend, "speed", 1.5)
    set_arg(descend, "end_depth", 50)
    set_arg(descend, "min_altitude", -1)
    set_arg(descend, "drop_descent_weight", 0)
    set_arg(descend, "drop_ascent_weight", 0)
    set_arg(descend, "max_time", 600)
    return mission
