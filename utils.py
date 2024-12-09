import re
from config import safe_patterns, risky_patterns, restricted_times, road_safety_status, highways_to_avoid, congested_areas

def matches_pattern(road_name, patterns):
    for pattern in patterns:
        if re.search(pattern, road_name.lower()):
            return True
    return False

def is_road_safe(road_name, current_time=None, avoid_highways=True, avoid_congested_areas=True):
    if isinstance(road_name, list):
        road_name = ' '.join(str(r) for r in road_name)
    road_name_lower = road_name.lower()

    # Check against known unsafe conditions
    if road_name_lower in road_safety_status:
        status = road_safety_status[road_name_lower]
        if status in ["unsafe", "flood-prone", "construction", "closed"]:
            return False

    # Time-based restrictions
    if current_time:
        for r_road, (start_hour, end_hour) in restricted_times.items():
            if r_road.lower() in road_name_lower:
                if not (start_hour <= current_time.hour < end_hour):
                    return False

    # Avoid highways if requested
    if avoid_highways:
        if any(hwy.lower() in road_name_lower for hwy in highways_to_avoid):
            return False

    # Avoid congested areas if requested
    if avoid_congested_areas:
        if any(c.lower() in road_name_lower for c in congested_areas):
            return False

    # If a road matches risky patterns, avoid it
    if matches_pattern(road_name, risky_patterns):
        return False

    return True

