import pandas as pd
from datetime import datetime, time as dtime
from osmnx import settings

# OSMnx settings
settings.use_cache = True
settings.log_console = True

# Pandas display options
pd.set_option('display.max_columns', None)
pd.set_option('display.width', 700)
pd.set_option('display.expand_frame_repr', False)

# Current time for constraints
current_time = datetime.combine(datetime.today(), dtime(19, 0, 0))

# Constraints and restrictions
restricted_roads = ["Headington Road", "Divinity road", "Bartlemas road", "Southfield road", "Evelyn Ct", "Marston road"]
highways_to_avoid = ["A34", "A40", "M40", "Eastern By-Pass Road"]
congested_areas = ["Cowley Road", "Iffley Road"]
restricted_times = {
    "Morrell Avenue": (6, 20),
    "Longwall street": (6, 20)
}
road_safety_status = {
    "headington road": "closed",
    "divinity road": "construction",
    "bartlemas road": "unsafe",
    "southfield road": "flood-prone",
    "evelyn ct": "closed",
    "marston road": "unsafe"
}

safe_patterns = [r'\bhospital\b', r'\bpolice station\b', r'\bfire station\b']
risky_patterns = [r'flood-prone', r'unsafe', r'construction', r'closed']

# Start and end points
start_point = (51.755890406539564, -1.2242235228594276)  # Headington Campus
end_point = (51.706047, -1.221047)  # Oxford Stadium (Kassam Stadium)

