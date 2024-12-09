from config import restricted_roads, highways_to_avoid, congested_areas

def get_constrained_edges(graph, restricted_roads, highways_to_avoid, congested_areas):
    constrained_edges = []
    for u, v, key, data in graph.edges(keys=True, data=True):
        road_name = data.get('name', '')
        if isinstance(road_name, list):
            road_name = ' '.join(str(r) for r in road_name)
        road_name_lower = road_name.lower()
        if (any(r.lower() in road_name_lower for r in restricted_roads) or
                any(hwy.lower() in road_name_lower for hwy in highways_to_avoid) or
                any(c.lower() in road_name_lower for c in congested_areas)):
            constrained_edges.append((u, v, key))
    return constrained_edges

