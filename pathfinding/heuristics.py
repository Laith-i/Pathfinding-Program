import math

def haversine_distance(graph, node1, node2):
    R = 6371000
    lat1 = math.radians(graph.nodes[node1]['y'])
    lon1 = math.radians(graph.nodes[node1]['x'])
    lat2 = math.radians(graph.nodes[node2]['y'])
    lon2 = math.radians(graph.nodes[node2]['x'])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

