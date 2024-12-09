import random
import networkx as nx

def select_landmarks(graph, num_landmarks):
    nodes = list(graph.nodes())
    landmarks = random.sample(nodes, num_landmarks)
    return landmarks

def precompute_landmark_distances(graph, landmarks):
    distance_to_landmark = {landmark: {} for landmark in landmarks}
    distance_from_landmark = {landmark: {} for landmark in landmarks}
    for landmark in landmarks:
        length_to = nx.single_source_dijkstra_path_length(graph, landmark, weight='length')
        distance_to_landmark[landmark] = length_to
        length_from = nx.single_source_dijkstra_path_length(graph.reverse(copy=False), landmark, weight='length')
        distance_from_landmark[landmark] = length_from

    return distance_to_landmark, distance_from_landmark

def landmark_heuristic(n, t, distance_to_landmark, distance_from_landmark, landmarks):
    estimates = []
    for landmark in landmarks:
        d_ln = distance_from_landmark[landmark].get(n, float('inf'))
        d_lt = distance_from_landmark[landmark].get(t, float('inf'))
        d_nl = distance_to_landmark[landmark].get(n, float('inf'))
        d_tl = distance_to_landmark[landmark].get(t, float('inf'))

        forward_estimate = d_tl - d_nl
        backward_estimate = d_ln - d_lt
        estimates.append(forward_estimate)
        estimates.append(backward_estimate)

    return max(estimates)

