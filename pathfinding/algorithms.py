import heapq
import time
import tracemalloc
from collections import deque
from config import current_time
from utils import is_road_safe
from .heuristics import haversine_distance
from .landmarks import landmark_heuristic

def a_star(graph, start, goal):
    tracemalloc.start()
    start_time = time.time()
    open_set = [(0, start, [start], 0)]
    visited = set()
    visited_nodes = 0

    while open_set:
        current_f, current_node, path, current_g = heapq.heappop(open_set)
        if current_node == goal:
            end_time = time.time()
            current_mem = tracemalloc.get_traced_memory()[1] / 10**6
            tracemalloc.stop()
            return path, current_g, visited_nodes, end_time - start_time, current_mem

        if current_node in visited:
            continue
        visited.add(current_node)
        visited_nodes += 1

        for neighbor in graph.neighbors(current_node):
            if neighbor in visited:
                continue
            edge_data = graph.get_edge_data(current_node, neighbor, 0)
            if edge_data is None:
                continue
            edge_length = edge_data.get('length', 1)
            tentative_g = current_g + edge_length
            heuristic = haversine_distance(graph, neighbor, goal)
            tentative_f = tentative_g + heuristic
            heapq.heappush(open_set, (tentative_f, neighbor, path + [neighbor], tentative_g))

    end_time = time.time()
    tracemalloc.stop()
    return None, None, visited_nodes, end_time - start_time, None

def dijkstra(graph, start, goal):
    tracemalloc.start()
    start_time = time.time()
    open_set = [(0, start, [start])]
    visited = set()
    min_cost = {start: 0}
    visited_nodes = 0

    while open_set:
        current_cost, current_node, path = heapq.heappop(open_set)
        if current_node == goal:
            end_time = time.time()
            current_mem = tracemalloc.get_traced_memory()[1] / 10**6
            tracemalloc.stop()
            return path, current_cost, visited_nodes, end_time - start_time, current_mem

        if current_node in visited:
            continue
        visited.add(current_node)
        visited_nodes += 1

        for neighbor in graph.neighbors(current_node):
            edge_data = graph.get_edge_data(current_node, neighbor, 0)
            if edge_data is None:
                continue
            edge_length = edge_data.get('length', 1)
            new_cost = current_cost + edge_length
            if neighbor not in min_cost or new_cost < min_cost[neighbor]:
                min_cost[neighbor] = new_cost
                heapq.heappush(open_set, (new_cost, neighbor, path + [neighbor]))

    end_time = time.time()
    tracemalloc.stop()
    return None, None, visited_nodes, end_time - start_time, None

def bfs(graph, start, goal):
    tracemalloc.start()
    start_time = time.time()
    queue = deque([(start, [start], 0)])
    visited = set()
    visited_nodes = 0

    while queue:
        current_node, path, path_length = queue.popleft()
        if current_node == goal:
            end_time = time.time()
            current_mem = tracemalloc.get_traced_memory()[1] / 10**6
            tracemalloc.stop()
            return path, path_length, visited_nodes, end_time - start_time, current_mem

        if current_node in visited:
            continue
        visited.add(current_node)
        visited_nodes += 1

        for neighbor in graph.neighbors(current_node):
            if neighbor not in visited:
                edge_data = graph.get_edge_data(current_node, neighbor, 0)
                if edge_data is None:
                    continue
                edge_length = edge_data.get('length', 1)
                new_path_length = path_length + edge_length
                queue.append((neighbor, path + [neighbor], new_path_length))

    end_time = time.time()
    tracemalloc.stop()
    return None, None, visited_nodes, end_time - start_time, None

def bidirectional_a_star(graph, start, goal):
    tracemalloc.start()
    start_time = time.time()

    graph_reversed = graph.reverse(copy=False)
    open_set_forward = [(0, start, [start], 0)]
    visited_forward = {}
    visited_nodes_forward = 0

    open_set_backward = [(0, goal, [goal], 0)]
    visited_backward = {}
    visited_nodes_backward = 0

    best_path = None
    best_cost = float('inf')

    while open_set_forward and open_set_backward:
        # Forward
        current_f_forward, current_node_forward, path_forward, current_g_forward = heapq.heappop(open_set_forward)
        if current_node_forward in visited_forward:
            continue
        visited_forward[current_node_forward] = (current_g_forward, path_forward)
        visited_nodes_forward += 1

        if current_node_forward in visited_backward:
            total_cost = current_g_forward + visited_backward[current_node_forward][0]
            if total_cost < best_cost:
                best_cost = total_cost
                path_backward = visited_backward[current_node_forward][1]
                best_path = path_forward + path_backward[::-1][1:]
                break

        for neighbor in graph.neighbors(current_node_forward):
            if neighbor in visited_forward:
                continue
            edge_data = graph.get_edge_data(current_node_forward, neighbor, 0)
            if edge_data is None:
                continue
            edge_length = edge_data.get('length', 1)
            tentative_g = current_g_forward + edge_length
            heuristic = haversine_distance(graph, neighbor, goal)
            tentative_f = tentative_g + heuristic
            heapq.heappush(open_set_forward, (tentative_f, neighbor, path_forward + [neighbor], tentative_g))

        # Backward
        current_f_backward, current_node_backward, path_backward, current_g_backward = heapq.heappop(open_set_backward)
        if current_node_backward in visited_backward:
            continue
        visited_backward[current_node_backward] = (current_g_backward, path_backward)
        visited_nodes_backward += 1

        if current_node_backward in visited_forward:
            total_cost = current_g_backward + visited_forward[current_node_backward][0]
            if total_cost < best_cost:
                best_cost = total_cost
                path_forward = visited_forward[current_node_backward][1]
                best_path = path_forward + path_backward[::-1][1:]
                break

        for neighbor in graph_reversed.neighbors(current_node_backward):
            if neighbor in visited_backward:
                continue
            edge_data = graph_reversed.get_edge_data(current_node_backward, neighbor, 0)
            if edge_data is None:
                continue
            edge_length = edge_data.get('length', 1)
            tentative_g = current_g_backward + edge_length
            heuristic = haversine_distance(graph_reversed, neighbor, start)
            tentative_f = tentative_g + heuristic
            heapq.heappush(open_set_backward, (tentative_f, neighbor, path_backward + [neighbor], tentative_g))

    end_time = time.time()
    total_visited_nodes = visited_nodes_forward + visited_nodes_backward
    current_mem = tracemalloc.get_traced_memory()[1] / 10**6
    tracemalloc.stop()

    if best_path:
        return best_path, best_cost, total_visited_nodes, end_time - start_time, current_mem
    else:
        return None, None, total_visited_nodes, end_time - start_time, current_mem

def a_star_alt(graph, start, goal, distance_to_landmark, distance_from_landmark, landmarks):
    tracemalloc.start()
    start_time = time.time()
    open_set = [(0, start, [start], 0)]
    visited = set()
    visited_nodes = 0

    while open_set:
        current_f, current_node, path, current_g = heapq.heappop(open_set)
        if current_node == goal:
            end_time = time.time()
            current_mem = tracemalloc.get_traced_memory()[1] / 10**6
            tracemalloc.stop()
            return path, current_g, visited_nodes, end_time - start_time, current_mem

        if current_node in visited:
            continue
        visited.add(current_node)
        visited_nodes += 1

        for neighbor in graph.neighbors(current_node):
            if neighbor in visited:
                continue
            edge_data = graph.get_edge_data(current_node, neighbor, 0)
            if edge_data is None:
                continue
            edge_length = edge_data.get('length', 1)
            tentative_g = current_g + edge_length
            heuristic = landmark_heuristic(neighbor, goal, distance_to_landmark, distance_from_landmark, landmarks)
            tentative_f = tentative_g + heuristic
            heapq.heappush(open_set, (tentative_f, neighbor, path + [neighbor], tentative_g))

    end_time = time.time()
    tracemalloc.stop()
    return None, None, visited_nodes, end_time - start_time, None

def a_star_backtracking(graph, start, goal, current_time=None, avoid_highways=True, avoid_congested_areas=True):
    # A* Backtracking WITH constraints
    tracemalloc.start()
    start_time = time.time()
    open_set = [(0, start, [start], 0)]
    visited = set()
    visited_nodes = 0

    while open_set:
        current_f, current_node, path, current_g = heapq.heappop(open_set)
        if current_node == goal:
            end_time = time.time()
            current_mem = tracemalloc.get_traced_memory()[1] / 10**6
            tracemalloc.stop()
            return path, current_g, visited_nodes, end_time - start_time, current_mem

        if current_node in visited:
            continue
        visited.add(current_node)
        visited_nodes += 1

        for neighbor in graph.neighbors(current_node):
            if neighbor in visited:
                continue
            edge_data = graph.get_edge_data(current_node, neighbor, 0)
            if not edge_data:
                continue
            road_id = edge_data.get('name', '')
            if isinstance(road_id, list):
                road_id = ' '.join(str(r) for r in road_id)

            if not is_road_safe(road_id, current_time=current_time, avoid_highways=avoid_highways,
                                avoid_congested_areas=avoid_congested_areas):
                continue

            edge_length = edge_data.get('length', 1)
            tentative_g = current_g + edge_length
            heuristic = haversine_distance(graph, neighbor, goal)
            tentative_f = tentative_g + heuristic
            heapq.heappush(open_set, (tentative_f, neighbor, path + [neighbor], tentative_g))

    end_time = time.time()
    tracemalloc.stop()
    return None, None, visited_nodes, end_time - start_time, None

def a_star_backtracking_with_waypoint(graph, start_node, end_node, waypoint_road="roosevelt drive"):
    # Find candidates for the waypoint
    waypoint_candidates = set()
    for u, v, key, data in graph.edges(keys=True, data=True):
        road_name = data.get('name', '')
        if isinstance(road_name, list):
            road_name = ' '.join(str(r) for r in road_name)
        if waypoint_road in road_name.lower():
            waypoint_candidates.add(u)
            waypoint_candidates.add(v)

    if not waypoint_candidates:
        print(f"No edges found containing '{waypoint_road}'. Cannot enforce waypoint constraint.")
        return None, None, None, None, None

    final_path = None
    final_cost = None
    final_visited = None
    final_exec_time = None
    final_mem = None

    for candidate in waypoint_candidates:
        path_to_waypoint, cost_to_waypoint, visited_to_waypoint, exec_time_to_waypoint, mem_waypoint = (
            a_star_backtracking(graph, start_node, candidate, current_time=current_time, avoid_highways=True,
                                avoid_congested_areas=True))

        if path_to_waypoint is None:
            continue

        path_from_waypoint, cost_from_waypoint, visited_from_waypoint, exec_time_from_waypoint, mem_end = (
            a_star_backtracking(graph, candidate, end_node, current_time=current_time, avoid_highways=True,
                                avoid_congested_areas=True))

        if path_from_waypoint is None:
            continue

        combined_path = path_to_waypoint + path_from_waypoint[1:]
        combined_cost = cost_to_waypoint + cost_from_waypoint
        combined_visited = visited_to_waypoint + visited_from_waypoint
        combined_exec_time = exec_time_to_waypoint + exec_time_from_waypoint
        combined_mem = max(mem_waypoint, mem_end) if (mem_waypoint and mem_end) else (mem_waypoint or mem_end)

        # Verify passing through waypoint road
        passes_through = False
        for u, v in zip(combined_path[:-1], combined_path[1:]):
            edge_data = graph.get_edge_data(u, v, 0)
            if edge_data:
                rname = edge_data.get('name', '')
                if isinstance(rname, list):
                    rname = ' '.join(str(r) for r in rname)
                if waypoint_road in rname.lower():
                    passes_through = True
                    break

        if passes_through:
            print(f"Path found passing through '{waypoint_road}'!")
            final_path = combined_path
            final_cost = combined_cost
            final_visited = combined_visited
            final_exec_time = combined_exec_time
            final_mem = combined_mem
            break

    return final_path, final_cost, final_visited, final_exec_time, final_mem

