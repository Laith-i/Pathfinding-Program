import matplotlib.pyplot as plt
import pandas as pd
from pathfinding.landmarks import select_landmarks, precompute_landmark_distances
from pathfinding.algorithms import a_star, dijkstra, bfs, bidirectional_a_star, a_star_alt

def compare_algorithms(graph, start, end):
    results = {}
    # Precompute landmarks for A* ALT
    num_landmarks = 30
    landmarks = select_landmarks(graph, num_landmarks)
    distance_to_landmark, distance_from_landmark = precompute_landmark_distances(graph, landmarks)

    algorithms = {
        'A*': a_star,
        'Dijkstra': dijkstra,
        'BFS': bfs,
        'Bidirectional A*': bidirectional_a_star,
        'A* with Landmarks': lambda g, s, t: a_star_alt(g, s, t, distance_to_landmark, distance_from_landmark, landmarks)
    }

    for algo_name, algo_func in algorithms.items():
        print(f"\nRunning {algo_name} algorithm...")
        path, cost, visited_nodes, exec_time_alg, mem_consumed = algo_func(graph, start, end)
        if path is not None:
            print(f"{algo_name} execution time: {exec_time_alg:.6f} seconds")
            print(f"{algo_name} visited nodes: {visited_nodes}")
            print(f"{algo_name} path length: {cost:.2f} meters")
            if mem_consumed:
                print(f"{algo_name} memory consumed: {mem_consumed:.2f} MB")
            else:
                print(f"{algo_name} memory consumed: N/A")
        else:
            print(f"{algo_name} failed to find a path.")
            print(f"{algo_name} visited nodes: {visited_nodes}")
            print(f"{algo_name} path length: N/A")
            print(f"{algo_name} memory consumed: N/A")

        is_optimal = (algo_name in ['A*', 'Dijkstra', 'Bidirectional A*', 'A* with Landmarks'])
        results[algo_name] = {
            'Path': path,
            'Visited Nodes': visited_nodes,
            'Path Length (meters)': cost,
            'Execution Time (seconds)': exec_time_alg,
            'Memory Consumption (MB)': mem_consumed,
            'Optimal': is_optimal
        }

    return results

def plot_comparison(df_results):
    color_mapping = {
        'A*': 'green',
        'Dijkstra': 'blue',
        'BFS': 'orange',
        'Bidirectional A*': 'purple',
        'A* with Landmarks': 'red',
        'A* Backtracking': 'black'
    }

    algorithms = df_results.index
    nodes_visited = df_results['Visited Nodes']
    path_lengths = df_results['Path Length (meters)']
    execution_times = df_results['Execution Time (seconds)']
    bar_colors = [color_mapping.get(algo, 'gray') for algo in algorithms]

    # Plot 1: Nodes Visited
    plt.figure(figsize=(10, 6))
    plt.bar(algorithms, nodes_visited, color=bar_colors)
    plt.title('Nodes Visited by Pathfinding Algorithms')
    plt.xlabel('Algorithm')
    plt.ylabel('Number of Nodes Visited')
    plt.tight_layout()
    plt.show()

    # Plot 2: Path Lengths
    plt.figure(figsize=(10, 6))
    plt.bar(algorithms, path_lengths, color=bar_colors)
    plt.title('Path Lengths Found by Pathfinding Algorithms')
    plt.xlabel('Algorithm')
    plt.ylabel('Path Length (meters)')
    plt.tight_layout()
    plt.show()

    # Plot 3: Execution Time
    plt.figure(figsize=(10, 6))
    plt.bar(algorithms, execution_times, color=bar_colors)
    plt.title('Execution Time of Pathfinding Algorithms')
    plt.xlabel('Algorithm')
    plt.ylabel('Execution Time (seconds)')
    plt.tight_layout()
    plt.show()

