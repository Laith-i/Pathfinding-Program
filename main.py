import osmnx as ox
import pandas as pd
from config import start_point, end_point, current_time
from visualization.compare import compare_algorithms, plot_comparison
from visualization.plotting import visualize_paths
from pathfinding.algorithms import a_star_backtracking_with_waypoint

if __name__ == "__main__":
    # Load the graph
    G = ox.graph_from_point(center_point=start_point, dist=20000, network_type='drive', 
                            retain_all=True, simplify=True)
    start_node = ox.distance.nearest_nodes(G, X=start_point[1], Y=start_point[0])
    end_node = ox.distance.nearest_nodes(G, X=end_point[1], Y=end_point[0])

    print(f"Graph statistics: Nodes={len(G.nodes)}, Edges={len(G.edges)}")
    print(f"Start Node ID: {start_node}")
    print(f"End Node ID: {end_node}")

    # Compare standard algorithms
    results = compare_algorithms(G, start_node, end_node)

    # Run A* Backtracking with waypoint
    final_path, final_cost, final_visited, final_exec_time, final_mem = a_star_backtracking_with_waypoint(
        G, start_node, end_node, waypoint_road="roosevelt drive"
    )

    if final_path is not None:
        results['A* Backtracking'] = {
            'Path': final_path,
            'Visited Nodes': final_visited,
            'Path Length (meters)': final_cost,
            'Execution Time (seconds)': final_exec_time,
            'Memory Consumption (MB)': final_mem,
            'Optimal': True
        }
    else:
        print("No valid path found that passes through 'roosevelt drive' under the given constraints.")

    df_results = pd.DataFrame(results).T
    print("\nAlgorithm Performance Comparison:")
    print(df_results[['Visited Nodes', 'Path Length (meters)', 'Execution Time (seconds)', 'Memory Consumption (MB)',
                      'Optimal']].to_string())

    visualize_paths(G, results, start_node, end_node)
    plot_comparison(df_results)

