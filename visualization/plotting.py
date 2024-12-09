import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import osmnx as ox
from constraints import get_constrained_edges
from config import restricted_roads, highways_to_avoid, congested_areas

def visualize_paths(graph, results, start_node, end_node):
    colors = {
        'A*': 'green',
        'Dijkstra': 'blue',
        'BFS': 'orange',
        'Bidirectional A*': 'purple',
        'A* with Landmarks': 'red',
        'A* Backtracking': 'black'
    }

    origin_x = graph.nodes[start_node]['x']
    origin_y = graph.nodes[start_node]['y']
    dest_x = graph.nodes[end_node]['x']
    dest_y = graph.nodes[end_node]['y']

    constrained_edges = get_constrained_edges(graph, restricted_roads, highways_to_avoid, congested_areas)
    constrained_edge_coords = []
    for u, v, key in constrained_edges:
        if 'geometry' in graph.edges[u, v, key]:
            geom = graph.edges[u, v, key]['geometry']
            xs, ys = zip(*list(geom.coords))
            constrained_edge_coords.append((xs, ys))
        else:
            x_u, y_u = graph.nodes[u]['x'], graph.nodes[u]['y']
            x_v, y_v = graph.nodes[v]['x'], graph.nodes[v]['y']
            constrained_edge_coords.append(([x_u, x_v], [y_u, y_v]))

    for algo, data in results.items():
        path = data['Path']
        if path:
            fig, ax = ox.plot_graph(graph, show=False, close=False, node_size=0,
                                    edge_color='lightgray', bgcolor='white', figsize=(15, 15))

            if constrained_edge_coords and algo == 'A* Backtracking':
                for (x_pair, y_pair) in constrained_edge_coords:
                    ax.plot(x_pair, y_pair, color='red', linewidth=2, alpha=0.7)

                for u, v in zip(path[:-1], path[1:]):
                    edge_data = graph.get_edge_data(u, v, 0)
                    if edge_data and 'geometry' in edge_data:
                        geom = edge_data['geometry']
                        xs, ys = zip(*list(geom.coords))
                    else:
                        x_u, y_u = graph.nodes[u]['x'], graph.nodes[u]['y']
                        x_v, y_v = graph.nodes[v]['x'], graph.nodes[v]['y']
                        xs, ys = [x_u, x_v], [y_u, y_v]
                    ax.plot(xs, ys, color=colors.get(algo, 'black'), linewidth=4, label=algo)
            else:
                route_nodes = [(graph.nodes[node]['x'], graph.nodes[node]['y']) for node in path]
                ax.plot([x for x, y in route_nodes], [y for x, y in route_nodes],
                        color=colors.get(algo, 'black'), linewidth=4, label=algo)

            # Start/End
            ax.scatter([origin_x, dest_x], [origin_y, dest_y],
                       c='red', s=100, marker='o', zorder=5, label='Start/End')

            legend_handles = [mpatches.Patch(color=colors.get(algo, 'black'), label=algo)]
            if algo == 'A* Backtracking':
                legend_handles.append(mpatches.Patch(color='red', label='Constrained Roads'))

            handles, labels = ax.get_legend_handles_labels()
            handles += legend_handles
            additional_labels = [algo] + (['Constrained Roads'] if algo == 'A* Backtracking' else [])
            labels += additional_labels
            by_label = dict(zip(labels, handles))
            ax.legend(by_label.values(), by_label.keys(), loc='best')

            ax.set_title(f'Path Visualization using {algo} Algorithm', fontsize=18)
            plt.show()
            plt.close(fig)

