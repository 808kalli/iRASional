import networkx as nx
import matplotlib.pyplot as plt
import math

def draw_path(G, graph, path):
    # Extracting x and y coordinates from the nodes, flipping the y-coordinate
    pos = {node: (G.nodes[node]['x'], -G.nodes[node]['y']) for node in G.nodes}

    # Drawing the graph
    nx.draw(G, pos, with_labels=True, node_size=200, node_color='skyblue', font_size=8)

    # Highlighting the path nodes based on the dashed attribute
    for node in path:
        color = 'darkblue' if graph[node].dashed else 'red'
        nx.draw_networkx_nodes(G, pos, nodelist=[str(node)], node_color=color, node_size=200)

    # Display the plot
    plt.show()

def print_path_info(G, graph, path):
    print("Path:")
    print(path)

    print("\nPath Attributes:")
    for element in path:
        next_nodes_str = ", ".join(map(str, graph[element].next))

        print("node id: {:<5} | x: {:<5} | y: {:<5} | next node(s): {:<10} | dashed: {}".format(
            element, graph[element].x, graph[element].y, next_nodes_str, graph[element].dashed
        ))

