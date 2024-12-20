import networkx as nx
import matplotlib.pyplot as plt
<<<<<<< HEAD
=======
import math
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010

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

<<<<<<< HEAD
=======

def find_position(G, gpsx, gpsy):
        minID = '0'
        minx = 0
        miny = 0
        min_distance = 88888888

        for i in G.nodes:
            distance = math.sqrt((pow(gpsx - G.nodes[i]['x'], 2 )+ pow(gpsy - G.nodes[i]['y'], 2)))
            if distance < min_distance:
                min_distance = distance
                minx = G.nodes[i]['x']
                miny = G.nodes[i]['y']
                minID = i

        return minID
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
