from src.search_algorithms import *


def node_expansion_performances_computation(cities: list, graph: Graph):
    """
    Computes node expansion performance (mean and standard deviation)
    for each possible transition between map locations, across multiple search algorithms.

    For every pair of distinct cities in the given map, the function runs four search algorithms
    (A*, Dijkstra, Breadth-First Search, and Depth-First Search), measuring how many nodes
    each algorithm expands to reach the goal. The results are grouped by the optimal path
    depth, and statistical summaries (mean and standard deviation) are computed for each algorithm.

    Inputs:
    - cities: List of city names (nodes) in the map.
    - map: The graph object representing the map, containing nodes and connections.

    Outputs:
    - risultati_a: Mean and standard deviation of node expansions by A*, grouped by optimal path depth.
    - risultati_d: Mean and standard deviation of node expansions by Dijkstra, grouped by optimal path depth.
    - risultati_bf: Mean and standard deviation of node expansions by Breadth-First Search, grouped by optimal path depth.
    - risultati_df: Mean and standard deviation of node expansions by Depth-First Search, grouped by optimal path depth.
    """
    performances = []
    for location in cities:
        for location2 in cities:
            if location != location2:
                start = graph.get(location)
                goal = graph.get(location2)
                path_a, _, nodes_a, _ = astar(graph, start, goal)
                _, _, nodes_d, _ = dijkstra(graph, start, goal)
                _, _, nodes_bf, _ = bfs(graph, start, goal)
                _, _, nodes_df, _ = dfs(graph, start, goal)
                performances.append({"profondità": len(path_a),
                                     "A*": nodes_a,
                                     "Dijkstra": nodes_d,
                                     "BFS": nodes_bf,
                                     "DFS": nodes_df})
    x_axis = {}
    for record in performances:
        depth = record["profondità"]
        if depth not in x_axis:
            x_axis[depth] = {"A*": [], "Dijkstra": [], "BFS": [], "DFS": []}
        for algo in ["A*", "Dijkstra", "BFS", "DFS"]:
            x_axis[depth][algo].append(record[algo])

    risultati_a = {}
    risultati_d = {}
    risultati_bf = {}
    risultati_df = {}

    for depth, data in x_axis.items():
        risultati_a[depth] = [mean(data["A*"]), standard_deviation(data["A*"])]
        risultati_d[depth] = [mean(data["Dijkstra"]), standard_deviation(data["Dijkstra"])]
        risultati_bf[depth] = [mean(data["BFS"]), standard_deviation(data["BFS"])]
        risultati_df[depth] = [mean(data["DFS"]), standard_deviation(data["DFS"])]

    return risultati_a, risultati_d, risultati_bf, risultati_df


def plot_performances(risultati_a, risultati_d, risultati_bf, risultati_df):
    """
    Plots the evolution of the mean and standard deviation of expanded nodes
    as a function of the optimal path depth for each search algorithm.

    Each curve represents the mean number of expanded nodes, while the error bars
    indicate the standard deviation across multiple runs for that solution depth.
    The x-axis corresponds to the optimal solution depth as determined by the A* algorithm.

    Inputs:
    - risultati_a : Mean and standard deviation by optimal path depth for the A* algorithm.
    - risultati_d : Mean and standard deviation by optimal path depth for the Dijkstra algorithm.
    - risultati_bf: Mean and standard deviation by optimal path depth for the Breadth-First Search algorithm.
    - risultati_df: Mean and standard deviation by optimal path depth for the Depth-First Search algorithm.

    """
    depths = sorted(risultati_a.keys())

    mean_a = [risultati_a[d][0] for d in depths]
    std_a = [risultati_a[d][1] for d in depths]

    mean_d = [risultati_d[d][0] for d in depths]
    std_d = [risultati_d[d][1] for d in depths]

    mean_bf = [risultati_bf[d][0] for d in depths]
    std_bf = [risultati_bf[d][1] for d in depths]

    mean_df = [risultati_df[d][0] for d in depths]
    std_df = [risultati_df[d][1] for d in depths]

    plt.figure(figsize=(10, 10))

    plt.errorbar(depths, mean_a, yerr=std_a, label='A*', color='blue', marker='o', capsize=5)
    plt.errorbar(depths, mean_d, yerr=std_d, label='Dijkstra', color='green', marker='o', capsize=5)
    plt.errorbar(depths, mean_bf, yerr=std_bf, label='BFS', color='orange', marker='o', capsize=5)
    plt.errorbar(depths, mean_df, yerr=std_df, label='DFS', color='red', marker='o', capsize=5)

    plt.xlabel('Solution Depth (Optimal Path Length)')
    plt.ylabel('Expanded Nodes')
    plt.title('Mean and Standard Deviation of Expanded Nodes vs Solution Depth',
              fontsize=14, fontweight='bold')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.show()
