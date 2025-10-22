from src.search_algorithms import *


def node_expansion_performances_computation(cities: list, map: Graph):
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
                start = map.get(location)
                goal = map.get(location2)
                path_a, _, nodes_a, _ = astar(map, start, goal)
                _, _, nodes_d, _ = dijkstra(map, start, goal)
                _, _, nodes_bf, _ = bfs(map, start, goal)
                _, _, nodes_df, _ = dfs(map, start, goal)
                performances.append({"profondità": len(path_a),
                                     "A*": nodes_a,
                                     "Dijkstra": nodes_d,
                                     "BFS": nodes_bf,
                                     "DFS": nodes_df})
    asse_x = {}
    for record in performances:
        profondita = record["profondità"]
        if profondita not in asse_x:
            asse_x[profondita] = {"A*": [], "Dijkstra": [], "BFS": [], "DFS": []}
        for algo in ["A*", "Dijkstra", "BFS", "DFS"]:
            asse_x[profondita][algo].append(record[algo])

    risultati_a = {}
    risultati_d = {}
    risultati_bf = {}
    risultati_df = {}

    for profondita, dati in asse_x.items():
        risultati_a[profondita] = [mean(dati["A*"]), standard_deviation(dati["A*"])]
        risultati_d[profondita] = [mean(dati["Dijkstra"]), standard_deviation(dati["Dijkstra"])]
        risultati_bf[profondita] = [mean(dati["BFS"]), standard_deviation(dati["BFS"])]
        risultati_df[profondita] = [mean(dati["DFS"]), standard_deviation(dati["DFS"])]

    return risultati_a, risultati_d, risultati_bf, risultati_df


def plot_performances(risultati_a, risultati_d, risultati_bf, risultati_df):
    """
    Plots the evolution of the mean and standard deviation of expanded nodes
    as a function of the optimal path depth, for each search algorithm.

    Each curve represents either the mean or standard deviation of the number
    of expanded nodes required to reach the goal, over the optimal
    solution depth found by A*.

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
    plt.plot(depths, mean_a, label='A* Mean', color='blue', marker='o')
    plt.plot(depths, mean_d, label='Dijkstra Mean', color='green', marker='o')
    plt.plot(depths, mean_bf, label='BFS Mean', color='orange', marker='o')
    plt.plot(depths, mean_df, label='DFS Mean', color='red', marker='o')

    plt.plot(depths, std_a, label='A* Std dev', color='blue', linestyle='--')
    plt.plot(depths, std_d, label='Dijkstra Std dev', color='green', linestyle='--')
    plt.plot(depths, std_bf, label='BFS Std dev', color='orange', linestyle='--')
    plt.plot(depths, std_df, label='DFS Std dev', color='red', linestyle='--')

    plt.xlabel('Solution Depth (Optimal Path Length)')
    plt.ylabel('Expanded Nodes')
    plt.title('Mean and Standard Deviation of Expanded Nodes vs Solution Depth', fontsize=14, fontweight='bold')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.show()
