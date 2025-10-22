from src.search_algorithms import *

map_location = "../map.json"

if __name__ == "__main__":
    map = load_graph_and_plot(map_location)

    print("\n---- THE ELDER WALKER ----")
    print("\nCities and locations of Skyrim:\n")
    cities = map.get_names()

    print("Total number of locations:", len(cities))
    working = True
    while working:

        menu_decisions = [0, 1, 2]
        print("\nWhat do you want to do?")
        print("1- Create your journey")
        print("2- Show Skyrim Map")
        print("0- Close the program")

        decision = 5
        while decision not in menu_decisions:
            decision = int(input("select your decision: [1/2/0]: "))

        print("\n")
        if decision == 1:
            starting_location = ""
            destination = ""
            while starting_location not in cities:
                starting_location = input("Enter your starting location: ")
            while destination not in cities:
                destination = input("Enter your goal location: ")

            start = map.get(starting_location)
            goal = map.get(destination)

            print("\nCalculating your journey...")
            path, cost, nodes_a, time_a = astar(map, start, goal)
            print("A*:", " -> ".join(n.name for n in path), f"| total cost: {cost:.2f} | Nodes expanded: {nodes_a} | processing time: {time_a}")
            path, cost, nodes_d, time_d = dijkstra(map, start, goal)
            print("DIJKSTRA:", " -> ".join(n.name for n in path),
                  f"| total cost: {cost:.2f} | Nodes expanded: {nodes_d} | processing time: {time_d}")
            path, cost, nodes_bf, time_bf = bfs(map, start, goal)
            print("BFS:", " -> ".join(n.name for n in path), f"| total jumps: {cost} | Nodes expanded: {nodes_bf} | processing time: {time_bf}")
            path, cost, nodes_df, time_df = dfs(map, start, goal)
            print("DFS:", " -> ".join(n.name for n in path), f"| total jumps: {cost} | Nodes expanded: {nodes_df} | processing time: {time_df}")

        elif decision == 2:
            print("\nCities and locations of Skyrim:")
            _ = map.get_names()
            print("Total number of locations:", len(cities))

        elif decision == 0:
            print("\nClosing the program...")
            working = False
