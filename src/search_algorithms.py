import math
import heapq
from collections import deque
from itertools import count
from src.utils import *

COST_PER_PIXEL = 10  # scaled cost per pixel distance


def bfs(graph: Graph, start: Location, goal: Location):
    """
    A breath first search algorithm. Uses double ended queque (deque) list
    to implement a FIFO logic inside the algorithm

    Inputs:
    - graph: the graph to search
    - start: the starting location of the search
    - goal: the destination of the search

    Outputs:
    - path: the path from start to goal
    - number of jumps: the number of jumps from start to goal
    - explored nodes: the number of explored nodes
    """
    queue = deque([start])
    visited = {start}
    came_from = {}
    nodes_expanded = 0

    while queue:
        node = queue.popleft()
        nodes_expanded += 1

        if node == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, len(path) - 1, nodes_expanded

        for nbr in graph.neighbors(node):
            if nbr not in visited:
                visited.add(nbr)
                came_from[nbr] = node
                queue.append(nbr)

    return [], 0, nodes_expanded


def dfs(graph: Graph, start: Location, goal: Location):
    """
    A depth first search algorithm.

    Inputs:
    - graph: the graph to search
    - start: the starting location of the search
    - goal: the destination of the search

    Outputs:
    - path: the path from start to goal
    - number of jumps: the number of jumps from start to goal
    - explored nodes: the number of explored nodes
    """

    stack = [start]
    visited = {start}
    came_from = {}
    nodes_expanded = 0

    while stack:
        node = stack.pop()
        nodes_expanded += 1

        if node == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, len(path) - 1, nodes_expanded

        for nbr in graph.neighbors(node):
            if nbr not in visited:
                visited.add(nbr)
                came_from[nbr] = node
                stack.append(nbr)

    return [], 0, nodes_expanded


def dijkstra(graph: Graph, start: Location, goal: Location):
    """
    A Dijkstra search algorithm. Computes the shortest path between two nodes in a weighted graph.
    It uses the heapq library to implement a priority queue ensuring that the
    next node selected for expansion always has the smallest cost. It also uses itertools.count
    to implement tiebreakers in the heap.

    Inputs:
    - graph: the graph to search
    - start: the starting location of the search
    - goal: the destination of the search

    Outputs:
    - path: the path from start to goal
    - total cost of the journey: the cost of the journey
    - explored nodes: the number of explored nodes
    """
    dist = {start: 0.0}
    came_from = {}
    pq = []
    counter = count()
    heapq.heappush(pq, (0.0, next(counter), start))
    visited = set()
    nodes_expanded = 0

    while pq:
        d, _, node = heapq.heappop(pq)
        if node in visited:
            continue
        visited.add(node)
        nodes_expanded += 1

        if node == goal:
            return reconstruct_path(came_from, start, goal), d, nodes_expanded

        for nbr, w in graph.neighbors(node).items():
            nd = d + w
            if nd < dist.get(nbr, math.inf):
                dist[nbr] = nd
                came_from[nbr] = node
                heapq.heappush(pq, (nd, next(counter), nbr))

    return [], math.inf, nodes_expanded


def heu_euclidean(location1: Location, location2: Location):
    """
    A euclidean heuristic function that computes the euclidean distance between
    two locations of the graph.

    Inputs:
    - location1: the first location
    - location2: the second location

    Output:
    - distance: the euclidean distance
    """
    Location1_coordinates, Location2_coordinates = location1.coords(), location2.coords()
    if Location1_coordinates is None or Location2_coordinates is None:
        return 0.0
    (x1, y1), (x2, y2) = Location1_coordinates, Location2_coordinates
    euclidean_distance = math.sqrt(pow((x1 - x2), 2) + pow((x1 - x2), 2))
    return COST_PER_PIXEL * euclidean_distance


def astar(graph: Graph, start: Location, goal: Location, heuristic=None):
    """
    A A* search algorithm. Finds the optimal path by combining Dijkstra’s cost-based search with
    a heuristic estimate of the remaining distance.
    It uses the heapq library to implement a priority queue ensuring that the
    next node selected for expansion always has the smallest cost. It also uses itertools.count
    to implement tiebreakers in the heap.

    Inputs:
    - graph: the graph to search
    - start: the starting location of the search
    - goal: the destination of the search
    - heuristic: the heuristic function to use

    Outputs:
    - path: the path from start to goal
    - total cost of the journey: the cost of the journey
    - explored nodes: the number of explored nodes
    """
    if heuristic is None:
        heuristic = heu_euclidean

    g_score = {start: 0.0}
    came_from = {}
    pq = []
    counter = count()
    heapq.heappush(pq, (heuristic(start, goal), next(counter), start))
    closed = set()
    nodes_expanded = 0

    while pq:
        _, __, node = heapq.heappop(pq)
        if node in closed:
            continue
        closed.add(node)
        nodes_expanded += 1

        if node == goal:
            return reconstruct_path(came_from, start, goal), g_score[node], nodes_expanded

        for nbr, w in graph.neighbors(node).items():
            tentative_g = g_score[node] + w
            if tentative_g < g_score.get(nbr, math.inf):
                came_from[nbr] = node
                g_score[nbr] = tentative_g
                f = tentative_g + heuristic(nbr, goal)
                heapq.heappush(pq, (f, next(counter), nbr))

    return [], math.inf, nodes_expanded
