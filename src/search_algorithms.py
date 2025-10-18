from collections import deque
from typing import Dict
from itertools import count
from src.utils import *
import heapq
import math

COST_PER_PIXEL = 10  # while creating the graph i scaled the cost 10 times the number of real pixel between loc


def bfs(graph: Graph, start: Location, goal: Location):
    queue = deque([start])
    visited = {start}
    came_from: Dict[Location, Location] = {}
    nodes_expanded = 0
    while queue:
        node = queue.popleft()
        nodes_expanded += 1
        if node == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, len(path) - 1 , nodes_expanded
        for nbr in graph.neighbors(node).keys():
            if nbr not in visited:
                visited.add(nbr)
                came_from[nbr] = node
                queue.append(nbr)
    return [], 0, nodes_expanded


def dfs(graph: Graph, start: Location, goal: Location):
    stack = [start]
    visited = {start}
    came_from: Dict[Location, Location] = {}
    nodes_expanded = 0
    while stack:
        node = stack.pop()
        nodes_expanded += 1
        if node == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, len(path) - 1, nodes_expanded
        for nbr in graph.neighbors(node).keys():
            if nbr not in visited:
                visited.add(nbr)
                came_from[nbr] = node
                stack.append(nbr)
    return [], 0, nodes_expanded


def dijkstra(graph: Graph, start: Location, goal: Location):
    dist = {start: 0.0}
    came_from = {}
    counter = count()  # <-- NEW
    pq = [(0.0, next(counter), start)]  # (priority, tiebreaker, node)
    visited = set()

    nodes_expanded = 0
    while pq:
        d, _, node = heapq.heappop(pq)  # unpack tiebreaker
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
                heapq.heappush(pq, (nd, next(counter), nbr))  # <-- use tiebreaker
    return [], math.inf, nodes_expanded


def heu_euclidean(u, v):
    cu, cv = u.coords(), v.coords()
    if cu is None or cv is None:
        return 0.0
    (x1, y1), (x2, y2) = cu, cv
    return COST_PER_PIXEL * math.hypot(x1 - x2, y1 - y2)


def astar(graph: Graph, start: Location, goal: Location, heuristic=None, ):
    # Default heuristic: straight-line in pixels scaled to cost units
    if heuristic is None:
        heuristic = heu_euclidean

    g_score = {start: 0.0}
    came_from = {}

    counter = count()
    f0 = g_score[start] + heuristic(start, goal)
    pq = [(f0, next(counter), start)]
    closed = set()

    nodes_expanded = 0
    while pq:
        _, __, node = heapq.heappop(pq)
        if node in closed:
            continue

        if node == goal:
            return reconstruct_path(came_from, start, goal), g_score[node], nodes_expanded
        closed.add(node)
        nodes_expanded += 1

        for nbr, w in graph.neighbors(node).items():
            tentative_g = g_score[node] + w
            if tentative_g < g_score.get(nbr, math.inf):
                came_from[nbr] = node
                g_score[nbr] = tentative_g
                f = tentative_g + heuristic(nbr, goal)
                heapq.heappush(pq, (f, next(counter), nbr))

    return [], math.inf, nodes_expanded
