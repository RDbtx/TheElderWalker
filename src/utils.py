from __future__ import annotations
from typing import Dict, List, Tuple, Optional, Iterable
import math


class Location:
    def __init__(self, name: str, x: Optional[float] = None, y: Optional[float] = None):
        self.name = name
        self.x = x
        self.y = y

    def coords(self) -> Optional[Tuple[float, float]]:
        if self.x is not None and self.y is not None:
            return (self.x, self.y)
        return None


class Graph:

    def __init__(self) -> None:
        self.nodes = {}
        self.adj = {}

    def add_location(self, name: str, x: Optional[float] = None, y: Optional[float] = None) -> Location:
        if name in self.nodes:
            existing = (self.nodes[name])
            if (x is not None and existing.x != x) or (y is not None and existing.y != y):
                self.nodes[name] = Location(name, x, y)
                self.replace_node(existing, self.nodes[name])
            return self.nodes[name]
        loc = Location(name, x, y)
        self.nodes[name] = loc
        self.adj.setdefault(loc, {})
        return loc

    def replace_node(self, old: Location, new: Location) -> None:
        if old not in self.adj:
            self.adj[new] = {}
            return

        self.adj[new] = self.adj.pop(old)
        for n, nbrs in list(self.adj.items()):
            if old in nbrs:
                nbrs[new] = nbrs.pop(old)

    def get(self, name: str) -> Location:
        return self.nodes[name]

    def connect(self, a: str | Location, b: str | Location, distance: float, bidirectional: bool = True) -> None:
        if isinstance(a, str):
            a = self.add_location(a)
        if isinstance(b, str):
            b = self.add_location(b)
        self.adj.setdefault(a, {})[b] = float(distance)
        if bidirectional:
            self.adj.setdefault(b, {})[a] = float(distance)

    def neighbors(self, node: Location) -> Dict[Location, float]:
        return self.adj.get(node, {})

    def locations(self) -> Iterable[Location]:
        return self.nodes.values()


# -----------------------------
# Path reconstruction helper
# -----------------------------

def reconstruct_path(came_from: Dict[Location, Location], start: Location, goal: Location):
    path: List[Location] = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = came_from[cur]
    path.append(start)
    path.reverse()
    return path


import json
from pathlib import Path
from typing import Mapping, Any


def _parse_json_structure(obj: Mapping[str, Any]) -> tuple[dict[str, dict[str, float]], dict[str, tuple[float, float]]]:
    """Return (adjacency, coords) extracted from JSON object in any of the supported formats."""
    adjacency: dict[str, dict[str, float]] = {}
    coords: dict[str, tuple[float, float]] = {}

    # Case 2b: has explicit top-level keys
    if "graph" in obj or "coords" in obj:
        g = obj.get("graph", {})
        if not isinstance(g, Mapping):
            raise ValueError("'graph' must be an object mapping node -> neighbors")
        adjacency = {str(k): {str(n): float(w) for n, w in (v or {}).items()} for k, v in g.items()}
        c = obj.get("coords", {})
        if isinstance(c, Mapping):
            for name, xy in c.items():
                if isinstance(xy, (list, tuple)) and len(xy) == 2:
                    coords[str(name)] = (float(xy[0]), float(xy[1]))
        return adjacency, coords

    # Cases 1 or 2a: keyed by node names
    for name, data in obj.items():
        name = str(name)
        if isinstance(data, Mapping):
            # 2a: node object possibly contains x/y and neighbors
            if any(k in data for k in ("x", "y", "neighbors")):
                x = data.get("x")
                y = data.get("y")
                if x is not None and y is not None:
                    coords[name] = (float(x), float(y))
                nbrs = data.get("neighbors", {})
                if not isinstance(nbrs, Mapping):
                    raise ValueError(f"'neighbors' of {name} must be an object")
                adjacency[name] = {str(n): float(w) for n, w in nbrs.items()}
            else:
                # 1: pure adjacency
                adjacency[name] = {str(n): float(w) for n, w in data.items()}
        else:
            raise ValueError("Invalid JSON: node value must be an object")

    return adjacency, coords


def load_graph_from_json(path: str | Path, bidirectional: bool = True, strict_symmetry: bool = False) -> Graph:
    """
    Build a Graph from a JSON file.

    - bidirectional=True: treat edges as undirected roads. We'll add the reverse edge if missing.
    - strict_symmetry=True: if both directions are present with different weights, raise an error.
      If False, we'll keep the smaller weight and warn via a print.
    """
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        obj = json.load(f)

    adjacency, coords = _parse_json_structure(obj)

    g = Graph()
    # Create all nodes first (with coords if available)
    for name in adjacency.keys() | coords.keys():
        (x, y) = coords.get(name, (None, None))
        g.add_location(name, x=x, y=y)

    # Add edges
    for src, nbrs in adjacency.items():
        for dst, w in nbrs.items():
            if bidirectional:
                # If JSON already has both directions and weights differ, resolve
                rev_w = adjacency.get(dst, {}).get(src)
                if rev_w is not None and rev_w != w:
                    if strict_symmetry:
                        raise ValueError(f"Asymmetric weights for {src}<->{dst}: {w} vs {rev_w}")
                    chosen = min(float(w), float(rev_w))
                    print(f"[warn] Asymmetric weights {src}<->{dst}: {w} vs {rev_w}. Using {chosen}.")
                    w = chosen
                # To avoid double-connecting the same undirected edge, only connect when src<=dst if reverse exists.
                if rev_w is not None and dst < src:
                    continue
                g.connect(src, dst, float(w), bidirectional=True)
            else:
                g.connect(src, dst, float(w), bidirectional=False)

    return g


import matplotlib.pyplot as plt
from math import tau


def _compute_positions(g: Graph) -> Dict[Location, Tuple[float, float]]:
    """
    Return {node: (x,y)}. If some nodes miss coords, place them on a circle.
    Coordinates are used as-is; circle fallback is in the [0,1] square.
    """
    nodes = list(g.locations())
    # Check if we have at least one coord
    have_any_coords = any(n.coords() is not None for n in nodes)
    pos: Dict[Location, Tuple[float, float]] = {}
    if have_any_coords:
        # Normalize provided coords to [0,1] for display if they seem unbounded
        xs, ys = [], []
        for n in nodes:
            c = n.coords()
            if c is not None:
                xs.append(c[0]);
                ys.append(c[1])
        if xs and ys:
            xmin, xmax = min(xs), max(xs)
            ymin, ymax = min(ys), max(ys)
            dx = (xmax - xmin) or 1.0
            dy = (ymax - ymin) or 1.0
            for n in nodes:
                c = n.coords()
                if c is None:
                    pos[n] = (0.5, 0.5)  # temporary; will be overwritten by circle for missing
                else:
                    pos[n] = ((c[0] - xmin) / dx, (c[1] - ymin) / dy)
        # Place missing on a small circle around center
        missing = [n for n in nodes if n.coords() is None]
        m = len(missing)
        if m:
            for i, n in enumerate(missing):
                ang = (i / m) * tau
                pos[n] = (0.5 + 0.2 * math.cos(ang), 0.5 + 0.2 * math.sin(ang))
        return pos
    else:
        # All missing: place everyone on a circle
        N = len(nodes) or 1
        for i, n in enumerate(nodes):
            ang = (i / N) * tau
            pos[n] = (0.5 + 0.4 * math.cos(ang), 0.5 + 0.4 * math.sin(ang))
        return pos


def plot_graph(
        g: Graph,
        annotate_nodes: bool = True,
        annotate_edges: bool = True,
        scale_edge_width: bool = True,
        title: Optional[str] = None,
        figsize: Tuple[float, float] = (10, 8),
) -> None:
    """Plot the graph using node coordinates when available.

    Edge weights are treated as *costs* (labels); widths optionally scale with cost (inverse so cheaper is thicker).
    """
    pos = _compute_positions(g)

    # Collect edges and weights
    edges: List[Tuple[Location, Location, float]] = []
    for u in g.locations():
        for v, w in g.neighbors(u).items():
            # draw each undirected edge once (u.name < v.name)
            if u.name < v.name:
                edges.append((u, v, w))

    if not edges:
        print("[plot_graph] No edges to plot.")

    # Prepare figure
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_title(title or "Graph (costs as labels)")
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([]);
    ax.set_yticks([])

    # Node scatter
    xs = [pos[n][0] for n in pos]
    ys = [pos[n][1] for n in pos]
    ax.scatter(xs, ys, s=60, zorder=3)

    # Edge drawing
    if edges:
        costs = [w for (_, _, w) in edges]
        cmin, cmax = min(costs), max(costs)

        # Edge width inversely proportional to cost, within [1.0, 4.0]
        def width_for(w: float) -> float:
            if not scale_edge_width or cmax == cmin:
                return 2.0
            # normalize so low cost -> wide
            t = (w - cmin) / (cmax - cmin)
            return 4.0 - 3.0 * t

        for u, v, w in edges:
            x1, y1 = pos[u]
            x2, y2 = pos[v]
            ax.plot([x1, x2], [y1, y2], linewidth=width_for(w), alpha=0.8, zorder=1)

        if annotate_edges:
            for u, v, w in edges:
                x1, y1 = pos[u]
                x2, y2 = pos[v]
                mx, my = (x1 + x2) / 2, (y1 + y2) / 2
                ax.text(mx, my, f"{w:g}", fontsize=9, ha='center', va='center',
                        bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.7))

    # Node labels
    if annotate_nodes:
        for n, (x, y) in pos.items():
            ax.text(x, y + 0.02, n.name, fontsize=10, ha='center', va='bottom')

    plt.tight_layout()
    plt.show()


def load_graph_and_plot(json_path: str, bidirectional: bool = True, strict_symmetry: bool = False):
    g = load_graph_from_json(json_path, bidirectional=bidirectional, strict_symmetry=strict_symmetry)
    plot_graph(g,title="Skyrim",figsize=(20, 20))
    return g


def print_cities_names(json_file):
    with open(json_file, "r", encoding="utf-8") as f:
        data = json.load(f)

    names = []

    for elem in data.keys():
        names.append(elem)

    for name in sorted(names):
        print(name)

    return names
