
"""
Single-file implementation of:
- Graph class
- Dijkstra's shortest path
- A* search
- Random graph generation
- Benchmark comparison
"""

import heapq
import random
import math
import time

class Graph:
    def __init__(self):
        self.adj = {}
        self.coords = {}

    def add_node(self, node, x=None, y=None):
        if node not in self.adj:
            self.adj[node] = []
        if x is not None and y is not None:
            self.coords[node] = (x, y)

    def add_edge(self, u, v, w):
        self.adj[u].append((v, w))

    def neighbors(self, node):
        return self.adj.get(node, [])

    def distance(self, a, b):
        ax, ay = self.coords[a]
        bx, by = self.coords[b]
        return math.sqrt((ax - bx)**2 + (ay - by)**2)

def dijkstra(graph, start, goal):
    pq = [(0, start)]
    dist = {start: 0}
    visited = set()
    nodes_visited = 0

    while pq:
        d, node = heapq.heappop(pq)
        if node in visited:
            continue
        visited.add(node)
        nodes_visited += 1

        if node == goal:
            return d, nodes_visited

        for neigh, w in graph.neighbors(node):
            nd = d + w
            if nd < dist.get(neigh, float('inf')):
                dist[neigh] = nd
                heapq.heappush(pq, (nd, neigh))

    return float('inf'), nodes_visited

def astar(graph, start, goal):
    pq = [(0, start)]
    g = {start: 0}
    visited = set()
    nodes_visited = 0

    while pq:
        f, node = heapq.heappop(pq)
        if node in visited:
            continue
        visited.add(node)
        nodes_visited += 1

        if node == goal:
            return g[node], nodes_visited

        for neigh, w in graph.neighbors(node):
            tentative_g = g[node] + w
            if tentative_g < g.get(neigh, float('inf')):
                g[neigh] = tentative_g
                h = graph.distance(neigh, goal)
                f = tentative_g + h
                heapq.heappush(pq, (f, neigh))

    return float('inf'), nodes_visited

def generate_random_graph(n=100, e=500):
    g = Graph()

    for i in range(n):
        g.add_node(i, random.random()*100, random.random()*100)

    for _ in range(e):
        u = random.randint(0, n-1)
        v = random.randint(0, n-1)
        if u != v:
            w = random.uniform(1, 10)
            g.add_edge(u, v, w)

    return g

def benchmark():
    g = generate_random_graph()
    start, goal = 0, 50

    t1 = time.time()
    dj_dist, dj_nodes = dijkstra(g, start, goal)
    t2 = time.time()

    t3 = time.time()
    as_dist, as_nodes = astar(g, start, goal)
    t4 = time.time()

    print("Dijkstra:")
    print(" Distance:", dj_dist)
    print(" Nodes Visited:", dj_nodes)
    print(" Time:", t2 - t1, "sec\n")

    print("A*:")
    print(" Distance:", as_dist)
    print(" Nodes Visited:", as_nodes)
    print(" Time:", t4 - t3, "sec\n")

if __name__ == "__main__":
    benchmark()
