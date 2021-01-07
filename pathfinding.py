# @author: S. Alex Yun
# A collection of pathfinding algorithms.
#
# Adapted from: https://www.redblobgames.com/pathfinding/a-star/
# - Copyright 2014 Red Blob Games <redblobgames@gmail.com>
# - License: Apache v2.0 <https://www.apache.org/licenses/LICENSE-2.0>

import heapq
import math

from collections import defaultdict, deque


class Graph:
    def __init__(self):
        self.edges = {}
    
    def __repr__(self):
        return str(self.edges)

    def neighbours(self, id):
        return self.edges[id]


class WeightedGraph(Graph):
    def cost(self, from_id, to_id):
        pass


class Gridworld:
    """A two-dimensional (m x n) lattice graph."""
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []

    def __repr__(self):
        from pprint import pformat
        board = [[0 for y in range(self.height)] for x in range(self.width)]
        if self.obstacles:
            for obstacle in self.obstacles:
                board[obstacle[1]][obstacle[0]] = 1
        return pformat(board, indent=4)

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.obstacles

    def neighbours(self, id):
        (x, y) = id
        neighbours = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # E W N S
        if (x + y) % 2 == 0: neighbours.reverse() # S N W E
        results = filter(self.in_bounds, neighbours)
        results = filter(self.passable, results)
        return results


class WeightedGridworld(Gridworld):
    """An extension of Gridworld with numeric weights."""
    def __init__(self, width, height):
        super().__init__(width,height)
        self.weights = {}
    
    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)


def from_id_width(id, width):
    return (id % width, id // width)


def draw_tile(graph, id, style):
    """
    Returns an appropriate position marker in the Gridworld, denoting
    the source node, goal node, obstacles, and/or the path the algorithm found.
    """
    r = " . "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'path' in style and id in style['path']:     r = " @ "
    if 'source' in style and id == style['source']: r = " S "
    if 'goal' in style and id == style['goal']:     r = " G "
    if id in graph.obstacles: r = "###"
    return r


def draw_grid(graph, **style):
    """
    Outputs the Gridworld with the source node, goal node, obstacles, cost,
    and/or the path the algorithm found.
    """
    print("~~~" * graph.width)
    for y in range(graph.height):
        for x in range(graph.width):
            print("%s" % draw_tile(graph, (x, y), style), end="")
        print()
    print("~~~" * graph.width)


def reconstruct_path(came_from, source, goal):
    """Returns the path that the algorithm traversed in the Gridworld."""
    current = goal
    path = []
    while current != source:
        path.append(current)
        current = came_from[current]
    path.append(source)
    path.reverse()
    return path


def bfs(graph, source_node, goal_node):
    """Breath-first search."""
    path = {source_node: None}
    frontier_queue = deque([source_node])
    while frontier_queue:
        cur_node = frontier_queue.popleft()

        # Early exit
        if cur_node == goal_node:
            print("Successfully found a path from %s to %s via BFS!" % (source_node, goal_node))
            break

        for next_node in graph.neighbours(cur_node):
            if next_node not in path:
                path[next_node] = cur_node
                frontier_queue.append(next_node)

    return path


def dfs(graph, source_node, goal_node):
    """Depth-first search (iterative version)."""
    path = set()
    frontier_stack = deque([source_node])
    parents = {source_node: None}
    while frontier_stack:
        cur_node = frontier_stack.pop()

        if cur_node == goal_node:
            print("Successfully found a path from %s to %s via DFS!" % (source_node, goal_node))
            break

        if cur_node not in path:
            path.add(cur_node)
            for next_node in graph.neighbours(cur_node):
                if next_node not in path: # cycle check
                    frontier_stack.append(next_node)
                    parents[next_node] = cur_node

    return parents


def dijkstra(graph, source_node, goal_node):
    """Also known as the uniform cost search."""
    path = {source_node: None}
    cost = defaultdict(lambda: math.inf)
    cost[source_node] = 0
    frontier_pq = [(0, source_node)] # min_heap
    while frontier_pq:
        cur_cost, cur_node = heapq.heappop(frontier_pq)

        if cur_node == goal_node:
            print("Succesfully found a path from %s to %s via Dijkstra!" % (source_node, goal_node))
            break
        
        for next_node in graph.neighbours(cur_node):
            alt_cost = cur_cost + graph.cost(cur_node, next_node)
            if alt_cost < cost[next_node]:
                path[next_node] = cur_node
                cost[next_node] = alt_cost
                heapq.heappush(frontier_pq, (alt_cost, next_node))
    
    return path, cost


def heuristic(cur_node, goal_node):
    """
    An utility function used in a_star.
    It returns the estimated cost of the cheapest path
    from the current node to the goal node.
    """
    (x1, y1) = cur_node
    (x2, y2) = goal_node
    return abs(x1 - x2) + abs(y1 - y2)


def a_star(graph, source_node, goal_node):
    """
    A* search is a type of best-first search and
    can be viewed as an extension of Dijkstra's algorithm."""
    path = {source_node: None}
    cost = defaultdict(lambda: math.inf)
    cost[source_node] = 0
    frontier_pq = [(0, source_node)]
    while frontier_pq:
        _, cur_node = heapq.heappop(frontier_pq)

        if cur_node == goal_node:
            print("Succesfully found a path from %s to %s via A*!" % (source_node, goal_node))
            break
        
        for next_node in graph.neighbours(cur_node):
            alt_cost = cost[cur_node] + graph.cost(cur_node, next_node)
            if alt_cost < cost[next_node]:
                path[next_node] = cur_node
                cost[next_node] = alt_cost
                priority = alt_cost + heuristic(next_node, goal_node)
                heapq.heappush(frontier_pq, (priority, next_node))
    
    return path, cost


def main():
    simple_graph = Graph()
    simple_graph.edges = {
        'A': ['B'],
        'B': ['C'],
        'C': ['B', 'D', 'F'],
        'D': ['C', 'E'],
        'E': ['F'],
        'F': [],
    }
    start, end = 'A', 'E'

    came_from_bfs = bfs(simple_graph, start, end)
    print("The traversed nodes are:", reconstruct_path(came_from_bfs, start, end))

    OBSTACLES = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]
    simple_gridworld = Gridworld(30, 15)
    simple_gridworld.obstacles = OBSTACLES
    start, end = (8, 7), (27, 2)

    came_from_bfs = bfs(simple_gridworld, start, end)
    draw_grid(simple_gridworld, path=reconstruct_path(came_from_bfs, start, end),
        point_to=came_from_bfs, source=start, goal=end)

    weighted_gridworld = WeightedGridworld(10, 10)
    weighted_gridworld.obstacles = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
    weighted_gridworld.weights = {
        loc: 5 for loc in [(3, 4), (3, 5),
        (4, 1), (4, 2), (4, 3),(4, 4), (4, 5), (4, 6), (4, 7), (4, 8),
        (5, 1), (5, 2), (5, 3), (5, 4), (5, 5), (5, 6), (5, 7), (5, 8),
        (6, 2), (6, 3), (6, 4), (6, 5), (6, 6), (6, 7),
        (7, 3), (7, 4), (7, 5)]
    }
    start, end = (1, 4), (8, 3)
    
    came_from_dijkstra, cost_dijkstra = dijkstra(weighted_gridworld, start, end)
    draw_grid(weighted_gridworld, point_to=came_from_dijkstra, source=start, goal=end)
    draw_grid(weighted_gridworld, path=reconstruct_path(came_from_dijkstra, start, end))
    draw_grid(weighted_gridworld, number=cost_dijkstra, source=start, goal=end)

    came_from_a_star, cost_a_star = a_star(weighted_gridworld, start, end)
    draw_grid(weighted_gridworld, point_to=came_from_a_star, source=start, goal=end)
    draw_grid(weighted_gridworld, path=reconstruct_path(came_from_a_star, start, end))
    draw_grid(weighted_gridworld, number=cost_a_star, source=start, goal=end)


if __name__ == "__main__":
    main()