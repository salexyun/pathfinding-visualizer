# @author: S. Alex Yun
# A collection of pathfinding algorithms.
#
# Adapted from: https://www.redblobgames.com/pathfinding/a-star/
# - Copyright 2014 Red Blob Games <redblobgames@gmail.com>
# - License: Apache v2.0 <https://www.apache.org/licenses/LICENSE-2.0>

try:
    import heapq
    import math
    import sys
    from collections import defaultdict, deque
    import pygame as pg
    import pygame_gui
    from pygame.locals import *
except ImportError as err:
    print("Could not load module. %s" % (err))
    sys.exit(2)


class Gridworld:
    """A two-dimensional (m x n) lattice graph."""
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.source = None
        self.goal = None
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
        super().__init__(width, height)
        self.weights = {}
    
    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)


def bfs(graph, source_node, goal_node):
    """Breath-first search."""
    visited = {source_node: None}
    frontier_queue = deque([source_node])
    while frontier_queue:
        cur_node = frontier_queue.popleft()

        # Early exit
        if cur_node == goal_node:
            break

        for next_node in graph.neighbours(cur_node):
            if next_node not in visited:
                visited[next_node] = cur_node
                frontier_queue.append(next_node)

    return visited


def dfs(graph, source_node, goal_node):
    """Depth-first search (iterative version)."""
    visited = set()
    frontier_stack = deque([source_node])
    parents = {source_node: None}
    while frontier_stack:
        cur_node = frontier_stack.pop()

        if cur_node == goal_node:
            break

        if cur_node not in visited:
            visited.add(cur_node)
            for next_node in graph.neighbours(cur_node):
                if next_node not in visited: # cycle check
                    frontier_stack.append(next_node)
                    parents[next_node] = cur_node

    return parents


def dijkstra(graph, source_node, goal_node):
    """Also known as the uniform cost search."""
    visited = {source_node: None}
    cost = defaultdict(lambda: math.inf)
    cost[source_node] = 0
    frontier_pq = [(0, source_node)] # min_heap
    while frontier_pq:
        cur_cost, cur_node = heapq.heappop(frontier_pq)

        if cur_node == goal_node:
            break
        
        for next_node in graph.neighbours(cur_node):
            alt_cost = cur_cost + graph.cost(cur_node, next_node)
            if alt_cost < cost[next_node]:
                visited[next_node] = cur_node
                cost[next_node] = alt_cost
                heapq.heappush(frontier_pq, (alt_cost, next_node))
    
    return visited, cost


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
    visited = {source_node: None}
    cost = defaultdict(lambda: math.inf)
    cost[source_node] = 0
    frontier_pq = [(0, source_node)]
    while frontier_pq:
        _, cur_node = heapq.heappop(frontier_pq)

        if cur_node == goal_node:
            break
        
        for next_node in graph.neighbours(cur_node):
            alt_cost = cost[cur_node] + graph.cost(cur_node, next_node)
            if alt_cost < cost[next_node]:
                visited[next_node] = cur_node
                cost[next_node] = alt_cost
                priority = alt_cost + heuristic(next_node, goal_node)
                heapq.heappush(frontier_pq, (priority, next_node))
    
    return visited, cost


def reconstruct_path(visited, source, goal):
    """Returns the path that the algorithm traversed in the Gridworld."""
    current = goal
    path = []
    while current != source:
        path.append(current)
        current = visited[current]
    path.append(source)
    path.reverse()
    return path


WINDOW_SIZE = [800, 600]
GRID_SIZE = [500, 500]
SQUARE_SIZE = 20


def set_source(screen, gridworld):
    """Initialize the source node as a Rect object."""
    gridworld.source = (0, 0)
    source_rect = pg.Rect([gridworld.source[0]*SQUARE_SIZE, gridworld.source[1]*SQUARE_SIZE,
                            SQUARE_SIZE, SQUARE_SIZE])
    source_dragging = False
    return source_rect, source_dragging


def set_goal(screen, gridworld):
    """Initialize the goal node as a Rect object."""
    gridworld.goal = (gridworld.width-1, gridworld.height-1)
    goal_rect = pg.Rect([gridworld.goal[0]*SQUARE_SIZE, gridworld.goal[1]*SQUARE_SIZE,
                        SQUARE_SIZE, SQUARE_SIZE])
    goal_dragging = False
    return goal_rect, goal_dragging


def draw_grid(screen, gridworld, source_rect, goal_rect):
    """Draw the gridworld, source node, and goal node on the display."""
    for x in range(gridworld.width):
        for y in range(gridworld.height):
            pg.draw.rect(screen, pg.Color('white'),
                        [x*SQUARE_SIZE, y*SQUARE_SIZE, SQUARE_SIZE, SQUARE_SIZE], 1)

    pg.draw.rect(screen, pg.Color('darkgreen'), source_rect)
    pg.draw.rect(screen, pg.Color('darkblue'), goal_rect)


def draw_obstacles(screen, gridworld):
    """Draw an obstacle when a square is clicked."""
    for obstacle in gridworld.obstacles:
        pg.draw.rect(screen, pg.Color('red'),
                    [obstacle[0]*SQUARE_SIZE, obstacle[1]*SQUARE_SIZE,
                    SQUARE_SIZE, SQUARE_SIZE])


def remove_obstacles(screen, obstacles_removed):
    """If an obstacle already exists and is clicked, it removes the obstacle."""
    for obstacle in obstacles_removed:
        pg.draw.rect(screen, pg.Color('white'),
                    [obstacle[0]*SQUARE_SIZE, obstacle[1]*SQUARE_SIZE,
                    SQUARE_SIZE, SQUARE_SIZE], 1)


def draw_visited(screen, gridworld, visited):
    """Draw all squares that the stated algorithm visited."""
    for coordinates in visited:
        if coordinates != gridworld.source and coordinates != gridworld.goal:
            pg.draw.rect(screen, pg.Color('orange'),
                        [coordinates[0]*SQUARE_SIZE, coordinates[1]*SQUARE_SIZE,
                        SQUARE_SIZE, SQUARE_SIZE])


# TODO: draw a line instead of a square.
def draw_path(screen, gridworld, path):
    """Draw the path that the stated algorithm found."""
    for coordinates in path:
        if coordinates != gridworld.source and coordinates != gridworld.goal:
            pg.draw.rect(screen, pg.Color('yellow'),
                        [coordinates[0]*SQUARE_SIZE, coordinates[1]*SQUARE_SIZE,
                        SQUARE_SIZE, SQUARE_SIZE])


def get_grid_pos(mouse_x, mouse_y):
    """Get the grid position from the mouse position."""
    grid_x = mouse_x // SQUARE_SIZE
    grid_y = mouse_y // SQUARE_SIZE
    return grid_x, grid_y


def main():
    # Initialize screen and set title
    pg.init()
    pg.display.set_caption("Pathfinding Visualizer")

    screen = pg.display.set_mode(WINDOW_SIZE)
    background = pg.Surface(GRID_SIZE)
    manager = pygame_gui.UIManager((WINDOW_SIZE))

    # Set up the GUI
    algo_menu = pygame_gui.elements.UIDropDownMenu(relative_rect=pg.Rect((580, 40), (200, 30)),
                                                    manager=manager, options_list=["BFS", "DFS", "Dijkstra", "A*"],
                                                    starting_option="Algorithm:")
    start_button = pygame_gui.elements.UIButton(relative_rect=pg.Rect((580, 480), (100, 30)),
                                                text="Start", manager=manager)
    reset_button = pygame_gui.elements.UIButton(relative_rect=pg.Rect((680, 480), (100, 30)),
                                                text="Reset", manager=manager)    
                                                 
    # Initialize the grid along with the source node and the goal node.
    gridworld = Gridworld(GRID_SIZE[0]//SQUARE_SIZE, GRID_SIZE[1]//SQUARE_SIZE)
    source_rect, source_dragging = set_source(background, gridworld)
    goal_rect, goal_dragging = set_goal(background, gridworld)
    
    algo_chosen = None
    start = False
    obstacles_dragging = False
    obstacles_removed = []
    clock = pg.time.Clock() # for controlling the frame rate

    while True:
        time_delta = clock.tick(60) / 1000.0
        for event in pg.event.get():
            if event.type == pg.QUIT:
                return

            elif event.type == pg.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = event.pos
                if source_rect.collidepoint(event.pos): # move source node
                    source_dragging = True
                    source_offset_x = source_rect.x - mouse_x
                    source_offset_y = source_rect.y - mouse_y
                elif goal_rect.collidepoint(event.pos): # move goal node
                    goal_dragging = True
                    goal_offset_x = goal_rect.x - mouse_x
                    goal_offset_y = goal_rect.y - mouse_y
                else: # create obstacles
                    x, y = get_grid_pos(mouse_x, mouse_y)
                    if (x, y) not in gridworld.obstacles:
                        gridworld.obstacles.append((x, y))
                    else:
                        gridworld.obstacles.remove((x, y))
                        obstacles_removed.append((x, y))
                    obstacles_rect = pg.Rect([x*SQUARE_SIZE, y*SQUARE_SIZE,
                                            SQUARE_SIZE, SQUARE_SIZE])
                    obstacles_dragging = True
                    obstacles_offset_x = obstacles_rect.x - mouse_x
                    obstacles_offset_y = obstacles_rect.y - mouse_y
            
            elif event.type == pg.MOUSEMOTION:
                mouse_x, mouse_y = event.pos
                if source_dragging:
                    source_rect.x = (mouse_x + source_offset_x) // SQUARE_SIZE * SQUARE_SIZE
                    source_rect.y = (mouse_y + source_offset_y) // SQUARE_SIZE * SQUARE_SIZE
                    x, y = get_grid_pos(source_rect.x, source_rect.y)
                    gridworld.source = (x, y)
                elif goal_dragging:
                    goal_rect.x = (mouse_x + goal_offset_x) // SQUARE_SIZE * SQUARE_SIZE
                    goal_rect.y = (mouse_y + goal_offset_y) // SQUARE_SIZE * SQUARE_SIZE
                    x, y = get_grid_pos(goal_rect.x, goal_rect.y)
                    gridworld.goal = (x, y)
                elif obstacles_dragging:
                    obstacles_rect.x = (mouse_x + obstacles_offset_x) // SQUARE_SIZE * SQUARE_SIZE
                    obstacles_rect.y = (mouse_y + obstacles_offset_y) // SQUARE_SIZE * SQUARE_SIZE
                    x, y = get_grid_pos(obstacles_rect.x, obstacles_rect.y)
                    gridworld.obstacles.append((x, y))
            
            elif event.type == pg.MOUSEBUTTONUP:
                source_dragging = False
                goal_dragging = False
                obstacles_dragging = False

            elif event.type == pg.USEREVENT:
                if event.user_type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED:
                    algo_chosen = event.text
                elif event.user_type == pygame_gui.UI_BUTTON_PRESSED:
                    if event.ui_element == start_button:
                        start = True
                    elif event.ui_element == reset_button:
                        gridworld = Gridworld(GRID_SIZE[0]//SQUARE_SIZE, GRID_SIZE[1]//SQUARE_SIZE)
                        source_rect, source_dragging = set_source(screen, gridworld)
                        goal_rect, goal_dragging = set_goal(screen, gridworld)
                        start = False

            manager.process_events(event)
        manager.update(time_delta)

        screen.fill(pg.Color('grey'))
        background.fill(pg.Color('grey'))
        manager.draw_ui(screen)

        # Draw the grid with source and goal nodes, and then obstacles.
        draw_grid(background, gridworld, source_rect, goal_rect)
        draw_obstacles(background, gridworld)
        remove_obstacles(background, obstacles_removed)
        draw_obstacles(background, gridworld)
        screen.blit(background, (0, 0))

        if start:
            visited = bfs(gridworld, gridworld.source, gridworld.goal) # BFS as default
            if algo_chosen == "DFS":
                visited = dfs(gridworld, gridworld.source, gridworld.goal)
            elif algo_chosen == "Dijkstra":
                pass
            elif algo_chosen == "A*":
                pass
            path = reconstruct_path(visited, gridworld.source, gridworld.goal)
            draw_visited(background, gridworld, visited)
            draw_path(background, gridworld, path)
            screen.blit(background, (0, 0))

        pg.display.update()

    pg.quit()


if __name__ == "__main__":
    main()