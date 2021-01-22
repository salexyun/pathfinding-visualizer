# @author: S. Alex Yun
# A collection of pathfinding algorithms.

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

# TODO: Allow diagonal movements in the grid.
class Gridworld:
    """
    A two-dimensional (m x n) lattice graph.
    Adapted from: https://www.redblobgames.com/pathfinding/a-star
    """
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.source = ()
        self.goal = ()
        self.obstacles = []

    def in_bounds(self, node):
        (x, y) = node
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, node):
        return node not in self.obstacles

    def get_neighbours(self, node):
        (x, y) = node
        neighbours = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # E W N S
        if (x + y) % 2 == 0: neighbours.reverse() # S N W E
        neighbours = filter(self.in_bounds, neighbours)
        neighbours = filter(self.passable, neighbours)
        return neighbours


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

        for next_node in graph.get_neighbours(cur_node):
            if next_node not in visited:
                visited[next_node] = cur_node
                frontier_queue.append(next_node)

    return visited


def dfs(graph, source_node, goal_node):
    """Depth-first search (iterative version)."""
    visited = set()
    frontier_stack = deque([source_node])
    path = {source_node: None}
    while frontier_stack:
        cur_node = frontier_stack.pop()

        if cur_node == goal_node:
            break

        if cur_node not in visited:
            visited.add(cur_node)
            for next_node in graph.get_neighbours(cur_node):
                if next_node not in path: # cycle check
                    frontier_stack.append(next_node)
                    path[next_node] = cur_node

    return path


def dijkstra(graph, source_node, goal_node):
    """Also known as the uniform cost search."""
    visited = {source_node: None}
    cost = defaultdict(lambda: math.inf)
    cost[source_node] = 0
    frontier_pq = [(0, source_node)] # min_heap, prioritizing by cost
    while frontier_pq:
        cur_cost, cur_node = heapq.heappop(frontier_pq)

        if cur_node == goal_node:
            break
        
        for next_node in graph.get_neighbours(cur_node):
            alt_cost = cur_cost + graph.cost(cur_node, next_node)
            if alt_cost < cost[next_node]:
                visited[next_node] = cur_node
                cost[next_node] = alt_cost
                heapq.heappush(frontier_pq, (alt_cost, next_node))
    
    return visited, cost


# TODO: Add different types of heuristics.
def heuristic(cur_node, goal_node):
    """
    It returns the estimated cost of the cheapest path
    from the current node to the goal node, using Manhattan distance.
    """
    (x1, y1) = cur_node
    (x2, y2) = goal_node
    return abs(x1 - x2) + abs(y1 - y2)


def a_star(graph, source_node, goal_node):
    """
    A* search is a type of best-first search and
    can be viewed as an extension of Dijkstra's algorithm.
    """
    visited = {source_node: None}
    cost = defaultdict(lambda: math.inf)
    cost[source_node] = 0
    frontier_pq = [(0, source_node)]
    while frontier_pq:
        _, cur_node = heapq.heappop(frontier_pq)

        if cur_node == goal_node:
            break
        
        for next_node in graph.get_neighbours(cur_node):
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
    reconst_path = []
    while current != source:
        reconst_path.append(current)
        current = visited[current]
    reconst_path.append(source)
    reconst_path.reverse()
    return reconst_path, len(reconst_path)


WINDOW_SIZE = [800, 600]
GRID_SIZE = [500, 500]
TILE_SIZE = 20


def set_source(screen, gridworld):
    """
    Initializes the source node in the Gridworld and
    returns it as a Pygame Rect object (to be displayed onto the screen).
    """
    gridworld.source = (0, 0)
    source_rect = pg.Rect([gridworld.source[0]*TILE_SIZE, gridworld.source[1]*TILE_SIZE,
                            TILE_SIZE, TILE_SIZE])
    source_dragging = False
    return source_rect, source_dragging


def set_goal(screen, gridworld):
    """
    Initializes the goal node in the Gridworld and
    returns it as a Pygame Rect object (to be displayed onto the screen).
    """
    gridworld.goal = (gridworld.width-1, gridworld.height-1)
    goal_rect = pg.Rect([gridworld.goal[0]*TILE_SIZE, gridworld.goal[1]*TILE_SIZE,
                        TILE_SIZE, TILE_SIZE])
    goal_dragging = False
    return goal_rect, goal_dragging


# TODO: Draw weighted grids with the ability to specify the weight on the GUI.
def draw_grid(screen, gridworld, source_rect, goal_rect):
    """Draws the Gridworld, source Rect, and goal Rect onto the screen."""
    for x in range(gridworld.width):
        for y in range(gridworld.height):
            pg.draw.rect(screen, pg.Color('lightgrey'),
                        [x*TILE_SIZE, y*TILE_SIZE, TILE_SIZE, TILE_SIZE], 1)

    pg.draw.rect(screen, pg.Color('darkgreen'), source_rect)
    pg.draw.rect(screen, pg.Color('darkblue'), goal_rect)


def draw_obstacles(screen, gridworld):
    "Draws an obstacle when a square in the Gridworld is clicked."""
    for obstacle in gridworld.obstacles:
        pg.draw.rect(screen, pg.Color('red'),
                    [obstacle[0]*TILE_SIZE, obstacle[1]*TILE_SIZE,
                    TILE_SIZE, TILE_SIZE])


def remove_obstacles(screen, obstacles_removed):
    """
    Opposite of draw_obstacles(): if an obstacle already exists
    (i.e., sqaure is coloured), clicking this square
    removes the obstacle and "uncolours" the square.
    """
    for obstacle in obstacles_removed:
        pg.draw.rect(screen, pg.Color('white'),
                    [obstacle[0]*TILE_SIZE, obstacle[1]*TILE_SIZE,
                    TILE_SIZE, TILE_SIZE], 1)


def draw_visited(screen, gridworld, visited):
    """Draws all squares that the stated algorithm visited."""
    for coordinates in visited:
        if coordinates != gridworld.source and coordinates != gridworld.goal:
            pg.draw.rect(screen, pg.Color('lightgrey'),
                        [coordinates[0]*TILE_SIZE, coordinates[1]*TILE_SIZE,
                        TILE_SIZE, TILE_SIZE])


# TODO: draw a line instead of a square.
def draw_path(screen, gridworld, reconst_path):
    """Draws the path that the stated algorithm found."""
    for coordinates in reconst_path:
        if coordinates != gridworld.source and coordinates != gridworld.goal:
            pg.draw.rect(screen, pg.Color('yellow'),
                        [coordinates[0]*TILE_SIZE, coordinates[1]*TILE_SIZE,
                        TILE_SIZE, TILE_SIZE])


def get_grid_pos(mouse_x, mouse_y):
    """Gets the grid position from the mouse position."""
    grid_x = mouse_x // TILE_SIZE
    grid_y = mouse_y // TILE_SIZE
    return grid_x, grid_y


def main():
    # Initialize the screen and set the window caption
    pg.init()
    pg.display.set_caption("Pathfinding Visualizer")

    screen = pg.display.set_mode(WINDOW_SIZE)
    background = pg.Surface(GRID_SIZE)
    manager = pygame_gui.UIManager((WINDOW_SIZE))

    # Initialize up the GUI on the right side.
    _ = pygame_gui.elements.UIDropDownMenu(relative_rect=pg.Rect((580, 40), (200, 30)),
                                            manager=manager, options_list=["BFS", "DFS", "Dijkstra", "A*"],
                                            starting_option="Algorithm:")
    start_button = pygame_gui.elements.UIButton(relative_rect=pg.Rect((580, 480), (100, 30)),
                                                text="Start", manager=manager)
    reset_button = pygame_gui.elements.UIButton(relative_rect=pg.Rect((680, 480), (100, 30)),
                                                text="Reset", manager=manager)    

    # Initialize the grid, source node, and goal node.
    grid = WeightedGridworld(GRID_SIZE[0]//TILE_SIZE, GRID_SIZE[1]//TILE_SIZE)
    source_rect, source_dragging = set_source(background, grid)
    goal_rect, goal_dragging = set_goal(background, grid)
    obstacles_dragging = False

    algo_chosen = None
    start = False
    obstacles_removed = []

    clock = pg.time.Clock() # for controlling the frame rate
    while True:
        time_delta = clock.tick(60) / 1000.0
        for event in pg.event.get():
            if event.type == pg.QUIT:
                return

            elif event.type == pg.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = event.pos
                if source_rect.collidepoint(event.pos): # source clicked to be moved
                    source_dragging = True
                    source_offset_x = source_rect.x - mouse_x
                    source_offset_y = source_rect.y - mouse_y
                elif goal_rect.collidepoint(event.pos): # goal clicked to be moved
                    goal_dragging = True
                    goal_offset_x = goal_rect.x - mouse_x
                    goal_offset_y = goal_rect.y - mouse_y
                else: # empty square clicked -> create obstacles
                    x, y = get_grid_pos(mouse_x, mouse_y)
                    if (x, y) not in grid.obstacles:
                        grid.obstacles.append((x, y))
                    else:
                        grid.obstacles.remove((x, y))
                        obstacles_removed.append((x, y))

                    obstacles_rect = pg.Rect([x*TILE_SIZE, y*TILE_SIZE,
                                            TILE_SIZE, TILE_SIZE])
                    obstacles_dragging = True
                    obstacles_offset_x = obstacles_rect.x - mouse_x
                    obstacles_offset_y = obstacles_rect.y - mouse_y
            
            # source, goal, or obstacle being dragged into new position(s); update position(s) accordingly.
            elif event.type == pg.MOUSEMOTION:
                mouse_x, mouse_y = event.pos
                if source_dragging:
                    source_rect.x = (mouse_x + source_offset_x) // TILE_SIZE * TILE_SIZE
                    source_rect.y = (mouse_y + source_offset_y) // TILE_SIZE * TILE_SIZE
                    x, y = get_grid_pos(source_rect.x, source_rect.y)
                    grid.source = (x, y)
                elif goal_dragging:
                    goal_rect.x = (mouse_x + goal_offset_x) // TILE_SIZE * TILE_SIZE
                    goal_rect.y = (mouse_y + goal_offset_y) // TILE_SIZE * TILE_SIZE
                    x, y = get_grid_pos(goal_rect.x, goal_rect.y)
                    grid.goal = (x, y)
                elif obstacles_dragging:
                    obstacles_rect.x = (mouse_x + obstacles_offset_x) // TILE_SIZE * TILE_SIZE
                    obstacles_rect.y = (mouse_y + obstacles_offset_y) // TILE_SIZE * TILE_SIZE
                    x, y = get_grid_pos(obstacles_rect.x, obstacles_rect.y)
                    grid.obstacles.append((x, y))
            
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
                    elif event.ui_element == reset_button: # wipe out the grid
                        grid = WeightedGridworld(GRID_SIZE[0]//TILE_SIZE, GRID_SIZE[1]//TILE_SIZE)
                        source_rect, source_dragging = set_source(screen, grid)
                        goal_rect, goal_dragging = set_goal(screen, grid)
                        start = False

            manager.process_events(event)
        manager.update(time_delta)

        # Set up the screen, background, and GUI.
        screen.fill(pg.Color('darkgrey'))
        background.fill(pg.Color('darkgrey'))
        manager.draw_ui(screen)

        # Draw the grid, source, goal, and obstacles onto the screen.
        draw_grid(background, grid, source_rect, goal_rect)
        draw_obstacles(background, grid)
        remove_obstacles(background, obstacles_removed)
        draw_obstacles(background, grid)
        screen.blit(background, (0, 0))

        # User chose an algorithm, draw the path on the grid accordingly.
        if start:
            visited = bfs(grid, grid.source, grid.goal) # BFS as default
            if algo_chosen == "DFS":
                visited = dfs(grid, grid.source, grid.goal)
            elif algo_chosen == "Dijkstra":
                visited, _ = dijkstra(grid, grid.source, grid.goal)
            elif algo_chosen == "A*":
                visited, _ = a_star(grid, grid.source, grid.goal)
            draw_visited(background, grid, visited)
            path, length = reconstruct_path(visited, grid.source, grid.goal)
            draw_path(background, grid, path)
            
            font = pg.font.SysFont('courier new', 18)
            text = font.render(
                f"Path length = {length-1}", 1, pg.Color('white'))
            text_pos = text.get_rect()
            text_pos.bottomleft = (20, 560)

            screen.blit(text, text_pos)
            screen.blit(background, (0, 0))

        pg.display.update()

    pg.quit()


if __name__ == "__main__":
    main()