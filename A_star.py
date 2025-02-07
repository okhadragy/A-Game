from math import *
import pygame as pg
import random,os,time


def heuristic(node, goal):
    """
    Calculate the heuristic value from the current node to the goal.
    This can be Manhattan distance, Euclidean distance, or other based on the graph type.
    
    Parameters:
    node (tuple or object): Current node position.
    goal (tuple or object): Goal node position.
    
    Returns:
    float: The heuristic value.
    """
    
    h = sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)
    return h


def a_star(graph, start, goal):
    """
    Implement the A* algorithm to find the shortest path from start to goal.

    Parameters:
    graph (Graph): The graph or grid on which A* will be performed.
    start (tuple or object): The starting node.
    goal (tuple or object): The goal node.

    Returns:
    list: The shortest path from start to goal.
    float: The total cost of the path.
    """
    visited_nodes = []
    open_nodes = {(start[0],start[1],0):0}
    came_from = {}
    path_cost = 0
    
    while open_nodes:
        least_cost_node = list(open_nodes.keys())[0]
        least_cost = open_nodes[least_cost_node]

        for node,cost in open_nodes.items():
            if (cost+heuristic(node,goal)) < (least_cost+heuristic(least_cost_node,goal)):
                least_cost = cost
                least_cost_node = node
                
        visited_nodes.append((least_cost_node[0],least_cost_node[1]))
        path_cost = least_cost
        open_nodes.pop(least_cost_node,None)
        
        # check if the current node is the goal
        if least_cost_node[0] == goal[0] and least_cost_node[1] == goal[1]:
            path = reconstruct_path(came_from,(least_cost_node[0],least_cost_node[1]))
            return (path,path_cost)

        for neighbour in graph[(least_cost_node[0],least_cost_node[1])]:
            if (neighbour[0],neighbour[1]) not in visited_nodes:
                came_from[(neighbour[0],neighbour[1])] = (least_cost_node[0],least_cost_node[1])
                cost = path_cost + neighbour[2]
                open_nodes[neighbour] = cost
    return None


def reconstruct_path(came_from, current):
    """
    Reconstruct the path from the start to the goal by backtracking from the goal node.
    
    Parameters:
    came_from (dict): Dictionary mapping nodes to their predecessors.
    current (tuple or object): The current node to backtrack from.
    
    Returns:
    list: The reconstructed path.
    """
    
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
        
    return path[::-1]


# Example Test Cases
def test_a_star():
    # Create a test graph and run the A* algorithm
    
    # seed = random.randint(1,100)
    maze = create_maze(100, 100)
    graph = create_graph(maze)
    start = list(graph.keys())[0]
    goal = list(graph.keys())[-1]
    result = a_star(graph,start, goal)
    
    if result:
        path,path_cost = result
        print("-----Path Cost-----")
        print(path_cost)
        visualize_path(maze,start,goal,path,0.00001,5)
    else:
        print("Couldn't find a path")
        visualize_path(maze,start,goal,[],0.00001,5)


def visualize_path(maze, start=(-1,-1), goal=(-1,-1), path=[], animation_time=0.02, scale=10):
    # visualize the path
    
    class Cell(pg.sprite.Sprite):
        def __init__(self, color = (0, 200, 255)):
            super(Cell, self).__init__()
            self.surf = pg.Surface((scale, scale))	
            self.surf.fill(color)
            self.rect = self.surf.get_rect()
    pg.init()
    screen = pg.display.set_mode((len(maze[0])*scale, len(maze)*scale))
    cell = Cell((0, 200, 255))
    path_cell = Cell((0, 255, 0))
    goal_cell = Cell((255, 0, 0))
    start_cell = Cell((255, 255, 0))
    obstacle = Cell((0, 0, 0))
    gameOn = True
    i = 0
    path_cells = []

    while gameOn:
        for event in pg.event.get():
            # Check for KEYDOWN event
            if event.type == pg.KEYDOWN:
                
                # If the Backspace key has been pressed set
                # running to false to exit the main loop
                if event.key == pg.K_BACKSPACE:
                    gameOn = False
                    
            # Check for QUIT event
            elif event.type == pg.QUIT:
                gameOn = False

        for y in range(0, len(maze)):
            for x in range(0, len(maze[0])):
                if maze[y][x] == "#":
                    screen.blit(obstacle.surf, (x*scale, y*scale))
                elif (x,y) == start:
                    screen.blit(start_cell.surf, (x*scale, y*scale))
                elif (x,y) == goal:
                    screen.blit(goal_cell.surf, (x*scale, y*scale))
                elif (x,y) in path_cells:
                    screen.blit(path_cell.surf, (x*scale, y*scale))
                else:
                    screen.blit(cell.surf, (x*scale, y*scale))
        
        pg.display.flip()
        time.sleep(animation_time)
        if i < len(path):
            x,y = path[i]
            screen.blit(path_cell.surf, (x*scale, y*scale))
            path_cells.append(path[i])
            i+=1
        

def create_maze(width, height, seed=42, obstacle_percentage=0.2, weighted_percentage=0.1):
    """
    Create a maze with random obstacles and weighted paths, but the same maze will
    be generated each time because of a fixed random seed.
    
    Parameters:
    width (int): Width of the maze.
    height (int): Height of the maze.
    obstacle_percentage (float): Percentage of cells that are obstacles.
    weighted_percentage (float): Percentage of cells that have a higher traversal cost.
    seed (int): Random seed to ensure the maze is the same each time.
    
    Returns:
    list: A 2D grid representing the maze. 
          '1' represents normal paths, 
          '#' represents obstacles, 
          values 2-5 represent weighted paths.
    """
    # Set the random seed to ensure consistent maze generation
    random.seed(seed)
    
    maze = []
    
    for i in range(height):
        row = []
        for j in range(width):
            # Randomly decide if the cell is an obstacle
            if random.random() < obstacle_percentage:
                row.append('#')  # Obstacle
            else:
                # Randomly assign weighted paths
                if random.random() < weighted_percentage:
                    row.append(random.randint(2, 5))  # Weighted path
                else:
                    row.append(1)  # Normal path with cost 1
        maze.append(row)
    
    return maze


def print_maze(maze):
    """
    Prints the maze in a human-readable format.
    
    Parameters:
    maze (list): A 2D grid representing the maze.
    """
    for row in maze:
        print(' '.join(str(cell) for cell in row))


def get_neighbors(x, y, maze):
    """
    Get valid neighbors of a cell (x, y) in the maze.
    
    Parameters:
    x (int): X-coordinate of the cell.
    y (int): Y-coordinate of the cell.
    maze (list): The maze represented as a 2D grid.
    
    Returns:
    list: List of valid neighbors as (neighbor_x, neighbor_y, cost) tuples.
    """
    neighbors = []
    height = len(maze)
    width = len(maze[0])
    
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
    
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0 <= ny < height and maze[ny][nx] != '#':
            neighbors.append((nx, ny, maze[ny][nx]))
    
    return neighbors


def create_graph(maze):
    """
    Create a graph from the maze where each cell is a key, and its neighbors (with weights) are the values.
    
    Parameters:
    maze (list): The maze represented as a 2D grid.
    
    Returns:
    dict: A dictionary representing the graph, where each key is a (x, y) tuple and the value is
          a list of neighbors with their respective costs [(neighbor_x, neighbor_y, cost), ...].
    """
    graph = {}
    
    for y in range(len(maze)):
        for x in range(len(maze[0])):
            if maze[y][x] != '#':  # If the cell is not an obstacle
                graph[(x, y)] = get_neighbors(x, y, maze)
    
    return graph


def print_graph(graph):
    """
    Print the graph structure where each cell has a list of its neighbors with their weights.
    
    Parameters:
    graph (dict): The graph structure.
    """
    for node, neighbors in graph.items():
        print(f"Cell {node}: {neighbors}")


# Example: Create a 100x100 maze and build the graph
os.system("cls")
test_a_star()

