import heapq
import math

# Manhattan distance
def heuristic(pos, goal):
    return math.fabs(goal[0] - pos[0]) + math.fabs(goal[1] - pos[1])

def getSuccessors(maze, pos):
    moves = maze[pos[1]][pos[0]]
    successors = []
    if moves[0] == 0:
        successors.append((pos[0], pos[1] - 1, 0))
    if moves[1] == 0:
        successors.append((pos[0] - 1, pos[1], 1))
    if moves[2] == 0:
        successors.append((pos[0], pos[1] + 1, 2))
    if moves[3] == 0:
        successors.append((pos[0] + 1, pos[1], 3))
    return successors

def aStarSearch(maze, start, goal):
    closed = []
    fringe = []
    startNode = Node(start[0], start[1], start[2], 0, [])
    heapq.heappush(fringe, (heuristic(start, goal), startNode))

    while fringe:
        _, node = heapq.heappop(fringe)
        if (node.x, node.y) == (goal[0], goal[1]):
            return node.path
        if (node.x, node.y) not in closed:
            closed.append((node.x, node.y))
            for (x, y, dir) in getSuccessors(maze, (node.x, node.y)):
                childNode = Node(x,
                                 y,
                                 dir,
                                 node.cost + 1,
                                 node.path + [dir])
                heapq.heappush(fringe, (node.cost + 1 + heuristic((x, y), goal), childNode))
    return []

class Node:
    def __init__(self, x, y, direction, cost, path):
        self.x = x
        self.y = y
        self.direction = direction
        self.cost = cost
        self.path = path

def correctMaze(maze):
    h = len(maze)
    w = len(maze[0])
    for row in range(h - 1):
        for col in range(w - 1):
            if maze[row][col][3] or maze[row][col + 1][1]:
                maze[row][col][3] = maze[row][col + 1][1] = 1
            if maze[row][col][2] or maze[row + 1][col][0]:
                maze[row][col][2] = maze[row + 1][col][0] = 1
    if maze[h - 1][w - 1][1] or maze[h - 1][w - 2][3]:
        maze[h - 1][w - 1][1] = maze[h - 1][w - 2][3] = 1
    if maze[h - 1][w - 1][0] or maze[h - 2][w - 1][2]:
        maze[h - 1][w - 1][0] = maze[h - 2][w - 1][2] = 1
    return maze

def main():
    # NWSE, 0 can go through, 1 is wall, 2 is exit
    maze1 = [[[1,1,1,0], [1,0,0,0], [1,0,1,0], [1,0,0,1]],
             [[1,1,1,0], [0,0,0,1], [1,1,1,0], [0,0,1,1]],
             [[0,1,1,1], [0,1,1,0], [1,0,1,0], [1,0,0,1]]]

    # x, y, direction
    s = [0, 0, 3]
    g = [3, 2, 2]

    print aStarSearch(maze1, s, g)

    print

    maze2 = [[[1,1,1,0], [1,1,0,1]],
             [[0,1,1,1], [1,0,1,1]]]

    cMaze2 = correctMaze(maze2)

    print cMaze2
    print aStarSearch(cMaze2, [0,0,3], [1,1,2])

    maze3 = [[[1,1,0,0], [1,0,1,0], [1,0,0,1]],
             [[0,1,0,0], [1,0,1,0], [0,0,1,1]],
             [[0,1,1,0], [1,0,1,0], [1,0,1,1]]]

    print aStarSearch(maze3, [2,1,3], [2,2,2])

if __name__ == "__main__":
    main()
