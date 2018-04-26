import heapq
import math

class E160_maze:

    def __init__(self, maze):
        self.maze = maze
        self.walls = []
        self.get_maze_walls()

    def get_maze_walls(self):

        h = len(self.maze)
        w = len(self.maze[0])
        hslope = 'horizontal'
        vslope = 'vertical'

        walls = []
        start = ()
        end = ()

        for row in range(h):
            for col in range(w):
                if (not start) and self.maze[row][col][0]:
                    start = (col, -row)
                if start and (not self.maze[row][col][0]):
                    end = (col, -row)
                if (col == w - 1) and start and self.maze[row][col][0]:
                    end = ((col + 1), -row)
                if end and start:
                    self.walls.append((start, end, hslope))
                    start = ()
                    end = ()

        for col in range(w):
            if (not start) and self.maze[h - 1][col][2]:
                start = (col, -h)
            if start and (not self.maze[(h - 1)][col][2]):
                end = (col, -h)
            if (col == w - 1) and start and self.maze[(h - 1)][col][2]:
                end = ((col + 1), -h)
            if end and start:
                self.walls.append((start, end, hslope))
                start = ()
                end = ()

        for col in range(w):
            for row in range(h):
                if (not start) and self.maze[row][col][1]:
                    start = (col, -row)
                if start and (not self.maze[row][col][1]):
                    end = (col, -row)
                if (row == h - 1) and start and self.maze[row][col][1]:
                    end = (col, -(row + 1))
                if end and start:
                    self.walls.append((start, end, vslope))
                    start = ()
                    end = ()

        for row in range(h):
            if (not start) and self.maze[row][w - 1][3]:
                start = (w, -row)
            if start and (not self.maze[row][w - 1][3]):
                end = (w, -row)
            if (row == h - 1) and start and self.maze[row][w - 1][3]:
                end = (w, -(row + 1))
            if end and start:
                self.walls.append((start, end, vslope))
                start = ()
                end = ()

    # Manhattan distance
    def heuristic(self, pos, goal):
        return math.fabs(goal[0] - pos[0]) + math.fabs(goal[1] - pos[1])

    def getSuccessors(self, pos):
        moves = self.maze[pos[1]][pos[0]]
        successors = []
        if moves[0] == 0:
            successors.append((pos[0], pos[1] - 1, math.pi / 2))
        if moves[1] == 0:
            successors.append((pos[0] - 1, pos[1], math.pi))
        if moves[2] == 0:
            successors.append((pos[0], pos[1] + 1, -math.pi / 2))
        if moves[3] == 0:
            successors.append((pos[0] + 1, pos[1], 0))
        return successors

    def aStarSearch(self, start, goal):
        closed = []
        fringe = []
        startNode = Node(start[0], start[1], start[2], 0, [])
        heapq.heappush(fringe, (self.heuristic(start, goal), startNode))

        while fringe:
            _, node = heapq.heappop(fringe)
            if (node.x, node.y) == (goal[0], goal[1]):
                return node.path
            if (node.x, node.y) not in closed:
                closed.append((node.x, node.y))
                for (x, y, dir) in self.getSuccessors((node.x, node.y)):
                    childNode = Node(x,
                                     y,
                                     dir,
                                     node.cost + 1,
                                     node.path + [dir])
                    heapq.heappush(fringe, (node.cost + 1 + self.heuristic((x, y), goal), childNode))
        return []

class Node:
    def __init__(self, x, y, direction, cost, path):
        self.x = x
        self.y = y
        self.direction = direction
        self.cost = cost
        self.path = path
#
# def correctMaze(maze):
#     h = len(maze)
#     w = len(maze[0])
#     for row in range(h - 1):
#         for col in range(w - 1):
#             if maze[row][col][3] or maze[row][col + 1][1]:
#                 maze[row][col][3] = maze[row][col + 1][1] = 1
#             if maze[row][col][2] or maze[row + 1][col][0]:
#                 maze[row][col][2] = maze[row + 1][col][0] = 1
#     if maze[h - 1][w - 1][1] or maze[h - 1][w - 2][3]:
#         maze[h - 1][w - 1][1] = maze[h - 1][w - 2][3] = 1
#     if maze[h - 1][w - 1][0] or maze[h - 2][w - 1][2]:
#         maze[h - 1][w - 1][0] = maze[h - 2][w - 1][2] = 1
#     return maze
#
# def main():
#     # NWSE, 0 can go through, 1 is wall, 2 is exit
#     maze1 = [[[1,1,1,0], [1,0,0,0], [1,0,1,0], [1,0,0,1]],
#              [[1,1,1,0], [0,0,0,1], [1,1,1,0], [0,0,1,1]],
#              [[0,1,1,1], [0,1,1,0], [1,0,1,0], [1,0,0,1]]]
#
#     # x, y, direction
#     s = [0, 0, 3]
#     g = [3, 2, 2]
#
#     print aStarSearch(maze1, s, g)
#
#     print
#
#     maze2 = [[[1,1,1,0], [1,1,0,1]],
#              [[0,1,1,1], [1,0,1,1]]]
#
#     cMaze2 = correctMaze(maze2)
#
#     print cMaze2
#     print aStarSearch(cMaze2, [0,0,3], [1,1,2])
#
#     maze3 = [[[1,1,0,0], [1,0,1,0], [1,0,0,1]],
#              [[0,1,0,0], [1,0,1,0], [0,0,1,1]],
#              [[0,1,1,0], [1,0,1,0], [1,0,1,1]]]
#
#     print aStarSearch(maze3, [2,1,3], [2,2,2])
#
# if __name__ == "__main__":
#     main()
