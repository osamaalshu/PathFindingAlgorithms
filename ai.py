from __future__ import print_function
from heapq import *  # Hint: Use heappop and heappush

ACTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]


class AI:
    def __init__(self, grid, type):
        self.grid = grid
        self.set_type(type)
        self.set_search()

    def set_type(self, type):
        self.final_cost = 0
        self.type = type

    def set_search(self):
        self.final_cost = 0
        self.grid.reset()
        self.finished = False
        self.failed = False
        self.previous = {}

        # Initialization of algorithms goes here
        if self.type == "dfs":
            self.frontier = [self.grid.start]
            self.explored = []
        elif self.type == "bfs":
            self.frontier = [self.grid.start]
            self.explored = []
        elif self.type == "ucs":
            self.frontier = [(0, self.grid.start)]
            self.costs = {self.grid.start: 0}
            self.explored = set()
        elif self.type == "astar":
            self.frontier = [(self.manhattan_heuristic(
                self.grid.start), 0, self.grid.start)]
            self.costs = {self.grid.start: 0}
            self.explored = set()

    def get_result(self):
        total_cost = 0
        current = self.grid.goal
        while not current == self.grid.start:
            total_cost += self.grid.nodes[current].cost()
            current = self.previous[current]
            # This turns the color of the node to red
            self.grid.nodes[current].color_in_path = True
        total_cost += self.grid.nodes[current].cost()
        self.final_cost = total_cost

    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    def dfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = self.frontier.pop()

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False
        self.explored.append(current)

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle and n not in self.frontier and n not in self.explored:
                    self.previous[n] = current
                    self.frontier.append(n)
                    self.grid.nodes[n].color_frontier = True

    # Implement BFS here (Don't forget to implement initialization at line 23)

    def bfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = self.frontier.pop(0)

        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False
        self.explored.append(current)

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle and n not in self.frontier and n not in self.explored:
                    self.previous[n] = current
                    self.frontier.append(n)
                    self.grid.nodes[n].color_frontier = True

    # Implement UCS here (Don't forget to implement initialization at line 23)

    def ucs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return

        current_cost, current = heappop(self.frontier)

        if current == self.grid.goal:
            self.finished = True
            return

        if current not in self.explored:
            self.explored.add(current)
            children = [(current[0] + a[0], current[1] + a[1])
                        for a in ACTIONS]
            self.grid.nodes[current].color_checked = True
            self.grid.nodes[current].color_frontier = False

            for n in children:
                if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                    child_cost = current_cost + self.grid.nodes[n].cost()
                    if not self.grid.nodes[n].puddle and (n not in self.explored and n not in [x[1] for x in self.frontier] or child_cost < self.costs.get(n, float('inf'))):
                        self.previous[n] = current
                        self.costs[n] = child_cost
                        heappush(self.frontier, (child_cost, n))
                        self.grid.nodes[n].color_frontier = True

    # Implement Astar here (Don't forget to implement initialization at line 23)

    def astar_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return

        _, current_cost, current = heappop(self.frontier)

        if current == self.grid.goal:
            self.finished = True
            return

        if current not in self.explored:
            self.explored.add(current)
            children = [(current[0] + a[0], current[1] + a[1])
                        for a in ACTIONS]
            self.grid.nodes[current].color_checked = True
            self.grid.nodes[current].color_frontier = False

            for n in children:
                if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                    child_cost = current_cost + self.grid.nodes[n].cost()
                    if not self.grid.nodes[n].puddle and (n not in self.explored and n not in [x[2] for x in self.frontier] or child_cost < self.costs.get(n, float('inf'))):
                        self.previous[n] = current
                        self.costs[n] = child_cost
                        total_cost = child_cost + self.manhattan_heuristic(n)
                        heappush(self.frontier, (total_cost, child_cost, n))
                        self.grid.nodes[n].color_frontier = True

    def manhattan_heuristic(self, position):
        goal = self.grid.goal
        return abs(position[0] - goal[0]) + abs(position[1] - goal[1])
