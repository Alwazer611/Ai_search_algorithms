import heapq
from problem import *
from node import *


class PriorityQueue:
    def __init__(self, items=(), priority_function=(lambda x: x)):
        self.priority_function = priority_function
        self.pqueue = []
        # add the items to the PQ
        for item in items:
            self.add(item)

    def add(self, item):
        pair = (self.priority_function(item), item)
        heapq.heappush(self.pqueue, pair)

    def pop(self):
        return heapq.heappop(self.pqueue)[1]

    def __len__(self):
        return len(self. pqueue)


def expand(problem, node):
    a = []
    s = node.state
    for action in problem.actions(s):
        new_s = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, new_s)
        a.append(Node(state=new_s, parent_node=node,
                 action_from_parent=action, path_cost=cost))
    return a


def get_path_actions(node):
    result = []
    if node == None:
        return result
    elif node.parent_node == None:
        return result
    else:
        while node.parent_node:
            result.append(node.action_from_parent)
            node = node.parent_node
    result.reverse()
    return result


def get_path_states(node):
    result = []
    if node == None:
        return result
    else:
        while node:
            result.append(node.state)
            node = node.parent_node

    result.reverse()
    return result


def best_first_search(problem, f):

    node = Node(problem.initial_state)
    frontier = PriorityQueue([node], priority_function=f)

    reached = {
        problem.initial_state: node
    }
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node

        for child in expand(problem, node):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.add(child)
    return None


def best_first_search_treelike(problem, f):
    node = Node(problem.initial_state)
    frontier = PriorityQueue([node], priority_function=f)
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node

        for child in expand(problem,  node):
            frontier.add(child)
    return None


def breadth_first_search(problem, treelike=False):
    if treelike:
        return best_first_search_treelike(problem, b_depth)
    else:
        return best_first_search(problem, b_depth)


def depth_first_search(problem, treelike=False):
    if treelike:
        return best_first_search_treelike(problem, d_depth)
    else:
        return best_first_search(problem, d_depth)


def uniform_cost_search(problem, treelike=False):
    if treelike:
        return best_first_search_treelike(problem, g)
    else:
        return best_first_search(problem, g)


def greedy_search(problem, h, treelike=False):
    if treelike:
        return best_first_search_treelike(problem, h)
    else:
        return best_first_search(problem, h)


def astar_search(problem, h, treelike=False):
    def f(n):
        return h(n) + g(n)
    if treelike:
        return best_first_search_treelike(problem, f)
    else:
        return best_first_search(problem, f)


def g(n):
    return n.path_cost


def b_depth(node):
    return node.depth


def d_depth(node):
    return -node.depth


class RouteProblem (Problem):

    def __init__(self, initial_state, goal_state=None, map_graph=None, map_coords=None):
        self.goal_state = goal_state
        self.initial_state = initial_state

        self.map_graph = map_graph
        self.map_coords = map_coords

    def actions(self, state):
        result = []
        for (v1, x), c in self.map_graph.items():
            if v1 == state:
                result.append(x)

        return result

    def result(self, state, action):
        result = []
        for (v1, x), c in self.map_graph.items():
            if v1 == state:
                result.append(x)

        l = len(result)
        for z in range(l):
            if(action == result[z]):
                return action

        return state

    def action_cost(self, state1, action, state2):

        return self.map_graph.get((state1, state2))

    def h(self, node):

        if(node.state == self.goal_state):
            return 0

        goalx = self.map_coords[self.goal_state][0]
        goaly = self.map_coords[self.goal_state][1]

        nodex = self.map_coords[node.state][0]
        nodey = self.map_coords[node.state][1]

        return ((goalx - nodex)**2 + (goaly - nodey)**2)**0.5


# example_map_graph = {
#     ('A', 'B'): 1,
#     ('A', 'C'): 1,
#     ('A', 'D'): 1,
#     ('B', 'A'): 1,
#     ('B', 'C'): 1,
#     ('B', 'E'): 1,
#     ('C', 'B'): 1
# }
# example_coords = {
#     'A': (1, 2),
#     'B': (0, 1),
#     'C': (1, 1),
#     'D': (2, 1),
#     'E': (0, 0),
# }

# example_route_problem = RouteProblem(
#     initial_state='A', goal_state='E', map_graph=example_map_graph, map_coords=example_coords)

# goal_node = breadth_first_search(example_route_problem, tree_like=False)


class GridProblem(Problem):

    def __init__(self, initial_state, N, M, wall_coords, food_coords):

        self.M = M
        self.N = N

        self.wall_coords = wall_coords
        self.food_coords = food_coords

        l = [False for i in range(len(food_coords))]

        food_eaten = tuple(l)

        self.initial_state = (initial_state, food_eaten)

    def actions(self, state):

        actionsl = []
        x = state[0][0]
        y = state[0][1]

        food_eaten = state[1]

        if(y+1 <= self.N and (x, y+1)not in self.wall_coords):
            actionsl.append("up")

        if(y-1 <= self.N and y-1 != 0 and (x, y-1) not in self.wall_coords):
            actionsl.append("down")

        if(x+1 <= self.M and (x+1, y)not in self.wall_coords):
            actionsl.append("right")

        if(x-1 <= self.M and x-1 != 0 and (x-1, y) not in self.wall_coords):
            actionsl.append("left")
        return actionsl

    def result(self, state, action):
        x = state[0][0]
        y = state[0][1]

        food_eaten = list(state[1])
        if(action in self.actions(state)):

            if(action == "up"):
                if((x, y+1) in self.food_coords):
                    idx = self.food_coords.index((x, y+1))
                    food_eaten[idx] = True
                food_eaten = tuple(food_eaten)
                l = ((x, y+1), food_eaten)
                state = l
                return state

            if(action == "down"):
                if((x, y-1) in self.food_coords):
                    idx = self.food_coords.index((x, y-1))
                    food_eaten[idx] = True
                food_eaten = tuple(food_eaten)
                l = ((x, y-1), food_eaten)
                state = l
                return state

            if(action == "right"):
                if((x+1, y) in self.food_coords):
                    idx = self.food_coords.index((x+1, y))
                    food_eaten[idx] = True
                food_eaten = tuple(food_eaten)
                l = ((x+1, y), food_eaten)
                state = l
                return state

            if(action == "left"):
                if((x-1, y) in self.food_coords):
                    idx = self.food_coords.index((x-1, y))
                    food_eaten[idx] = True
                food_eaten = tuple(food_eaten)
                l = ((x-1, y), food_eaten)
                state = l
                return state
        else:

            return state

    def action_cost(self, state1, action, state2):

        return 1

    def is_goal(self, state):
        return all(state[1])

    def h(self, node):

        if(self.is_goal(node.state)):
            return 0
        x = node.state[0][0]
        y = node.state[0][1]

        food_eaten = node.state[1]

        list = []
        for i in range(len(self.food_coords)):
            if(food_eaten[i] == False):

                x_2 = self.food_coords[i][0]
                y_2 = self.food_coords[i][1]

                x_pos = abs(x-x_2)
                y_pos = abs(y-y_2)
                m_d = x_pos+y_pos
                list.append(m_d)

        return min(list)
