from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def get_sucessors(self, node):
        tuples_list = self.node_grid.get_successors(node.i, node.j)
        successor_list = []
        for tuple in tuples_list:
            successor_list.append(self.node_grid.get_node(tuple[0], tuple[1]))
        return successor_list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        self.node_grid.reset()
        pq=[]
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        start_node.g=0
        heapq.heappush(pq,(start_node.g, start_node))

        while not pq==[]:
            f,node=heapq.heappop(pq)
            node.closed = True
            successor_list=self.get_sucessors(node)

            if node.get_position()==goal_position:
                return self.construct_path(node),node.g

            for successor in successor_list:
                if not successor.closed:
                    if successor.g>node.g+successor.distance_to(node.i,node.j):
                        successor.g = node.g + successor.distance_to(node.i, node.j)
                        successor.parent=node
                        heapq.heappush(pq,(successor.g,successor))





        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path



    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        self.node_grid.reset()

        pq=[]
        start_node=self.node_grid.get_node(start_position[0],start_position[1])
        start_node.g=start_node.distance_to(goal_position[0],goal_position[1])
        heapq.heappush(pq,(start_node.g,start_node))

        while not pq==[]:
            f,node=heapq.heappop(pq)

            successor_list=self.get_sucessors(node)
            node.closed=True
            for successor in successor_list:
                if not successor.closed:
                    successor.closed=True
                    successor.parent = node
                    if successor.get_position() == goal_position:
                        return self.construct_path(successor), start_node.g
                    successor.g=successor.distance_to(goal_position[0],goal_position[1])
                    heapq.heappush(pq,(successor.g,successor))





        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path


    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        self.node_grid.reset()
        pq=[]
        start_node=self.node_grid.get_node(start_position[0],start_position[1])
        start_node.g=0
        start_node.f=start_node.distance_to(goal_position[0],goal_position[1])

        heapq.heappush(pq,(start_node.g,start_node))

        while not pq==[]:

            f,node=heapq.heappop(pq)
            node.closed=True

            if node.get_position()==goal_position:
                return self.construct_path(node), node.g

            successor_list=self.get_sucessors(node)
            for successor in successor_list:
                if not successor.closed:
                    successor.closed=True
                    if successor.f>node.g+node.distance_to(successor.i,successor.j)+successor.distance_to(goal_position[0],goal_position[1]):
                        successor.g=node.g+node.distance_to(successor.i,successor.j)
                        successor.f=successor.g+successor.distance_to(goal_position[0],goal_position[1])
                        successor.parent=node
                        heapq.heappush(pq,(successor.g,successor))


        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path


