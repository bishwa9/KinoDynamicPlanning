import heapq
import numpy
import time

class list_object(object):
    def __init__(self, action, parent):
        self.ActionToGetHere = action
        self.parent = parent

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):

        plan = []
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        # print start_config
        # if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
        self.planning_env.InitializePlot(goal_config)

        start_config_id = int(self.planning_env.discrete_env.ConfigurationToNodeId(start_config))
        goal_config_id = int(self.planning_env.discrete_env.ConfigurationToNodeId(goal_config))
        begin_config_id = start_config_id

        q = []
        index = 0

        no_vertices = reduce(lambda x, y: x * y, self.planning_env.discrete_env.num_cells, 1)

        parent = dict()
        dist_from_start = dict()

        heuristic_ = self.planning_env.ComputeHeuristicCost(goal_config_id,start_config_id)
        heapq.heappush(q, (0, index, start_config_id))
        index += 1
        parent[start_config_id] = -1
        dist_from_start[start_config_id] = 0
        num_nodes = 0

        while len(q) > 0:
            current = heapq.heappop(q)[-1]
            dist_till_current = dist_from_start[current]
            num_nodes += 1
            successors, successors_actions = self.planning_env.GetSuccessors(current)
            for action_idx in range(len(successors_actions)):
                action = successors_actions[action_idx]
                #path that was rolled out by the action
                neighbor_path = successors[action_idx]

                #collision check
                # Get the current transform
                T = self.planning_env.robot.GetTransform()
                inCollision = False
                for fp_idx in range(len(neighbor_path) - 1):
                    id1 = neighbor_path[fp_idx]
                    id2 = neighbor_path[fp_idx+1]
                    start_config = self.planning_env.discrete_env.NodeIdToConfiguration(id1)
                    end_config = self.planning_env.discrete_env.NodeIdToConfiguration(id2)
                    if self.planning_env.checkCollision(start_config, end_config):
                        inCollision = True
                        break
                 
                neighbor_id = neighbor_path[-1]

                start_config = self.planning_env.discrete_env.NodeIdToConfiguration(current)
                end_config = self.planning_env.discrete_env.NodeIdToConfiguration(neighbor_id)
                self.planning_env.PlotEdge(start_config, end_config)
                
                if neighbor_id not in parent and not inCollision:
                    parent[neighbor_id] = list_object(action, current)
                    # Plotting function
                    # if self.visualize:
                    #     self.planning_env.PlotEdge(start_config, end_config)
                    heuristic_ = self.planning_env.ComputeHeuristicCost(goal_config_id,neighbor_id)
                    cost_till = self.planning_env.ComputeDistance(current, neighbor_id) + dist_till_current
                    dist_from_start[neighbor_id] = cost_till
                    heapq.heappush(q, (10*heuristic_+cost_till, index, neighbor_id))
                    index += 1
                    if neighbor_id == goal_config_id:
                        root = neighbor_id
                        while root != -1:
                            plan.append(action)
                            parent_node = parent[root]
                            root = parent_node
                            action = parent_node.ActionToGetHere
                        # if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                        #     self.planning_env.PlotFinal(plan[::-1])
                        return plan[::-1]
        print "Done nothing"