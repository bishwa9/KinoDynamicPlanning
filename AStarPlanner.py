import numpy
import time


class list_object(object):
    def __init__(self, parent, action):
        self.ActionToGetHere = action
        self.parent = parent

class AStarPlanner(object):
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        # initializations
        plan = []
        start_node_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        print start_config
        print goal_config
        goal_node_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        open_list = [start_node_id]  # nodes not searched
        closed_list = []  # nodes searched
        parent_map = dict()  # map connecting parents and successors
        g_map = {start_node_id: 0}  # map connecting selected nodes to their distances from start
        h_cost = self.planning_env.ComputeHeuristicCost(start_node_id, goal_node_id)  # heuristic cost
        f_map = {start_node_id: h_cost}  # map connecting selected nodes to the total cost
        start_time = time.time()

        # check condition for visualize
        if self.visualize and hasattr(self.planning_env, "InitializePlot"):
            self.planning_env.InitializePlot(goal_config)

        # check if open list is not empty
        while open_list:

            # select node in open list with min f
            sel_node = min(f_map, key=lambda k: f_map[k])
            # check if the goal is reached
            if sel_node == goal_node_id:
                break

            # pop the selected node from the open list
            open_list.remove(sel_node)
            f_map.pop(sel_node)

            # update closed list
            closed_list.append(sel_node)

            # finding successors for selected node
            # successors = self.planning_env.GetSuccessors(sel_node)
            successors, successors_actions = self.planning_env.GetSuccessors(sel_node)

            # iterating for each successor
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

                if inCollision:
                    continue

                successor = neighbor_path[-1]

                # check if successor already visited
                if successor in closed_list:
                    continue

                # calculate distance for the successor
                tentative_g = g_map[sel_node] + self.planning_env.ComputeDistance(sel_node, successor)

                # check if successor not pushed in open list
                if successor not in open_list:
                    # push to open list
                    open_list.append(successor)

                # check if successor in open list
                elif tentative_g >= g_map[successor]:
                    continue

                # calculate all the costs for the successor
                h_cost_succ = self.planning_env.ComputeHeuristicCost(successor, goal_node_id)
                parent_map[successor] = list_object(sel_node, action)
                g_map[successor] = tentative_g
                f_map[successor] = g_map[successor] + h_cost_succ

        # put the nodes from the parent map to the plan if the goal is reached
        if sel_node == goal_node_id:
            plan = [action]
            while sel_node in parent_map.keys():
                sel_node = parent_map[sel_node]
                plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(sel_node))
            plan.reverse()  # reverse the plan

        dist = 0  # variable to find path length
        for i in range(len(plan) - 1):
            if self.visualize:
                self.planning_env.PlotEdge(plan[i], plan[i + 1])
            node_i = self.planning_env.discrete_env.ConfigurationToNodeId(plan[i])
            node_i1 = self.planning_env.discrete_env.ConfigurationToNodeId(plan[i + 1])
            dist += self.planning_env.ComputeDistance(node_i, node_i1)

        print "dist", dist  # calculating path length
        print "run time", time.time() - start_time  # calculating run time
        print "number of nodes", len(plan)  # calculating number of nodes
        return plan  # returning final plan
