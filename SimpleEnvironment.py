import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution

        self.control_res = 0.5
        self.control_min = -1
        self.control_max = 1
        self.control_inc = numpy.arange(self.control_min, self.control_max, self.control_res)

        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

    def checkCollision(self, start_config, end_config):
        no_samples = 10
        T = self.robot.GetTransform()
        for xpp, ypp in zip(numpy.linspace(start_config[0], end_config[0], no_samples), numpy.linspace(start_config[1], end_config[1], no_samples)):
            T_new = T
            T_new[0][3] = xpp
            T_new[1][3] = ypp
            self.robot.SetTransform(T_new)
            if self.robot.GetEnv().CheckCollision(self.robot, self.table):
                return True
        return False

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)
        control_set = numpy.array([[1, 1, 0.1], [0.5, 1, 0.1], [1, 0.5, 0.1]])

        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = numpy.array(self.discrete_env.GridCoordToConfiguration(grid_coordinate))

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #
            for i_c in range(len(control_set)):
                omega_left = control_set[i_c, 0]
                omega_right = control_set[i_c, 1]
                dur = control_set[i_c, 2]
                control_obj = Control(omega_left, omega_right, dur)
                footprint = self.GenerateFootprintFromControl(start_config, control_obj)
                action_obj = Action(control_obj, footprint)
                self.actions[idx].append(action_obj)         
            

    def GetSuccessors(self, node_id):
        node_grid_coord = self.discrete_env.NodeIdToGridCoord(node_id)
        orientation_id = node_grid_coord[2]
        applicable_actions = self.actions[orientation_id]
        successor_actions = dict()
        successors = dict()

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        for idx_a in range(len(applicable_actions)):
            successor_actions[idx_a] = applicable_actions[idx_a]
            control_obj = applicable_actions[idx_a].control
            footprints = applicable_actions[idx_a].footprint
            list_s = []
            for idx_fp in range(len(footprints)):
                fp_node_id = self.discrete_env.ConfigurationToNodeId(footprints[idx_fp])
                list_s.append(fp_node_id)
            successors[idx_a] = list_s
        return successors, successor_actions

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        dist = numpy.linalg.norm(numpy.array(end_config) - numpy.array(start_config))
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        cost = numpy.linalg.norm(numpy.array(end_config) - numpy.array(start_config))
        return cost
