import numpy
import time
import math

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - self.lower_limits[idx])/resolution[idx])

    def ConfigurationToNodeId(self, config):

        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))
        return node_id

    def NodeIdToConfiguration(self, nid):

        # TODO: Done
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        return config

    def ConfigurationToGridCoord(self, config):

        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        # coord = [0] * self.dimension

        # Check around
        coord = [numpy.floor((i - self.lower_limits[index])/self.resolution[index]) for index, i in enumerate(config)]
        return coord

    def GridCoordToConfiguration(self, coord):

        # TODO:
        # This function maps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        # config = [0] * self.dimension
        config = [(i+1/2.)*self.resolution[index] + self.lower_limits[index] for index, i in enumerate(coord)]
        return config

    def GridCoordToNodeId(self, coord):

        # TODO:
        # This function maps a grid coordinate to the associated
        # node id

        node_id = coord[0] + sum([reduce(lambda x, y: x * y, self.num_cells[:i], 1)*coord[i] for i in range(1, len(coord))])
        return node_id

    def NodeIdToGridCoord(self, node_id):

        # TODO:
        # This function maps a node id to the associated
        # grid coordinate

        coord = []
        divider = 1
        remainder = node_id
        for i in range(1, len(self.num_cells))[::-1]:
            divider = reduce(lambda x, y: x * y, self.num_cells[:i], 1)
            coord.append(math.floor(remainder/divider))
            remainder = remainder % divider
        coord.append(int(remainder))
        coord = coord[::-1]
        return coord