#!/usr/bin/env python

import world as world_mod
import random
import rospy
from randomwalker.srv import GetBounds
from randomwalker.srv import GetScore
from randomwalker.srv import GetBoundsResponse
"""
### Steps for this file

1. Make this into a node called 'map_server' (in main())
2. Add a rospy.spin() to the end of main()
3. Create a service called 'get_bounds', with the implementation in
   `self._get_bounds`
4. Don't forget to import GetBounds.
5. Implement `self._get_bounds` by returning a GetBoundsResponse with the number
   of rows and columns in the world.
6. Create a service called 'get_score', with the implementation in
   `self._get_score`
7. Implement `self._get_score`, by calling self._world.get_score. Get the row
   and col being queried from the request object.
8. (Optional) Check that the queried location is in bounds, and if not, raise a
   rospy.ServiceException.

### Helpful resources

- [ROS tutorial on services](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)
- [Official rospy documentation on services](http://wiki.ros.org/rospy/Overview/Services)
"""

class MapServer(object):
    """Implements a ROS service that gets the score for locations in the world.
    
    Services:
    get_score (GetScore.srv): Given a square location (row, col), returns the
        score of that square.
    """
    def __init__(self, world):
        self._world = world
        # TODO: Create a rospy service called 'get_bounds'. Make _get_bounds the
        # handler.
	rospy.Service('get_bounds', GetBounds, self._get_bounds)
        # TODO: Create a rospy service called 'get_score'. Make _get_score the
        # handler.
	rospy.Service('get_score', GetScore, self._get_score)

    def _get_score(self, request):
        # TODO: (Optional) Check that the location is in bounds, and raise a
        # rospy.ServiceException if not.
	if (request.row >= self._world.num_rows() or request.row < 0 or
            request.col >= self._world.num_cols() or request.col < 0):
	    raise rospy.ServiceException('Row or column is out of bounds.') 
        # TODO: Return the score for the correct row and column.
        return self._world.get_score(request.row, request.col)

    def _get_bounds(self, request):
        # TODO: Return a GetBoundsResponse with the size of the grid.
        return GetBoundsResponse(self._world.num_rows(), self._world.num_cols())

def main():
    # TODO: Make this program into a ROS node called 'map_server' using init_node.
    rospy.init_node('map_server')
    world = world_mod.World(world_mod.NUM_ROWS, world_mod.NUM_COLS)
    random.seed(0)
    world.randomize(random)
    server = MapServer(world)
    # TODO: Call rospy.spin()
    rospy.spin()

if __name__ == '__main__':
    main()
