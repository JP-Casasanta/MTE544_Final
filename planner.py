from mapUtilities import *
from a_star import *
from rrt import *
from rrt_star import *
import time
from utilities import smooth_path, Logger

POINT_PLANNER=0; A_STAR_PLANNER=1; RRT_PLANNER=2; RRT_STAR_PLANNER=3


# TODO Modify this class so that is uses the RRT* planner with virtual obstacles

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        self.costMap=None
        self.initTrajectoryPlanner()
        
        return self.trajectory_planner(startPose, endPose, self.type)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):
        
        #### If using the map, you can leverage on the code below originally implemented for A* (BONUS points option)
        #self.m_utilites=mapManipulator(laser_sig=0.4)    
        #self.costMap=self.m_utilites.make_likelihood_field()

        #TODO Remember to initialize the rrt_star

        """ Trial 1/2
        obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1),
    ]  # [x,y,size(radius)]
    """
        
        #""" Trial 3/4
        obstacle_list = [
        (5, 5, 1),
        (6, 6, 2),
        (6, 8, 2),
        (4, 10, 1),
        (3, 10, 1),
        (2, 10, 1),
        (1, 10, 1),
        (0, 10, 1),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (8, 12, 1),
    ]  # [x,y,size(radius)]
    #"""

        self.rrt_star = RRTStar(
        start=[0, 0],
        goal=[6, 10],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=1,
        robot_radius=0.8,
        max_iter=2000,
        search_until_max_iter=True)

        
    
    def trajectory_planner(self, startPoseCart, endPoseCart, type):
        
        #### If using the map, you can leverage on the code below originally implemented for A* (BONUS points option)
        #### If not using the map (no bonus), you can just call the function in rrt_star with the appropriate arguments and get the returned path
        #### then you can put the necessary measure to bypass the map stuff down here.
        # Map scaling factor (to save planning time)
        scale_factor = 1 # this is the downsample scale, if set 2, it will downsample the map by half, and if set x, it will do the same as 1/x


        #startPose=self.m_utilites.position_2_cell(startPoseCart)
        #endPose=self.m_utilites.position_2_cell(endPoseCart)
        
        startPose = startPoseCart
        endPose = endPoseCart

        start_time = time.time()
        
        startPose = [int(i/scale_factor) for i in startPose]
        endPose   = [int(j/scale_factor) for j in endPose]

        #mazeOrigin = self.m_utilites.position_2_cell([0,0])

        # TODO This is for A*, modify this part to use RRT*
        #path = search(self.costMap, startPose, endPose, scale_factor)
    
        self.rrt_star.start = self.rrt_star.Node(startPoseCart[0],startPoseCart[1])
        self.rrt_star.end = self.rrt_star.Node(endPoseCart[0],endPoseCart[1])
        self.rrt_star.goal_node = self.rrt_star.Node(endPoseCart[0],endPoseCart[1])



        path = self.rrt_star.planning(animation=False)

        end_time = time.time()

        # This will display how much time the search algorithm needed to find a path
        print(f"the time took for a_star calculation was {end_time - start_time}")

        path_ = [[x*scale_factor, y*scale_factor] for x,y in path ]
        pass
        Path = path_
        #Path = np.array(list(map(self.m_utilites.cell_2_position, path_ )))

        # TODO Smooth the path before returning it to the decision maker
        # this can be in form of a function that you can put in the utilities.py 
        # or add it as a method to the original rrt.py 

        path_smooth = smooth_path(Path)

        if path_smooth is not None:
            self.rrt_star.draw_graph()
            #plt.plot(path[0][0], path[0][1], 'x', label="start")
            #plt.plot(path[-1][0], path[-1][1], '+', label="end")
            plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--', label="rough")
            plt.plot([x for (x, y) in path_smooth], [y for (x, y) in path_smooth], 'b--', label="smooth")
            plt.grid(True)
            plt.legend()
            plt.show()

        pathLogger = Logger( "path.csv" , ["x", "y"])
        for item in path_smooth:
            pathLogger.log_values([item[0], item[1]])

        return path_smooth


if __name__=="__main__":

    #m_utilites=mapManipulator()
    
    #map_likelihood=m_utilites.make_likelihood_field()

    #test here

    PLANNER = planner(RRT_STAR_PLANNER)
    path = PLANNER.plan([0,0], [6, 10])


    if path is not None:
        PLANNER.rrt_star.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
        plt.grid(True)
        plt.show()

    pass




