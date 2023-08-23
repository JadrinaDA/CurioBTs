#!/usr/bin/env python

##############################################################################
# Documentation
##############################################################################
"""
ROS node that listens for obstacle information and the position of the robot
and then publishes if there is an obstacle in the path based on the distances
calculated.
"""
##############################################################################
# Imports
##############################################################################


import rospy
from std_msgs.msg import Bool
from obstacle_detector.msg import Obstacles, SegmentObstacle
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Path
import math
import time

# Publisher to signal to behavior tree is path is clear or not
pub = rospy.Publisher('/pathclear', Bool, queue_size=10)
# Boolean to toggle
obstacle_in_front = False
# Global variable to store robot position
pos = (0,0)
# Global variable to store next point on local path
next = None
# Stores last 5 positions
poses = []
# Obstacle
obstacle = (0,0)


# Callback function called when obstacle data is received
def callback(data):
    global obstacle_in_front
    global pos
    global next
    global poses

    segments = data.segments
    min_d = 1000
    # For each segment detected:
    posmin = None
    for segment in segments:
        # Calculate middle point and get 1st and last point
        x = (segment.first_point.x + segment.last_point.x) / 2
        y = (segment.first_point.y + segment.last_point.y) / 2
        x1 = segment.first_point.x 
        x2 = segment.last_point.x
        y1 = segment.first_point.y 
        y2 = segment.last_point.y
        dist1 = math.sqrt((x-pos[0])**2 + (y-pos[1])**2)
        dist2 = math.sqrt((x1-pos[0])**2 + (y1-pos[1])**2)
        dist3 = math.sqrt((x2-pos[0])**2 + (y2-pos[1])**2)
        # Choose the point closest to the robot
        dist = min(dist1, dist2, dist3)
        if dist == dist2:
            x,y = x1, y1
        if dist == dist3:
            x,y = x2, y2
        # If we have a path then get the angle between the path and vector from point to robot
        if next:
            result = math.atan2(y - pos[1], x - pos[0]) - math.atan2(next.y - pos[1], next.x - pos[0])
        else:
            result = 10
        #rospy.loginfo((x, y))
        #rospy.loginfo(pos)
        # If distance is lower than our current minimum and obstacle is close to path save it
        if dist < min_d and abs(result) < 0.7:
            min_d = dist
            posmin = (x, y)
    old = obstacle_in_front
    obstacle_in_front = False 
    # We set a threshold to see if there is an obstacle in our path
    if len(poses) > 0:
        rospy.loginfo("x")
        rospy.loginfo(min_d)
        # We also check if we havent moved a lot in some time
        dist_r = time.time() - poses[len(poses)-1]
        #rospy.loginfo(dist_r)
        rospy.loginfo(posmin)
        # If an obstacle is close and we havent moved much we are stuck
        if min_d < 1.1 and dist_r > 8:
            obstacle_in_front = True
    # Only print when there is a change
    if (old != obstacle_in_front):
        rospy.loginfo(obstacle_in_front)
    

def callbackg(data):
    # Callback when we receive robot position
    global pos
    global poses
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    # We also want to get the time we got this position
    # this will help determine we are stuck
    if pos != (0,0):
        if len(poses) == 3:
            poses = poses[1:]
        poses.append(time.time())
    pos = (x, y)

def get_next(data):
    # Callback when we receive path, take the second point on the path
    global next
    global pos
    global poses
    next = data.poses[1].pose.position
    
def listener():
    global obstacle_in_front

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    
    # Set up our three subscribers and their callbacks
    rospy.Subscriber("/raw_obstacles", Obstacles, callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callbackg)
    #rospy.Subscriber("/move_base/TrajectoryPlannerROS/local_plan", Path, get_next)
    rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, get_next)
    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        pub.publish(obstacle_in_front)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
