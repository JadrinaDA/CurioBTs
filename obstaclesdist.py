#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from obstacle_detector.msg import Obstacles, SegmentObstacle
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path
import math

pub = rospy.Publisher('/pathclear', Bool, queue_size=10)
obstacle_in_front = False
pos = (0,0)
next = None


def callback(data):
    global obstacle_in_front
    global pos
    global next

    segments = data.segments
    min = 1000
    for segment in segments:
        x = (segment.first_point.x + segment.last_point.x) / 2
        y = (segment.first_point.y + segment.last_point.y) / 2
        dist = math.sqrt((x-pos[0])**2 + (y-pos[1])**2)
        if next:
            result = math.atan2(y - pos[1], x - pos[0]) - math.atan2(next.y - pos[1], next.x - pos[0])
        else:
            result = 10
        rospy.loginfo(str(result)+"x")
        if dist < min and (abs(result) < math.pi /8):
            min = dist
    obstacle_in_front = False
    

    if min < 1.2:
        obstacle_in_front = True
    rospy.loginfo(obstacle_in_front)
    

def callbackg(data):
    global pos
    pos = (data.polygon.points[3].x, data.polygon.points[3].y)

def get_next(data):
    global next
    next = data.poses[0].pose.position
    
def listener():
    global obstacle_in_front

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    

    rospy.Subscriber("/raw_obstacles", Obstacles, callback)
    rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, callbackg)
    rospy.Subscriber("/move_base/TrajectoryPlannerROS/local_plan", Path, get_next)
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
