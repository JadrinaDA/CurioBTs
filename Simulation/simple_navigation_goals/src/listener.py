#!/usr/bin/env python
import rospy
import ros_numpy
import numpy
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

dir = "none"
pub = rospy.Publisher('/pathclear', Bool, queue_size=10)
obstacle_in_front = False
global_cm = None


def callback(data):
    global dir
    global obstacle_in_front
    #global global_cm
    row = ros_numpy.occupancy_grid.occupancygrid_to_numpy(data)
    #if (not numpy.array_equal(row, global_cm)):
    #    diffgl = True
    center_grid = numpy.zeros((1,40))
    if dir[0] == "down":
        row = row[row.shape[0]//2 - 40 :row.shape[0]//2]
    else:
        row = row[row.shape[0]//2:row.shape[0]//2 + 40]
    for r in row:
        if (dir == "left"):
            center_grid = numpy.append(center_grid, [r[:row.shape[1]//2 - 40 :row.shape[1]//2].data], axis=0)
        else:
            center_grid = numpy.append(center_grid, [r[row.shape[1]//2:row.shape[1]//2 + 40].data], axis=0)
    obstacle_in_front = False
    rospy.loginfo(center_grid.sum(axis=0))
    if (center_grid.sum(axis=0).max() > 1900):
        obstacle_in_front = True
    rospy.loginfo(obstacle_in_front)

def get_dir(data):
    global dir
    first = data.poses[0].pose.position
    second = data.poses[len(data.poses) - 1].pose.position
    dir = ["", ""]
    if first.x > second.x:
        dir[0] = "up"
    else:
        dir[0] = "down"
    if first.y > second.y:
        dir[1] = "left"
    else:
        dir[1] = "right"
    rospy.loginfo(dir)

def callbackg(data):
    global global_cm
    global_cm = ros_numpy.occupancy_grid.occupancygrid_to_numpy(data)
    rospy.loginfo(global_cm.shape)
    
def listener():
    global obstacle_in_front

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    
    pub.publish(obstacle_in_front)

    rospy.Subscriber("move_base/local_costmap/costmap", OccupancyGrid, callback)
    #rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid, callbackg)
    rospy.Subscriber("/move_base/TrajectoryPlannerROS/local_plan", Path, get_dir)

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
