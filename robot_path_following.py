#!/usr/bin/env python
#Ubey Ali, 150130912
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf
from tf import transformations
import math
waypoint=None
temp=0
waypoint_theta_temp=1
#waypoint callback
def waypoint_callback(msg): #  callback

    #***************************************
    #***          Obtain current destination
    #***************************************

    #save waypoint data for printing out in main loop
    global waypoint
    waypoint=msg;


if __name__ == '__main__':

    #setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    rospy.init_node("crazy_driver_456")
    waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, waypoint_callback) # <--- set up callback
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    listener = tf.TransformListener()

    #start a loop; one loop per second
    delay = rospy.Rate(1.0); # perhaps this could be faster for a controller?
    while not rospy.is_shutdown():


        #***************************************
        #***          Obtain current robot pose
        #***************************************
        
        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            
            (translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        

        #***************************************
        #***          Print current robot pose
        #***************************************

        #Print out the x,y coordinates of the transform
        print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")

        #Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        print("Robot is believed to have orientation (theta): (",robot_theta,")\n")

        #***************************************
        #***          Print current destination
        #***************************************

        # the waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        # subscribed to in the .Subscriber call above.

        #Print out the x,y coordinates of the latest message
        print("Current waypoint (x,y): (",waypoint.translation.x,",",waypoint.translation.y,")")

        #Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        waypointrotq = [waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w]
        w_xorient, w_yorient, w_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(waypointrotq))
        waypoint_theta=w_zorient # only need the z axis
        print("Current waypoint (theta): (",waypoint_theta,")\n")

        #***************************************
        #***          DRIVE THE ROBOT HERE (same as with assignment 1 
        #**           - except you are driving towards a goal not away from an obstacle)
        #***************************************
        #for containing the motor commands to send to the robot
        motor_command=Twist()
       # motor_command.linear.x=5000.0;
       # motor_command.angular.z=0.0;
	x=translation[0]		#x,y coordinates of the transform
	y=translation[1]
	cos_robot_theta=math.cos(robot_theta)
	sin_robot_theta=math.sin(robot_theta)
        xr =  cos_robot_theta * (waypoint.translation.x - x) + sin_robot_theta * (waypoint.translation.y - y)
        yr =  cos_robot_theta * (waypoint.translation.y - y) - sin_robot_theta * (waypoint.translation.x - x)
        dwr = math.atan2(yr, xr) # difference between waypoint and robot rotational pointing dir
	#print("Current waypoint (theta): (",waypoint_theta,waypoint_theta_temp,")\n")
	if waypoint_theta_temp - waypoint_theta == 0 :
		motor_command.linear.x=0.7 * math.hypot(xr, yr)
		motor_command.angular.z=0.7 * dwr + 0.7 * (dwr-temp)
	else:
		motor_command.linear.x=0
		motor_command.angular.z=0
	waypoint_theta_temp=waypoint_theta
	temp = dwr	# to use it in the next step
        motor_command_publisher.publish(motor_command)
        delay.sleep()

    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")



















