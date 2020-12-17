#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf
import math
import numpy as np
from scipy.spatial.transform import Rotation

# in this script, the quat is formatted as [x, y, z, w]! We need it like that for Unlike the Madgwick implementation!
# dq = 1/2 * q * w:
def qDot(q, w):
    q_dot = np.array([0.0, 0.0, 0.0, 1.0])

    q_dot[0] = 0.5 * ( q[3] * w[0] + q[1] * w[2] - q[2] * w[1])    # x!!
    q_dot[1] = 0.5 * ( q[3] * w[1] - q[0] * w[2] + q[2] * w[0])    # y!!
    q_dot[2] = 0.5 * ( q[3] * w[2] + q[0] * w[1] - q[1] * w[0])    # z!!
    q_dot[3] = 0.5 * (-q[0] * w[0] - q[1] * w[1] - q[2] * w[2])    # w!!
    return q_dot


class RotationViz():

    def __init__(self):
        rospy.init_node('rotation_viz_node', anonymous=False)
        rospy.on_shutdown(self.shutdowni_callback)
        markerPub = rospy.Publisher('rocket_marker_topic', Marker, queue_size=10)
        tf_broadcaster = tf.TransformBroadcaster()

        self.start_time = rospy.Time.now()

        # parameters
        rocket_frame_id = rospy.get_param('~rocket_frame_id', 'rotating_frame')
        rocket_parent_frame_id = rospy.get_param('~rocket_parent_frame_id', 'world')


        refresh_rate = 25.0             # also the computation period
        w = np.array([[1], [1], [1]])   # angular rate w (rad/s)

        # markers
        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = rocket_frame_id

        self.robotMarker.header.stamp = rospy.get_rostime()
        self.robotMarker.ns = "rot_viz_marker"
        self.robotMarker.id = 0
        self.robotMarker.type = 1  # cube
        self.robotMarker.action = 0
        self.robotMarker.pose.position.x = 0.0
        self.robotMarker.pose.position.y = 0.0
        self.robotMarker.pose.position.z = 0.0

        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = 1.0
        self.robotMarker.scale.y = 0.6
        self.robotMarker.scale.z = 0.2

        self.robotMarker.color.r = 0.0
        self.robotMarker.color.g = 0.0
        self.robotMarker.color.b = 1.0
        self.robotMarker.color.a = 1.0

        self.robotMarker.lifetime = rospy.Duration(0.0)
        self.robotMarker.frame_locked = True
        #####################################################################
        
        self.robotAxis = Marker()
        self.robotAxis.header.frame_id = rocket_frame_id

        self.robotAxis.header.stamp = rospy.get_rostime()
        self.robotAxis.ns = "rot_viz_marker"
        self.robotAxis.id = 1
        self.robotAxis.type = 0  # cube
        self.robotAxis.action = 0
        self.robotAxis.pose.position.x = 0.0
        self.robotAxis.pose.position.y = 0.0
        self.robotAxis.pose.position.z = 0.0

        self.robotAxis.pose.orientation.x = 0
        self.robotAxis.pose.orientation.y = 0
        self.robotAxis.pose.orientation.z = 0
        self.robotAxis.pose.orientation.w = 1.0
        self.robotAxis.scale.x = 0.05
        self.robotAxis.scale.y = 0.1
        self.robotAxis.scale.z = 0.1

        self.robotAxis.color.r = 238.0/255.0
        self.robotAxis.color.g = 130.0/255.0
        self.robotAxis.color.b = 238.0/255.0
        self.robotAxis.color.a = 1.0

        axis_start_point = Point()
        axis_start_point.x = 0.0
        axis_start_point.y = 0.0
        axis_start_point.z = 0.0

        axis_end_point = Point()
        axis_end_point.x = w[0]
        axis_end_point.y = w[1]
        axis_end_point.z = w[2]
        
        self.robotAxis.points.append(axis_start_point)
        self.robotAxis.points.append(axis_end_point)

        self.robotAxis.lifetime = rospy.Duration(0.0)
        self.robotAxis.frame_locked = True
        
        ##########################################################################
        self.robotXAxis = Marker()
        self.robotXAxis.header.frame_id = rocket_frame_id

        self.robotXAxis.header.stamp = rospy.get_rostime()
        self.robotXAxis.ns = "rot_viz_marker"
        self.robotXAxis.id = 2
        self.robotXAxis.type = 0  # arrow
        self.robotXAxis.action = 0
        self.robotXAxis.pose.position.x = 0.0
        self.robotXAxis.pose.position.y = 0.0
        self.robotXAxis.pose.position.z = 0.0

        self.robotXAxis.pose.orientation.x = 0
        self.robotXAxis.pose.orientation.y = 0
        self.robotXAxis.pose.orientation.z = 0
        self.robotXAxis.pose.orientation.w = 1.0
        self.robotXAxis.scale.x = 0.05
        self.robotXAxis.scale.y = 0.1
        self.robotXAxis.scale.z = 0.1

        self.robotXAxis.color.r = 1.0
        self.robotXAxis.color.g = 0.0
        self.robotXAxis.color.b = 0.0
        self.robotXAxis.color.a = 1.0

        XAxis_start_point = Point()
        XAxis_start_point.x = 0.0
        XAxis_start_point.y = 0.0
        XAxis_start_point.z = 0.0

        XAxis_end_point = Point()
        XAxis_end_point.x = 1.0
        XAxis_end_point.y = 0.0
        XAxis_end_point.z = 0.0

        self.robotXAxis.points.append(XAxis_start_point)
        self.robotXAxis.points.append(XAxis_end_point)

        self.robotXAxis.lifetime = rospy.Duration(0.0)
        self.robotXAxis.frame_locked = True
        
        #############################################################################
        self.robotYAxis = Marker()
        self.robotYAxis.header.frame_id = rocket_frame_id

        self.robotYAxis.header.stamp = rospy.get_rostime()
        self.robotYAxis.ns = "rot_viz_marker"
        self.robotYAxis.id = 3
        self.robotYAxis.type = 0  # arrow
        self.robotYAxis.action = 0
        self.robotYAxis.pose.position.x = 0.0
        self.robotYAxis.pose.position.y = 0.0
        self.robotYAxis.pose.position.z = 0.0

        self.robotYAxis.pose.orientation.x = 0
        self.robotYAxis.pose.orientation.y = 0
        self.robotYAxis.pose.orientation.z = 0
        self.robotYAxis.pose.orientation.w = 1.0
        self.robotYAxis.scale.x = 0.05
        self.robotYAxis.scale.y = 0.1
        self.robotYAxis.scale.z = 0.1

        self.robotYAxis.color.r = 0.0
        self.robotYAxis.color.g = 1.0
        self.robotYAxis.color.b = 0.0
        self.robotYAxis.color.a = 1.0

        YAxis_start_point = Point()
        YAxis_start_point.x = 0.0
        YAxis_start_point.y = 0.0
        YAxis_start_point.z = 0.0

        YAxis_end_point = Point()
        YAxis_end_point.x = 0.0
        YAxis_end_point.y = 1.0
        YAxis_end_point.z = 0.0

        self.robotYAxis.points.append(YAxis_start_point)
        self.robotYAxis.points.append(YAxis_end_point)

        self.robotYAxis.lifetime = rospy.Duration(0.0)
        self.robotYAxis.frame_locked = True
        
        #################################################################################
        self.robotZAxis = Marker()
        self.robotZAxis.header.frame_id = rocket_frame_id

        self.robotZAxis.header.stamp = rospy.get_rostime()
        self.robotZAxis.ns = "rot_viz_marker"
        self.robotZAxis.id = 4
        self.robotZAxis.type = 0  # arrow
        self.robotZAxis.action = 0
        self.robotZAxis.pose.position.x = 0.0
        self.robotZAxis.pose.position.y = 0.0
        self.robotZAxis.pose.position.z = 0.0

        self.robotZAxis.pose.orientation.x = 0
        self.robotZAxis.pose.orientation.y = 0
        self.robotZAxis.pose.orientation.z = 0
        self.robotZAxis.pose.orientation.w = 1.0
        self.robotZAxis.scale.x = 0.05
        self.robotZAxis.scale.y = 0.1
        self.robotZAxis.scale.z = 0.1

        self.robotZAxis.color.r = 0.0
        self.robotZAxis.color.g = 0.0
        self.robotZAxis.color.b = 1.0
        self.robotZAxis.color.a = 1.0

        ZAxis_start_point = Point()
        ZAxis_start_point.x = 0.0
        ZAxis_start_point.y = 0.0
        ZAxis_start_point.z = 0.0

        ZAxis_end_point = Point()
        ZAxis_end_point.x = 0.0
        ZAxis_end_point.y = 0.0
        ZAxis_end_point.z = 1.0

        self.robotZAxis.points.append(ZAxis_start_point)
        self.robotZAxis.points.append(ZAxis_end_point)

        self.robotZAxis.lifetime = rospy.Duration(0.0)
        self.robotZAxis.frame_locked = True

        #################################################################################

        self.robotText = Marker()
        self.robotText.header.frame_id = rocket_parent_frame_id

        self.robotText.header.stamp = rospy.get_rostime()
        self.robotText.ns = "rot_viz_marker"
        self.robotText.id = 10
        self.robotText.type = 9  # text
        self.robotText.action = 0
        self.robotText.pose.position.x = 0.0
        self.robotText.pose.position.y = 0.0
        self.robotText.pose.position.z = 4.0

        self.robotText.pose.orientation.x = 0
        self.robotText.pose.orientation.y = 0
        self.robotText.pose.orientation.z = 0
        self.robotText.pose.orientation.w = 1.0
        self.robotText.scale.z = 0.5

        self.robotText.color.r = 1.0
        self.robotText.color.g = 1.0
        self.robotText.color.b = 1.0
        self.robotText.color.a = 1.0
        self.robotText.text = "Ouch!"

        self.robotText.lifetime = rospy.Duration(0.0)
        self.robotText.frame_locked = True



        rate = rospy.Rate(refresh_rate)
        dT = 1.0 / refresh_rate
        q_current = np.array([0.0, 0.0, 0.0, 1.0])
        

        while not rospy.is_shutdown():

            q_current += dT * qDot(q_current, w)
            att_now = Rotation.from_quat(q_current)
            eul_now = att_now.as_euler('zyx', degrees=True)
            eul_string = "(deg) Yaw: " + str(round(eul_now[0],1)) + " Pitch: " + str(round(eul_now[1],1)) + " Roll: " + str(round(eul_now[2],1))

            self.robotMarker.header.stamp = rospy.get_rostime()
            markerPub.publish(self.robotMarker)

            self.robotAxis.header.stamp = rospy.get_rostime()
            markerPub.publish(self.robotAxis)

            self.robotXAxis.header.stamp = rospy.get_rostime()
            markerPub.publish(self.robotXAxis)
            self.robotYAxis.header.stamp = rospy.get_rostime()
            markerPub.publish(self.robotYAxis)
            self.robotZAxis.header.stamp = rospy.get_rostime()
            markerPub.publish(self.robotZAxis)

            self.robotText.header.stamp = rospy.get_rostime()
            self.robotText.text = eul_string
            markerPub.publish(self.robotText)


            tf_broadcaster.sendTransform((0.0, 0.0, 0.0),
                                         att_now.as_quat(),
                                         rospy.Time.now(),
                                         rocket_frame_id,
                                         rocket_parent_frame_id)

            rate.sleep()


    def shutdowni_callback(self):
        print("Shutting down")


if __name__ == '__main__':
    try:
        RotationViz()
    except rospy.ROSInterruptException:
        pass

