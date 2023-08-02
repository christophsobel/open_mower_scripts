#!/usr/bin/env python

# get z height, parse and publish to sensor topic

import rospy
from xbot_msgs.msg import SensorInfo
from xbot_msgs.msg import SensorDataDouble
from geometry_msgs.msg import PoseWithCovariance

my_info = SensorInfo()
sensor_data_publisher = None
mean_z = None

# publish sensor data
def sensor_publish():
    global sensor_data_publisher, mean_z
    rospy.init_node('xbot_sensor_z_height', anonymous=True)

    my_info.has_critical_high = False
    my_info.has_critical_low = False
    my_info.has_min_max = False
    # my_info.min_value = 
    # my_info.max_value = 
    my_info.value_type = SensorInfo.TYPE_DOUBLE
    my_info.value_description = SensorInfo.VALUE_DESCRIPTION_TEMPERATURE
    my_info.unit = "m"
    my_info.sensor_id = "z_height"
    my_info.sensor_name = "position in Z"

    sensor_info_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/info", SensorInfo, queue_size=1, latch=True)  
    sensor_data_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/data", SensorDataDouble, queue_size=1)
    sensor_info_publisher.publish(my_info)

    rospy.Subscriber("/xbot_driver_gps/pose", PoseWithCovariance, callback_pose)

    data = SensorDataDouble()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if mean_z != None:
            data.stamp = rospy.Time.now()
            data.data = mean_z
            
            # print("Z: ", data.data, " ", type(data.data))

            # Publish
            sensor_data_publisher.publish(data)
            mean_z = None
        rate.sleep()


# read IMU, store max value
def callback_pose(msg):
    global mean_z
    if mean_z == None:
        mean_z = msg.pose.position.z
        return
    else:
        mean_z = (mean_z + msg.pose.position.z) * 0.5


if __name__ == '__main__':
    try:
        print("Monitoring position")
        sensor_publish()
    except rospy.ROSInterruptException:
        pass