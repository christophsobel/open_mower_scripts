#!/usr/bin/env python

#===========================================
# Christoph Sobel 22.06.2023
#
# get imu z acceleration, parse and publish to sensor topic
#
#===========================================

import rospy
from xbot_msgs.msg import SensorInfo
from xbot_msgs.msg import SensorDataDouble
from sensor_msgs.msg import Imu

my_info = SensorInfo()
sensor_data_publisher = None
max_z = None

# publish sensor data
def sensor_publish():
    global sensor_data_publisher, max_z
    rospy.init_node('xbot_sensor_acceleration', anonymous=True)

    my_info.has_critical_high = False
    my_info.has_critical_low = False
    my_info.has_min_max = False
    # my_info.min_value = 
    # my_info.max_value = 
    my_info.value_type = SensorInfo.TYPE_DOUBLE
    my_info.value_description = SensorInfo.VALUE_DESCRIPTION_TEMPERATURE
    my_info.unit = "m/s²"
    my_info.sensor_id = "acceleration_z"
    my_info.sensor_name = "acceleration in Z"

    sensor_info_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/info", SensorInfo, queue_size=1, latch=True)  
    sensor_data_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/data", SensorDataDouble, queue_size=1)
    sensor_info_publisher.publish(my_info)

    rospy.Subscriber("/imu/data_raw", Imu, callback_imu)

    data = SensorDataDouble()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if max_z != None:
            data.stamp = rospy.Time.now()
            data.data = max_z
            
            # print("Max Z: ", data.data, " ", type(data.data))

            # Publish
            sensor_data_publisher.publish(data)
            max_z = None
        rate.sleep()


# read IMU, store max value
def callback_imu(imu_msg):
    global max_z
    # print(log_msg.level)
    if max_z == None:
        max_z = imu_msg.linear_acceleration.z
        return
    if imu_msg.linear_acceleration.z > max_z:
        max_z = imu_msg.linear_acceleration.z


if __name__ == '__main__':
    try:
        print("Monitoring IMU")
        sensor_publish()
    except rospy.ROSInterruptException:
        pass