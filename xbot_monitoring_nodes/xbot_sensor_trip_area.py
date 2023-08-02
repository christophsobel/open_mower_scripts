#!/usr/bin/env python

# get position, calculate trip, area and publish to sensor topic

import os
import rospy
import math
from xbot_msgs.msg import SensorInfo
from xbot_msgs.msg import SensorDataDouble
from xbot_msgs.msg import AbsolutePose
from mower_msgs.msg import HighLevelStatus

trip_info = SensorInfo()
trip = 0.0

area_info = SensorInfo()
area = 0.0

OM_TOOL_WIDTH = float(os.getenv('OM_TOOL_WIDTH'))

last_x = None
last_y = None

last_status = None


# publish sensor data
def sensor_publish():
    global trip, area
    rospy.init_node('xbot_sensor_trip_area', anonymous=True)

    # trip data
    trip_info.has_critical_high = False
    trip_info.has_critical_low = False
    trip_info.has_min_max = False
    # trip_info.min_value = 
    # trip_info.max_value = 
    trip_info.value_type = SensorInfo.TYPE_DOUBLE
    trip_info.value_description = SensorInfo.VALUE_DESCRIPTION_TEMPERATURE
    trip_info.unit = "m"
    trip_info.sensor_id = "trip"
    trip_info.sensor_name = "tip since undock"
    trip_sensor_info_publisher = rospy.Publisher("xbot_monitoring/sensors/" + trip_info.sensor_id + "/info", SensorInfo, queue_size=1, latch=True)  
    trip_sensor_data_publisher = rospy.Publisher("xbot_monitoring/sensors/" + trip_info.sensor_id + "/data", SensorDataDouble, queue_size=1)
    trip_sensor_info_publisher.publish(trip_info)
    trip_data = SensorDataDouble()

    # area data
    area_info.has_critical_high = False
    area_info.has_critical_low = False
    area_info.has_min_max = False
    # area_info.min_value = 
    # area_info.max_value = 
    area_info.value_type = SensorInfo.TYPE_DOUBLE
    area_info.value_description = SensorInfo.VALUE_DESCRIPTION_TEMPERATURE
    area_info.unit = "m²"
    area_info.sensor_id = "area"
    area_info.sensor_name = "area since undock"
    area_sensor_info_publisher = rospy.Publisher("xbot_monitoring/sensors/" + area_info.sensor_id + "/info", SensorInfo, queue_size=1, latch=True)  
    area_sensor_data_publisher = rospy.Publisher("xbot_monitoring/sensors/" + area_info.sensor_id + "/data", SensorDataDouble, queue_size=1)
    area_sensor_info_publisher.publish(area_info)
    area_data = SensorDataDouble()

    rospy.Subscriber("/xbot_positioning/xb_pose", AbsolutePose, callback_pose)
    rospy.Subscriber("/mower_logic/current_state", HighLevelStatus, callback_status)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        trip_data.stamp = rospy.Time.now()
        trip_data.data = trip

        area_data.stamp = rospy.Time.now()
        area_data.data = area
            
        print("trip: ", trip_data.data, " ", type(trip_data.data))
        print("area: ", area_data.data, " ", type(area_data.data))

        # Publish
        trip_sensor_data_publisher.publish(trip_data)
        area_sensor_data_publisher.publish(area_data)
        rate.sleep()


# read data, calculate delta value
def callback_pose(msg):
    global last_x, last_y, trip, area, last_status
    if last_x != None and last_y != None and last_status == "MOWING":
        dx = msg.pose.pose.position.x - last_x
        dy = msg.pose.pose.position.y - last_y
        trip = trip + math.sqrt(dx*dx + dy*dy)
        area = trip * OM_TOOL_WIDTH
    last_x = msg.pose.pose.position.x
    last_y = msg.pose.pose.position.y


def callback_status(msg):
    global last_status, trip, area
    if last_status == "UNDOCKING" and msg.state_name == "MOWING":
        trip = 0
        area = 0
    last_status = msg.state_name

if __name__ == '__main__':
    try:
        print("Monitoring position and state")
        sensor_publish()
    except rospy.ROSInterruptException:
        pass