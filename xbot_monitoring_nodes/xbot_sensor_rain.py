#!/usr/bin/env python

# get rain sensor value and publish to sensor topic

import rospy
from xbot_msgs.msg import SensorInfo
from xbot_msgs.msg import SensorDataString
from mower_msgs.msg import Status

my_info = SensorInfo()
sensor_data_publisher = None
mean_value = None

# publish sensor data
def sensor_publish():
    global sensor_data_publisher, mean_value
    rospy.init_node('xbot_sensor_rain', anonymous=True)

    my_info.has_critical_high = False
    my_info.has_critical_low = False
    my_info.has_min_max = False
    # my_info.min_value = 
    # my_info.max_value = 
    my_info.value_type = SensorInfo.TYPE_STRING
    # my_info.value_description = SensorInfo.VALUE_DESCRIPTION_TEMPERATURE
    my_info.unit = ""
    my_info.sensor_id = "rain"
    my_info.sensor_name = "rain sensor"

    sensor_info_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/info", SensorInfo, queue_size=1, latch=True)  
    sensor_data_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/data", SensorDataString, queue_size=1)
    sensor_info_publisher.publish(my_info)

    rospy.Subscriber("/mower/status", Status, callback_sensor)

    data = SensorDataString()

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        if mean_value != None:
            data.stamp = rospy.Time.now()
            data.data = str(mean_value)
            
            # print("sensor: ", data.data, " ", type(data.data))

            # Publish
            sensor_data_publisher.publish(data)
            mean_value = None
        rate.sleep()


# read, store value
def callback_sensor(msg):
    global mean_value
    if mean_value == None:
        mean_value = msg.rain_detected
        return
    else:
        mean_value = (mean_value | msg.rain_detected)


if __name__ == '__main__':
    try:
        print("Monitoring rain sensor")
        sensor_publish()
    except rospy.ROSInterruptException:
        pass