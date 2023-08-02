#!/usr/bin/env python

# get mow motor tacho rpm value, parse and publish to sensor topic

import rospy
from xbot_msgs.msg import SensorInfo
from xbot_msgs.msg import SensorDataDouble
from mower_msgs.msg import Status

my_info = SensorInfo()
sensor_data_publisher = None
mean_value = None

# publish sensor data
def sensor_publish():
    global sensor_data_publisher, mean_value
    rospy.init_node('xbot_sensor_mow_tacho', anonymous=True)

    my_info.has_critical_high = False
    my_info.has_critical_low = False
    my_info.has_min_max = False
    # my_info.min_value = 
    # my_info.max_value = 
    my_info.value_type = SensorInfo.TYPE_DOUBLE
    my_info.value_description = SensorInfo.VALUE_DESCRIPTION_TEMPERATURE
    my_info.unit = "rpm"
    my_info.sensor_id = "mow_tacho"
    my_info.sensor_name = "mow motor tacho"

    sensor_info_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/info", SensorInfo, queue_size=1, latch=True)  
    sensor_data_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/data", SensorDataDouble, queue_size=1)
    sensor_info_publisher.publish(my_info)

    rospy.Subscriber("/mower/status", Status, callback_sensor)

    data = SensorDataDouble()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if mean_value != None:
            data.stamp = rospy.Time.now()
            data.data = mean_value
            
            # print("sensor: ", data.data, " ", type(data.data))

            # Publish
            sensor_data_publisher.publish(data)
            mean_value = None
        rate.sleep()


# read sensor, store mean value
def callback_sensor(msg):
    global mean_value
    if mean_value == None:
        mean_value = msg.mow_esc_status.tacho
        return
    else:
        mean_value = (mean_value + msg.mow_esc_status.tacho) * 0.5


if __name__ == '__main__':
    try:
        print("Monitoring mow tacho")
        sensor_publish()
    except rospy.ROSInterruptException:
        pass