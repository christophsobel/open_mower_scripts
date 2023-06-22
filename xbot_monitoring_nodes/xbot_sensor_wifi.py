#!/usr/bin/env python

#===========================================
# Christoph Sobel 22.06.2023
#
# get wifi signal strenght by parsing "iw wlan link" and publish to sensor topic
#
#===========================================

import rospy
from xbot_msgs.msg import SensorInfo
from xbot_msgs.msg import SensorDataDouble
import subprocess

my_info = SensorInfo()

# publish sensor data
def sensor_publish():
    rospy.init_node('xbot_sensor_wifi', anonymous=True)

    my_info.has_critical_high = False
    my_info.has_critical_low = False
    my_info.has_min_max = True
    my_info.min_value = -99
    my_info.max_value = 0
    my_info.value_type = SensorInfo.TYPE_DOUBLE
    my_info.value_description = SensorInfo.VALUE_DESCRIPTION_TEMPERATURE
    my_info.unit = "dBm"
    my_info.sensor_id = "wifi_strength"
    my_info.sensor_name = "wifi strength"

    sensor_info_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/info", SensorInfo, queue_size=1, latch=True)  
    sensor_data_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/data", SensorDataDouble, queue_size=1)
    sensor_info_publisher.publish(my_info)

    data = SensorDataDouble()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        data.stamp = rospy.Time.now()
        data.data = float(get_wifi_info())
        
        #print(data.data)

        # Publish
        sensor_data_publisher.publish(data)
        rate.sleep()


# read "iw wlan0 link" and parse
def get_wifi_info():
    wifi = subprocess.run(["iw", "wlan0" ,"link"], capture_output=True)
    #print(wifi.stdout.decode("utf-8"))
    for line in wifi.stdout.decode("utf-8").replace("\t","").split("\n"):
        if "signal: " in line:
            #print(line[8:11])
            return line[8:11]


if __name__ == '__main__':
    try:
        print("Monitoring wifi signal strength")
        sensor_publish()
    except rospy.ROSInterruptException:
        pass