#!/usr/bin/env python

# get rpi data by parsing cat commands and publish to sensor topic

import rospy
from xbot_msgs.msg import SensorInfo
from xbot_msgs.msg import SensorDataDouble
import subprocess

rpi_temp = SensorInfo()
rpi_freq = SensorInfo()

# publish sensor data
def sensor_publish():
    rospy.init_node('xbot_sensor_rpi', anonymous=True)

    # rpi cpu temperature
    rpi_temp.has_critical_high = False
    rpi_temp.has_critical_low = False
    rpi_temp.has_min_max = True
    rpi_temp.min_value = -99
    rpi_temp.max_value = 150
    rpi_temp.value_type = SensorInfo.TYPE_DOUBLE
    rpi_temp.value_description = SensorInfo.VALUE_DESCRIPTION_TEMPERATURE
    rpi_temp.unit = "°C"
    rpi_temp.sensor_id = "rpi_cpu_temp"
    rpi_temp.sensor_name = "RPi CPU Temp"

    rpi_temp_sensor_info_publisher = rospy.Publisher("xbot_monitoring/sensors/" + rpi_temp.sensor_id + "/info", SensorInfo, queue_size=1, latch=True)  
    rpi_temp_sensor_data_publisher = rospy.Publisher("xbot_monitoring/sensors/" + rpi_temp.sensor_id + "/data", SensorDataDouble, queue_size=1)
    rpi_temp_sensor_info_publisher.publish(rpi_temp)

    rpi_temp_data = SensorDataDouble()

    # rpi cpu frequency
    rpi_freq.has_critical_high = False
    rpi_freq.has_critical_low = False
    rpi_freq.has_min_max = False
    # rpi_freq.min_value = 
    # rpi_freq.max_value = 
    rpi_freq.value_type = SensorInfo.TYPE_DOUBLE
    # rpi_freq.value_description = SensorInfo.VALUE_DESCRIPTION_TEMPERATURE
    rpi_freq.unit = "Hz"
    rpi_freq.sensor_id = "rpi_cpu_freq"
    rpi_freq.sensor_name = "RPi CPU Frequency"

    rpi_freq_sensor_info_publisher = rospy.Publisher("xbot_monitoring/sensors/" + rpi_freq.sensor_id + "/info", SensorInfo, queue_size=1, latch=True)  
    rpi_freq_sensor_data_publisher = rospy.Publisher("xbot_monitoring/sensors/" + rpi_freq.sensor_id + "/data", SensorDataDouble, queue_size=1)
    rpi_freq_sensor_info_publisher.publish(rpi_freq)

    rpi_freq_data = SensorDataDouble()

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        rpi_temp_data.stamp = rospy.Time.now()
        rpi_temp_data.data = float(get_rpi_temp())

        rpi_freq_data.stamp = rospy.Time.now()
        rpi_freq_data.data = float(get_rpi_freq())        
        #print(data.data)

        # Publish
        rpi_temp_sensor_data_publisher.publish(rpi_temp_data)
        rpi_freq_sensor_data_publisher.publish(rpi_freq_data)
        rate.sleep()


# read and parse
def get_rpi_freq():
    res = subprocess.run(["cat", "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"], capture_output=True)
    #print(res.stdout.decode("utf-8"))
    return float(res.stdout.decode("utf-8"))*0.001

def get_rpi_temp():
    res = subprocess.run(["cat", "/sys/class/thermal/thermal_zone0/temp"], capture_output=True)
    #print(res.stdout.decode("utf-8"))
    return float(res.stdout.decode("utf-8"))*0.001    


if __name__ == '__main__':
    try:
        print("Monitoring rpi info")
        sensor_publish()
    except rospy.ROSInterruptException:
        pass