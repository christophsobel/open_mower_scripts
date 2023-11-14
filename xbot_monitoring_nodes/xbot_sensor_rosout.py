#!/usr/bin/env python

# get rosout, parse and publish to sensor topic

import rospy
from xbot_msgs.msg import SensorInfo
from xbot_msgs.msg import SensorDataString
from rosgraph_msgs.msg import Log

my_info = SensorInfo()
sensor_data_publisher = None

# publish sensor data
def sensor_publish():
    global sensor_data_publisher
    rospy.init_node('xbot_sensor_rosout', anonymous=True)

    my_info.has_critical_high = False
    my_info.has_critical_low = False
    my_info.has_min_max = False
    # my_info.min_value = 
    # my_info.max_value = 
    my_info.value_type = SensorInfo.TYPE_STRING
    # my_info.value_description = ""
    my_info.unit = ""
    my_info.sensor_id = "rosout"
    my_info.sensor_name = "rosout"

    sensor_info_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/info", SensorInfo, queue_size=1, latch=True)  
    sensor_data_publisher = rospy.Publisher("xbot_monitoring/sensors/" + my_info.sensor_id + "/data", SensorDataString, queue_size=1)
    sensor_info_publisher.publish(my_info)

    

    rospy.Subscriber("/rosout_agg", Log, callback_rosout)

    rospy.spin()


# read rosout and parse
def callback_rosout(log_msg):
    global sensor_data_publisher
    # print(log_msg.level)
    if "congested" in log_msg.msg:
        return
    if log_msg.level > 4:
        print(str(log_msg.level) + ": " + log_msg.name + ": " + log_msg.msg)
        data = SensorDataString()
        data.stamp = rospy.Time.now()
        data.data = str(log_msg.level) + ": " + log_msg.name + ": " + log_msg.msg
        #print(data.data)
        # Publish
        sensor_data_publisher.publish(data)



if __name__ == '__main__':
    try:
        print("Monitoring rosout")
        sensor_publish()
    except rospy.ROSInterruptException:
        pass


##
## Severity level constants
##
# byte DEBUG=1 #debug level
# byte INFO=2  #general level
# byte WARN=4  #warning level
# byte ERROR=8 #error level
# byte FATAL=16 #fatal/critical level
##
## Fields
##
# Header header
# byte level
# string name # name of the node
# string msg # message 
# string file # file the message came from
# string function # function the message came from
# uint32 line # line the message came from
# string[] topics # topic names that the node publishes