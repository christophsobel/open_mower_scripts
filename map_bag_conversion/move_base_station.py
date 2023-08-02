import utm
import rosbag
import math
import os
from geometry_msgs.msg import Polygon, Point32, Pose
from _MapArea import MapArea  # Imports the custom message from its package

#bag = rosbag.Bag('/home/pi/.ros/map.bag')
bag = rosbag.Bag(r'C:\Users\Operat0r\Documents\openmower\2023-06-22_Map_bag\map_combined.gpx_output_map.bag')

lat0 = 53.4043775
lon0 = 8.4611403
print("base_point_old", lat0, " ", lon0)

lat1 = 53.40436617
lon1 = 8.461142
print("base_point_new", lat1, " ", lon1)

topics = bag.get_type_and_topic_info()[1].keys()
print(topics)

x0, y0, zone, ut = utm.from_latlon(lat0, lon0)
print("base_point_old in utm", x0, " ", y0)

x1, y1, zone1, ut1 = utm.from_latlon(lat1, lon1)
print("base_point_new in utm", x1, " ", y1)

dx = x1 - x0
dy = y1 - y0
print("delta x", dx, " y", dy)

# Loop through areas and obstacles

mowing_areas_list = []
for topic, msg, t in bag.read_messages(topics=['mowing_areas']):
    map_area = MapArea()
    # outline
    point_list = []
    for point in msg.area.points:
        # Create points:
        point_list.append(Point32(x=point.x + dx, y=point.y + dy, z=0.0))
    poly = Polygon(points=point_list)
    map_area.area = poly
    # obstacles
    for obst in msg.obstacles:
        point_list = []
        for point in obst.points:
            # Create points:
            point_list.append(Point32(x=point.x + dx, y=point.y + dy, z=0.0))
        poly = Polygon(points=point_list)
        map_area.obstacles.append(poly)
    mowing_areas_list.append(map_area)

navigation_areas_list = []
for topic, msg, t in bag.read_messages(topics=['navigation_areas']):
    map_area = MapArea()
    # outline
    point_list = []
    for point in msg.area.points:
        # Create points:
        point_list.append(Point32(x=point.x + dx, y=point.y + dy, z=0.0))
    poly = Polygon(points=point_list)
    map_area.area = poly
    # obstacles
    for obst in msg.obstacles:
        point_list = []
        for point in obst.points:
            # Create points:
            point_list.append(Point32(x=point.x + dx, y=point.y + dy, z=0.0))
        poly = Polygon(points=point_list)
        map_area.obstacles.append(poly)
    navigation_areas_list.append(map_area)

# docking point
docking_pose = Pose()
for topic, msg, t in bag.read_messages(topics=['docking_point']):
    # print(msg)
    print("parsing docking point")
    docking_pose.position.x = msg.position.x + dx
    docking_pose.position.y = msg.position.y + dy
	# Pose Orientation
    docking_pose.orientation = msg.orientation

bag.close()


with rosbag.Bag('_shifted_map.bag', 'w') as outbag:
	print('writing bag file')
	for mowing_area in mowing_areas_list:
		outbag.write("mowing_areas", mowing_area)
	for navigation_area in navigation_areas_list:	
		outbag.write("navigation_areas", navigation_area)
	outbag.write("docking_point", docking_pose)

print('Done')