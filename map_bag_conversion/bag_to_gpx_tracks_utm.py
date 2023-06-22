#!/usr/bin/python

#===========================================
# Christoph Sobel 11.05.2023
# script to convert openmower ros map.bag to gpx file
#
# initial version published on https://wiki.openmower.de/index.php?title=Modify_map_using_drone_footage
#===========================================

from codecs import latin_1_decode
import rosbag
import utm
import gpxpy.gpx
import math
import os
from tf.transformations import euler_from_quaternion

bag = rosbag.Bag('/home/pi/.ros/map.bag')
# bag = rosbag.Bag('output.bag')

lat0 = 53.00
lon0 = 8.00
# lat0 = float(os.getenv('OM_DATUM_LAT'))
# lon0 = float(os.getenv('OM_DATUM_LONG'))
print("base_point", lat0, " ", lon0)

topics = bag.get_type_and_topic_info()[1].keys()
print(topics)

gpx = gpxpy.gpx.GPX()
gpx.name = 'open_mower_map'
gpx.description = 'Map data from openmower map.bag'

# base point (e.g. gps antenna)
# base_point = gpxpy.gpx.GPXWaypoint(latitude=lat0, longitude=lon0, name="base_point")
# gpx.waypoints.append(base_point)

x0, y0, zone, ut = utm.from_latlon(lat0, lon0)
print("base_point in utm", x0, " ", y0)

gpx_track = gpxpy.gpx.GPXTrack(name="base_point")
gpx.tracks.append(gpx_track)
gpx_segment = gpxpy.gpx.GPXTrackSegment()
gpx_track.segments.append(gpx_segment)
gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lat0, lon0, elevation=0))


# Loop through areas and obstacles
area_counter = 0

for area_type in ['mowing_areas', 'navigation_areas']:
    for topic, msg, t in bag.read_messages(topics=[area_type]):
        print("{}_{}_area".format(area_type[:-1], area_counter))
        # Create track in GPX:
        gpx_track = gpxpy.gpx.GPXTrack(name="{}_{}_area".format(area_type[:-1], area_counter))
        gpx.tracks.append(gpx_track)
        # Create segment in GPX track:
        gpx_segment = gpxpy.gpx.GPXTrackSegment()
        gpx_track.segments.append(gpx_segment)

        # outline
        for point in msg.area.points:
            # Create points:
            lat, lon = utm.to_latlon(x0 + point.x, y0 + point.y, zone, ut)
            gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon, elevation=0))

        # obstacles
        obstacle_counter = 0
        for obst in msg.obstacles:
            print("{}_{}_obstacle_{}".format(area_type[:-1], area_counter, obstacle_counter))
            # Create track in GPX:
            gpx_track = gpxpy.gpx.GPXTrack(name="{}_{}_obstacle_{}".format(area_type[:-1], area_counter, obstacle_counter))
            gpx.tracks.append(gpx_track)
            # Create segment in GPX track:
            gpx_segment = gpxpy.gpx.GPXTrackSegment()
            gpx_track.segments.append(gpx_segment)
            for point in obst.points:
                # Create points:
                lat, lon = utm.to_latlon(x0 + point.x, y0 + point.y, zone, ut)
                gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon, elevation=0))
            obstacle_counter += 1
        area_counter += 1

# docking point
for topic, msg, t in bag.read_messages(topics=['docking_point']):
    # print(msg)
    print("parsing docking point")
    quat_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(quat_list)
    # print((roll, pitch, yaw))
    x2 = msg.position.x + math.cos(yaw)
    y2 = msg.position.y + math.sin(yaw)

    lat1, lon1 = utm.to_latlon(x0 + msg.position.x, y0 + msg.position.y, zone, ut)
    lat2, lon2 = utm.to_latlon(x0 + x2,y0 + y2, zone, ut)

    gpx_track = gpxpy.gpx.GPXTrack(name="docking_point")
    gpx.tracks.append(gpx_track)
    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)
    gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lat1, lon1, elevation=0))
    gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lat2, lon2, elevation=0))

print('Created GPX file')
with open("map_bag.gpx", "w") as f:
    f.write(gpx.to_xml())

bag.close()
print('Done')