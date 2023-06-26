# Scripts to convert the open mower map.bag file

## bag_to_gpx_tracks_utm.py
script to convert openmower ROS map.bag to gpx file

## gpx_to_bag.py
script to convert gpx files to openmower ROS map.bag
- before using change the base point lat0 and lon0 coordinates
- sourcing the open_mower_ros catkin workspace is needed because of custom message

## gpx_to_bag_2.py
script to convert gpx files to openmower ROS map.bag
- before using change the base point lat0 and lon0 coordinates
- no need for the open_mower_ros sourcing, this version imports the _MapArea.py file with custom message
- uses argpase for file name run with "python gpx_to_bag_included.py GPX_FILENAME"