# Nodes for the xbot_monitoring package
Various nodes that report data via MQTT

copy the scripts to `open_mower_ros\src\lib\xbot_monitoring\src\`

Add following to `open_mower_ros\src\open_mower\launch\open_mower.launch`
```
<node pkg="xbot_monitoring" type="xbot_sensor_acceleration.py" name="xbot_sensor_acceleration" output="screen" respawn="true" respawn_delay="10"/>
<node pkg="xbot_monitoring" type="xbot_sensor_rosout.py" name="xbot_sensor_rosout" output="screen" respawn="true" respawn_delay="10"/>
<node pkg="xbot_monitoring" type="xbot_sensor_wifi.py" name="xbot_sensor_wifi" output="screen" respawn="true" respawn_delay="10"/>
```