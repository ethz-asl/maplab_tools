# Maplab Tools

## Service Interface

Provides service calls to perform certain actions in maplab remotely.  

## Camera Info Publisher

Reads a Maplab sensors yaml, subscribes to all the image topics and republishes them together with the camera_info.

## Voxgraph Submap Converter

Listens to submaps from voxgraph and republishs them as `sensor_msgs::PointCloud2`. Also, allows to store the submaps as PLYs. 
