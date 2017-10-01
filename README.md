# Open source routing machine ROS service

http://project-osrm.org/

## How to build and run the service

### Build and setup local OSRM pipeline
```
git clone https://github.com/Project-OSRM/osrm-backend/ -b 5.12 && osrm-backend
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=On && make -j4
sudo make install
```

### Prepare routing data
```
mkdir -p /tmp/data
wget -P /tmp/data http://download.geofabrik.de/europe/monaco-latest.osm.pbf
osrm-extract -p /usr/local/share/osrm/profiles/car.lua /tmp/data/monaco-latest.osm.pbf && osrm-contract /tmp/data/monaco-latest.osrm && osrm-datastore /tmp/data/monaco-latest.osrm
```

### Build and run the ROSRM service
```
catkin_make
source devel/setup.sh
rosrun rosrm rosrm_server
```

### Request some routes
```
rosservice call /rosrm/route "waypoints: [{position: {'x':7.432251,'y':43.745995}}, {position: {'x':7.43062,'y':43.742139}}, {position: {'x':7.421576, 'y':43.734608}}]"
```
