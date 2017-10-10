# Open source routing machine ROS service

ROS service for [OSRM](http://project-osrm.org/)

## How to build and run the service

### Build and setup local OSRM pipeline

```
sudo apt install libbz2-dev zlib1g-dev libboost-dev libexpat1-dev liblua5.2-dev libtbb-dev
git clone https://github.com/Project-OSRM/osrm-backend/ -b 5.12 && cd osrm-backend
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=On && make -j4
sudo make install
```

### Prepare routing data

#### Download OSM data

```
mkdir -p /tmp/data
wget -P /tmp/data http://download.geofabrik.de/europe/monaco-latest.osm.pbf
```

#### Data preprocessing

```
mkdir -p /tmp/data
wget -P /tmp/data http://download.geofabrik.de/europe/monaco-latest.osm.pbf
osrm-extract -p /usr/local/share/osrm/profiles/car.lua /tmp/data/monaco-latest.osm.pbf
osrm-partition /tmp/data/monaco-latest.osrm
osrm-customize /tmp/data/monaco-latest.osrm
osrm-contract /tmp/data/monaco-latest.osrm
```

#### Load OSRM data into shared memory

```
osrm-datastore /tmp/data/monaco-latest.osrm
```

### Build and run the ROSRM service

```
catkin_make
source devel/setup.sh
rosrun rosrm rosrm_server
```

### ROSRM requests

```
rosservice call /rosrm/route "waypoints: [{position: {'x':7.432251,'y':43.745995}}, {position: {'x':7.43062,'y':43.742139}}, {position: {'x':7.421576, 'y':43.734608}}]"

rosservice call /rosrm/route "{waypoints: [{position: {'x':7.437959,'y':43.749614}}, {position: {'x':7.411480, 'y':43.730639}}], number_of_alternatives: 1, steps: true}"
```
