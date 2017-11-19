ROS proxy services for [OSRM](http://project-osrm.org/)

## How to build and run

### Build and setup local OSRM pipeline

```
sudo apt install libbz2-dev zlib1g-dev libboost-dev libexpat1-dev liblua5.2-dev libtbb-dev
git clone https://github.com/Project-OSRM/osrm-backend/ -b 5.13 && cd osrm-backend
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

### Build and run the ROSRM services

Running services with default parameters (using shared memory block and multi-layer Dijkstra preprocessing)
```
catkin_make
source devel/setup.sh
rosrun rosrm rosrm_server
```

Running services with modified parameters
```
rosrun rosrm rosrm_server _algorithm:=CH _base_path:=/tmp/data/monaco-latest.osrm
```


### ROSRM requests

#### Shortest path

```
- position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
radiuses: [0]
bearings:
- {bearing: 0, range: 0}
approaches: ''
exclude: ['']
steps: false
continue_straight: false
annotation: 0
overview: 0
number_of_alternatives: 0"
```

- Point-to-point shortest path
[OSRM demo](http://map.project-osrm.org/?z=15&center=43.743259%2C7.431693&loc=43.749614%2C7.437959&loc=43.730639%2C7.411480&hl=en&alt=0)
or [Mapbox Directions demo](https://www.mapbox.com/get-directions/#13.78/43.7402/7.4247?coordinates=7.437959,43.749614;7.411480,43.730639)

```
rosservice call /rosrm/route "{waypoints: [{position: {'x':7.437959,'y':43.749614}}, {position: {'x':7.411480, 'y':43.730639}}], number_of_alternatives: 1, steps: true}"
```

- Shortest path with via points

[OSRM demo](http://map.project-osrm.org/?z=15&center=43.743259%2C7.431693&loc=43.745995%2C7.432251&loc=43.742139%2C7.43062&loc=43.734608%2C7.421576&hl=en&alt=0)

```
rosservice call /rosrm/route "waypoints: [{position: {'x':7.432251,'y':43.745995}}, {position: {'x':7.43062,'y':43.742139}}, {position: {'x':7.421576, 'y':43.734608}}]"
```


#### Map-matching

```
- header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
radiuses: [0]
approaches: ''
exclude: ['']
steps: false
continue_straight: false
annotation: 0
overview: 0
number_of_alternatives: 0
gaps: 0
tidy: false"
```


https://gist.github.com/oxidase/fddbbc74201120a1cb04749b3b9d52be

```
rosservice call /rosrm/match "{waypoints: [{pose:{position: {'x':7.431371,'y':43.744840}}}, {pose:{position: {'x':7.430491,'y':43.743088}}}, {pose:{position: {'x':7.430105,'y':43.741103}}}, {pose:{position: {'x':7.429804,'y':43.739119}}}, {pose:{position: {'x':7.426114,'y':43.737460}}}, {pose:{position: {'x':7.422702,'y':43.737073}}}, {pose:{position: {'x':7.419977,'y':43.735414}}}, {pose:{position: {'x':7.418518,'y':43.733925}}}, {pose:{position: {'x':7.418882,'y':43.732638}}}, {pose:{position: {'x':7.421500,'y':43.734096}}}], annotation: 63}"
```
