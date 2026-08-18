# Map provenance

`willow-2010-02-18-0.10.pgm` / `.yaml`

A 2D occupancy grid of the Willow Garage office, built by a robot from laser
data on 2010-02-18 at 0.1 m resolution. Distributed with the
`turtlebot_navigation` package of [turtlebot_apps][repo] (OSRF, Tully Foote,
BSD licence) and retrieved unmodified from:

    https://raw.githubusercontent.com/turtlebot/turtlebot_apps/indigo/turtlebot_navigation/maps/willow-2010-02-18-0.10.pgm
    https://raw.githubusercontent.com/turtlebot/turtlebot_apps/indigo/turtlebot_navigation/maps/willow-2010-02-18-0.10.yaml

It is used as the scenario geometry of the project's second iteration. The map
is not consumed at run time: `tools/map_to_world.py` extrudes it once into
`worlds/willow_office.world`, which is what Gazebo loads. Regenerate with

    python3 tools/map_to_world.py \
        --name willow_office --crop -23 -8 8 26 --open 6 --close 3 --band 0.4 \
        -o src/mrs_bringup/worlds/willow_office.world

Two properties of the source are worth keeping in mind when reading results
obtained on it. The geometry is a real building's, so its rooms are unequal and
its corridors close into loops that no hand-authored world would reproduce for
free. But it is also already the output of one SLAM system: the radial spray at
doorways is that robot's laser seeing through them, and the wall thickness
varies with how often the robot looked at a given wall. The extrusion removes
the spray and quantises the rest, so what the fleet explores is a cleaned
reading of a real building rather than the building itself.

[repo]: https://github.com/turtlebot/turtlebot_apps

---

`fr079-lau-0.05.pgm` / `.yaml`

A 2D occupancy grid of building 079 of the University of Freiburg computer
science campus: one long corridor with some forty offices and labs opening off
both sides. The grid is the standard `fr079` map of the [Robotics Data Set
Repository (Radish)][radish], built from the laser and odometry log recorded by
Cyrill Stachniss; the image used here is the rendering published with the
[DynamicVoronoi][dv] code of Lau, Sprunk and Burgard, retrieved from

    http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/FR079.png

It is used as the scenario geometry of the project's third iteration, in which
the reactive exploration policy of the first two is replaced by a deliberative
one. The building is chosen for its topology rather than its size: a corridor
spine with many small rooms hung off it turns exploration into a sequence of
discrete, spatially separated tasks, which is what makes an assignment between
agents either good or bad. A corridor ring, such as the Willow Garage map of the
second iteration, does not.

The published image carries no metadata, so two things are supplied here and
should be read as assumptions rather than as records. First, the free / occupied
/ unknown classes are recovered by thresholding the grey levels, which are
already close to trichromatic in the source. Second, the resolution is taken to
be 0.05 m per pixel, the resolution the dataset is distributed at; it makes the
909 x 322 pixel image a 45.5 x 16.1 m building, consistent with the dimensions
published for fr079. If that scale is wrong, every metric length in the third
iteration is wrong by the same factor.

    python3 tools/png_to_map.py FR079.png \
        -o src/mrs_bringup/maps/fr079-lau-0.05 --resolution 0.05

    python3 tools/map_to_world.py --map src/mrs_bringup/maps/fr079-lau-0.05 \
        --name fr079_office --angle 0 --open 3 --close 1 --band 0.4 \
        -o src/mrs_bringup/worlds/fr079_office.world

The map is already axis-aligned, so no deskew is applied (`--angle 0`), and the
whole building is extruded, so there is no crop window: the two scenario choices
the second iteration had to make and defend do not arise here. The morphological
radii do: the closing must stay below the thickness of the thinnest interior
wall, since a closing wide enough to bridge a 0.1 m partition merges the rooms
on either side of it into one.

[radish]: http://radish.sourceforge.net/
[dv]: http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
