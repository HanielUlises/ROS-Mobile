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
