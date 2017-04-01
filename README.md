jlbot
============
A controller for mobile robot navigation that follows the hybrid deliberate/reactive paradigm. It finds a path to a destination by using the Wavefront Planner algorithm and uses Motor Schemas for avoiding obstacles and following the planned path. It is made for Player, a network server for robot control, and Stage, a mobile robot simulator.
Compiling
============
cd <project_home>/build
cmake ..
make
Running
============
USAGE: jlbot x y
The current working directory must the same as the pnm file.
```bash
cd <project_home>/resources
../bin/jlgot 8.5 -4
```
