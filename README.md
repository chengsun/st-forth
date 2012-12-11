ST Forth robotic arm
===

Key files
---
* ik.fth is the main file, with inverse kinematics routines; it can use
  routines from either sim.fth or bv.fth
* sim.fth is a backend designed for cooperating with the simulator
* bv.fth is a backend designed for running on the actual robot

* sim.cpp is a robot arm simulator and visualiser

The other stuff are support files or libraries that ST have supplied us with
