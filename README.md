# SERIAL KINEMATICS 

**Maintainer:** Gedaliah Knizhnik (gedaliah.knizhnik@gmail.com)

In my experience, libraries that allow the modeling of serial kinematic chains, such as for robotic manipulators, are bundled with large libraries for control, modeling, and planning that involve significant effort to setup, migrate to, and incorporate into existing codebases. Nothing against that, but it's a lot of work to use them if you just need some serial kinematics modeling and don't want to buy into the whole infrastructure.

This repository aims to build a lightweight library for modeling serial kinematic chains. Current features are:

1. Creation and display of DH parameterized serial kinematic chains/robots.
2. Forward Kinematics (FK) on serial kinematic chains
    * Transform any frame into any other frame
    * Transform points or velocities in any frame into another frame 

Features planned for development:

* Addition of non-mobile offset frames for the end-effector
* Inverse kinematics (IK)
* Jacobian support for control purposes. 

This README will get updated as these features (and more) become functional.
