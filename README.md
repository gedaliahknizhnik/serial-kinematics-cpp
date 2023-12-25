# SERIAL KINEMATICS 

**Maintainer:** Gedaliah Knizhnik (gedaliah.knizhnik@gmail.com)

In my experience, libraries that allow the modeling of serial kinematic chains, such as for robotic manipulators, are bundled with large libraries for control, modeling, and planning that involve significant effort to setup, migrate to, and incorporate into existing codebases. Nothing against that, but it's a lot of work to use them if you just need some serial kinematics modeling and don't want to buy into the whole infrastructure.

In this repository I want to build a lightweight library for modeling serial kinematic chains, with the goal of being able to: 

* Calculate the position of the end effector in the base frame
* Calculate the position of arbitrary points in any intermediate frame in the base frame
* Calculate the Jacobian of any frame for control purposes.

This README will get updated as these features (and more) become functional.
