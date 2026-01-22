# Lab 2

Be sure to read lab instructions carefully. When asked questions in the lab,
responses should be full english sentences.

Learning objectives:

- Practice with Python
- Practice with Pixi, a modern software package manager
- Learning to use inverse kinematics solver
- Learning to work with URDF files
- Learning to use visualization as a way to debug inverse kinematics solutions
and URDF files

## Part 0: Pixi

First, we must install a package manager to install software for us. [Pixi]() is
a relatively new cross-platform package management tool that will come in handy
this quarter as we work with various robotic systems. It is especially good for
managing Python packages and dependencies.

To install pixi on linux, run the following command in a shell:

```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

or use wget:

```bash
wget -qO- https://pixi.sh/install.sh | sh
```

See the Pixi website above for install instructions on other platforms.

In this repository, run `pixi install` to initialize the packages we will need.

Run `pixi shell` to start a shell with our software. Work in this shell for the rest of the lab.

## Part 1: URDF

**URDF** stands for **Unified Robot Description Format.** It is an XML-based
text format for representing a robot model. [The full specification can be found
here](https://wiki.ros.org/urdf/XML).

There are many tools for creating, viewing, and editing URDF files. One is
included in this pixi package.

Try running

```bash
yourdfpy ./ur5/ur5_gripper.urdf
```

and a window should open, allowing you to inspect the robot model.

Open the URDF file in a text editor and inspect the file. As you may notice,
these files are not the nicest to read and write by humans.

Our URDF file was created from a "xacro" file, a macro language for describing
XML. Inspect the xacro file
[here](https://github.com/fmauch/universal_robot/blob/kinetic-devel/ur_description/urdf/ur5.urdf.xacro).

1. From the xacro file, can you determine the lengths of each link of the robot
arm? What are the lengths? Given these, as well as the visualization of our
robot arm in yourdfpy, what do you think the maximum range of the robot arm is
(approximately)? Note that units are in meters.

## Part 2: Inverse Kinematics

In a terminal, run `pixi run setup` then `jupyter lab`. The second commmand will
open a browser window. This is the Jupyter Lab interface.

Click the "497 Kernel" button in the top of the home page.

Then, use the sidebar to open the file `my_IK.ipynb`. Execute cells of code in
order, from top to bottom, using the `Shift` + `Enter` keys.

Play around with different target end effector positions, and see what effect
they have on the resulting robot arm poses.

2. Can you get your robot end effector to the maximum range you estimated in Q1,
along any of the x/y/z axes? Why or why not?

3. Write code to find and visualize the 3D volume of points that are reachable
by the robot arm. Hints: the forward kinematics function gives you the actual
position of the end effector for a given target position. The target position is
visualized as a red dot in the original plotting code. You want to randomly
sample from points on a large sphere around the robot, and attempt to get the
robot to reach those points, such that the arm is fully "stretched". Record the
resulting points, and visualize. You may want the "Convex Hull" method in the
"scipy" library, as well as the method `plot_trisurf` from the matplotlib
library. AI is allowed for producing the visualization code, but not for
sampling from the sphere. Sampling points on the surface of a sphere is a
[non-trivial problem](https://mathworld.wolfram.com/SpherePointPicking.html).

## Part 3: Collision Checking

4. Define and visualize a plane in the space with the robot arm; the plane should not
intersect the robot arm when all the joints are set to zero position. Write code
to determine if **any part of** the robot arm is intersecting the plane for an arbitrary pose of
the robot arm, and include at least three test cases for your method.

## Part 4: Trajectory Generation (Graduate Students Only)

5. Write code to generate a trajectory that starts from the robot arm at zero
position, and moves the end effector in such a way that the end effector
approaches very near to the plane, but no part of the robot arm intersects the
plane. Create an `mp4` animation of your robot arm moving (either directly in python,
or by saving multiple `.png` files to disk and animating the series of `png`
files with a program like `ffmpeg`, and submit along with your writeup.

Hint: manually define a few "safe" waypoints, starting from the zero position.
Look at the [inverse kinematics
module](https://ikpy.readthedocs.io/en/latest/inverse_kinematics.html) and try
setting the `starting_nodes_angles` using previous "safe" solutions. You may
also try setting the [orientation
mode](https://github.com/Phylliade/ikpy/wiki/Orientation).

(Extra credit) Visualize the trajectory of the robot end effector by "tracing a
line" in your visualization, and make the robot arm "draw something" on the
plane.
