URE Manipulation and Simulation Tutorials
=========================================

This package supplies Sphinx-based tutorial content to assist you with simulating a URE manipulator inside of
Gazebo, and controlling it from MoveIt! through RVIZ.  Then, adding a manipulator to a simulated Ridgeback, creating
a custom MoveIt! configuration for your new creation and using a stereo camera to fuse sensor data into the arm motion
planning
:doc:`Gazebo <gazebo>` is where we will start with a basic simulation of just a UR5E arm
:doc:`MoveIt! <moveit>` will take that simulated arm and plan and execute motions from inside RVIZ
:doc:`Platform <platform>` will expand a Ridgeback mobile platform to have UR5E arms integrated on top of it
:doc:`Custom MoveIt! <moveit2>` will create a custom MoveIt! configuration for your Ridgeback/UR system
:doc:`Vision <vision>` will add dynamic path planning based on the obstacles seen by a stereo camera

.. toctree::
    :titlesonly:
    :hidden:
    :caption: Getting Started

    Overview <self>
    gazebo
    moveit
    platform
    moveit2
    vision
