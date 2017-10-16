Turtlebot Delivery Project 2015/2016
======

This project is the result of the TAMS master project 2015.
The goal is to implement a simple delivery scenario, in which an object is recognized
on a table, picked up by the ur5 arm and placed on a nearby turtlebot.
In turn the turtlebot brings the object to a desired target.

The functionality is divided into 4 packages:
* coordination - high level task execution and turtlebot communication
* manipulation - ur5 pick and place capability
* objectrecognition - object localization and marker publishing
* turtlebot - fetch and delivery task implementation

---

## Usage

When running the demo on hardware it is recommended to use the turtlebot 'Donny', since
it has a funnel attached to the top which fits perfectly with a cylinder as for
instance a pringle's box.
There are two ways to run the demo, either with or without turtlebot delivery, as is
described below.

### Running the full demo

The full demo can by run by:

```roslaunch tams_ur5_turtlebot_delivery delivery.launch```

This includes all necessary components that need to be run on the ur5 workstation.

Additionally the corresponding turtlebot needs to be started and initialized with:

```roslaunch tams_ur5_turtlebot_delivery_turtlebot turtlebot_delivery.launch```

### Running the demo without the turtlebot

Alternatively it is possible to run the demo without the turtlebot functionality.
To do that 'donny' needs to be placed at the docking station next to the table beforehand.

By calling 
```roslaunch tams_ur5_turtlebot_delivery ur5_pick_setup.launch```
only those components are launched that are necessary for objectrecognition and manipulation.

To execute the pick and place attempt one simply needs to start the PlaceObjectActionClient:

```rosrun tams_ur5_turtlebot_delivery_manipulation PlaceObjectActionClient```
