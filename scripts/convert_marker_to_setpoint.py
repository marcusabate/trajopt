#!/usr/bin/env python

"""
    convert_marker_to_setpoint.py

    Marcus Abate

    This script grabs the MarkerArray of poses given by traj_generator.cpp
    and converts each pose to a setpoint that is then given to aero_quadsim
    for use as a "waypoint" during simulation. The onboard px4 controller
    determines actual execution in this implementation: a faster controller
    is NOT implemented here.

    The executable here simply does conversion; it does not interface directly
    with the aero_quadsim simulator, but rather sends the requisite messages.
"""

def convert_poses():
    """
        Subscribe to /visualization_marker_array and grab all markers.
        Cut out all but the last marker: this one contains waypoints along the
            'path'.
        Publish these 'path' waypoints to the desired_setpoint topic
