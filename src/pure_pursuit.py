#! /usr/bin/env python

"""
@author: jared
Generic pure pursuit control law for bicycle dynamics model
"""

######## NOTE : ALTERED PURE PURSUIT FOR CITY DRIVING ########
######## NOT FOR REGULAR USAGE W/ PATH OF WAYPOINTS ##########

import numpy as np

def purepursuit(ld, L, vel_des, x, y, q, segments):
    """
    Given lookahead distance, robot length, desired velocity, current pose, and path waypoints (2d np array), 
    returns an instantaneous x linear velocity and steering angle eta to follow trajectory
    """
    goal = look_ahead(ld, x, y, segments) # Instantaneous goal - lookahead point

    if goal is None:
        return None, None

    R = (ld*ld)/(2*goal[1]) # Radius of curvature connecting these points

    # Control law !!
    eta = np.arctan(L / R)
    lin_vel = vel_des

    return eta, lin_vel

def look_ahead(ld, x, y, segments):
    """
    Given position, lookahead distance, and line segments to check through
    returns the point on the first found segment that is instantaneously one look ahead distance away from rover
    If multiple intersections, selects "farthest along" point
    https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm

    segment type : (x1,y1,x2,y2)
    """
    for seg in segments:

        seg_vec = np.array([seg[2] - seg[0], seg[3] - seg[1]])
        rover_vec = np.array([seg[0], seg[1]]) - np.array([x, y])

        a = np.dot(seg_vec, seg_vec)
        b = 2 * np.dot(rover_vec, seg_vec)
        c = np.dot(rover_vec, rover_vec) - ld * ld
        discriminant = b*b-4*a*c
        
        if discriminant >= 0:
            # p1 and p2 are integer multiples of the segment vector corresponding to 
            # where the intersection lies
            p1 = (-b + np.sqrt(discriminant))/(2*a)
            p2 = (-b - np.sqrt(discriminant))/(2*a)
            
            # p1 will always be farther than p2
            if p1 >= 0 and p1 <= 1:
                # Successful intersection
                return p1 * seg_vec + np.array([seg[0], seg[1]])
                
            if p2 >= 0 and p2 <= 1:
                # Successful intersection
                return p2 * seg_vec + np.array([seg[0], seg[1]])
    
    # If nothing found
    return None
