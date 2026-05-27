import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # GEWIJZIGD: We halen X(4) uit initial_estimate (want daar zit hij gegarandeerd in!)
    pose_x4 = initial_estimate.atPose2(X(4))
    
    # L(2) halen we uit de result of initial_estimate (beide kan, result is nauwkeuriger)
    point_l2 = result.atPoint2(L(2))
    
    # De wiskundige formules (Doel minus Start)
    dx = point_l2[0] - pose_x4.x()
    dy = point_l2[1] - pose_x4.y()
    
    # Bereken afstand en hoek
    distance = math.sqrt(dx**2 + dy**2)
    bearing_rad = math.atan2(dy, dx) - pose_x4.theta()
    rotation = math.degrees(bearing_rad)
    
    # Voeg de factor toe
    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), gtsam.Rot2.fromDegrees(rotation), distance, MEASUREMENT_NOISE))
    
    return graph