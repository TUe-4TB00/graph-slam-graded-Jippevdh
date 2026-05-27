
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    
    odometry_measurements = gtsam.Pose2(2.0, 0.0, np.pi/2)
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), odometry_measurements, ODOMETRY_NOISE))

   
    estimated_pose = gtsam.Pose2(5.41, 1.41, np.pi/2)
    initial_estimate.insert(X(4), estimated_pose)
    
    return graph, initial_estimate