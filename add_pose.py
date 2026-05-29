
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    # Robot rotates 45° CCW, moves 2m forward, then rotates 45° more CCW.
    # Composing these three motions in X(3)'s frame gives Pose2(sqrt(2), sqrt(2), pi/2).
    odometry = gtsam.Pose2(math.sqrt(2), math.sqrt(2), math.pi / 2)
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), odometry, ODOMETRY_NOISE))

    true_pose_3 = gtsam.Pose2(4.0, 0.0, 0.0)
    pose_4 = true_pose_3.compose(odometry)
    initial_estimate.insert(X(4), pose_4)

    return graph, initial_estimate
