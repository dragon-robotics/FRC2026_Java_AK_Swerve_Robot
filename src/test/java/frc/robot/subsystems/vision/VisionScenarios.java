package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

final class VisionScenarios {
  private VisionScenarios() {}

  static PoseObservation goodMultiTag(double x, double y, double headingRad, double timestamp) {
    return obs(timestamp, pose(x, y, 0.0, headingRad), 0.08, 2, 2.0);
  }

  static PoseObservation flippedSingleTag(double x, double y, double headingRad, double timestamp) {
    return obs(timestamp, pose(x, y, 0.0, headingRad), 0.28, 1, 4.0);
  }

  static PoseObservation singleTagHighAmbiguity(double x, double y, double timestamp) {
    return obs(timestamp, pose(x, y, 0.0, 0.0), 0.60, 1, 3.0);
  }

  static PoseObservation outOfBounds(double timestamp) {
    return obs(timestamp, pose(-3.0, -3.0, 0.0, 0.0), 0.10, 2, 2.0);
  }

  static PoseObservation highZ(double x, double y, double timestamp) {
    return obs(timestamp, pose(x, y, 1.5, 0.0), 0.10, 2, 2.0);
  }

  static PoseObservation tooFar(double x, double y, double timestamp) {
    return obs(timestamp, pose(x, y, 0.0, 0.0), 0.10, 2, 7.0);
  }

  private static Pose3d pose(double x, double y, double z, double yawRad) {
    return new Pose3d(x, y, z, new Rotation3d(0.0, 0.0, yawRad));
  }

  private static PoseObservation obs(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double avgDistance) {
    return new PoseObservation(
        timestamp, pose, ambiguity, tagCount, avgDistance, PoseObservationType.PHOTONVISION);
  }
}
