// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.constants.FieldConstants;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Supplier<Pose2d> robotPoseSupplier;

  // Per-camera flip detection state
  private final Pose2d[] lastAcceptedPose;
  private final double[] lastAcceptedTimestamp;

  public Vision(VisionConsumer consumer, Supplier<Pose2d> robotPoseSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.robotPoseSupplier = robotPoseSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // Initialize per-camera flip detection state
    this.lastAcceptedPose = new Pose2d[io.length];
    this.lastAcceptedTimestamp = new double[io.length];
    for (int i = 0; i < io.length; i++) {
      lastAcceptedPose[i] = null;
      lastAcceptedTimestamp[i] = -1.0;
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      String camName = VisionConstants.APTAG_CAMERA_NAMES[i];
      Logger.processInputs(
          "Vision/Camera" + i + "_" + camName.charAt(camName.length() - 1), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
      String camDirName = VisionConstants.APTAG_CAMERA_NAMES[cameraIndex];
      String cameraLabel = cameraIndex + "_" + camDirName.charAt(camDirName.length() - 1);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        Pose2d visionPose2d = observation.pose().toPose2d();
        Pose2d currentEstimate = robotPoseSupplier.get();
        double odometryDiscrepancy =
            visionPose2d.getTranslation().getDistance(currentEstimate.getTranslation());

        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > VisionConstants.SINGLE_TAG_MAX_AMBIGUITY) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > VisionConstants.MAX_Z_ERROR // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > FieldConstants.APTAG_FIELD_LAYOUT.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > FieldConstants.APTAG_FIELD_LAYOUT.getFieldWidth()

                // Single-tag distance gating — long-range single tags have severe flip ambiguity
                || (observation.tagCount() == 1
                    && observation.averageTagDistance()
                        > VisionConstants.SINGLE_TAG_MAX_DISTANCE_METERS)

                // Multi-tag hard distance cap — no observation beyond MAX_TAG_DISTANCE is reliable
                || (observation.tagCount() >= 2
                    && observation.averageTagDistance() > VisionConstants.MAX_TAG_DISTANCE)

                // Odometry discrepancy — reject if vision pose is too far from current estimate
                || odometryDiscrepancy > VisionConstants.MAX_POSE_DISCREPANCY_METERS;

        // Coplanar multi-tag check — if all visible tags share (roughly) the same
        // facing direction they give no additional rotational constraint, so apply
        // tighter distance and ambiguity gates.
        if (!rejectPose && observation.tagCount() >= 2) {
          boolean isCoplanar = true;
          int[] cameraTags = inputs[cameraIndex].tagIds;
          if (cameraTags.length >= 2) {
            // Get the normal (yaw) of the first tag as reference
            var refTagPose = FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(cameraTags[0]);
            if (refTagPose.isPresent()) {
              Rotation3d refRot = refTagPose.get().getRotation();
              for (int t = 1; t < cameraTags.length; t++) {
                var otherPose = FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(cameraTags[t]);
                if (otherPose.isPresent()) {
                  Rotation3d otherRot = otherPose.get().getRotation();
                  double angleDiff = Math.toDegrees(Math.abs(refRot.getZ() - otherRot.getZ()));
                  // Normalize to [0, 180]
                  angleDiff = angleDiff % 360.0;
                  if (angleDiff > 180.0) angleDiff = 360.0 - angleDiff;
                  if (angleDiff > VisionConstants.COPLANAR_ANGLE_THRESHOLD_DEG) {
                    isCoplanar = false;
                    break;
                  }
                }
              }
            }
          }
          if (isCoplanar) {
            if (observation.averageTagDistance() > VisionConstants.COPLANAR_MAX_DISTANCE_METERS) {
              rejectPose = true;
            } else if (observation.ambiguity() > VisionConstants.COPLANAR_MAX_AMBIGUITY) {
              rejectPose = true;
            }
          }
        }

        // Speed-based flip detection — reject if displacement between consecutive
        // accepted poses exceeds what is physically possible given elapsed time
        if (!rejectPose && lastAcceptedPose[cameraIndex] != null) {
          double dt = observation.timestamp() - lastAcceptedTimestamp[cameraIndex];
          if (dt > 0.0) {
            double clampedDt = Math.min(dt, VisionConstants.MAX_FLIP_DETECTION_DT_SECONDS);
            double maxDisplacement =
                VisionConstants.MAX_ROBOT_SPEED_MPS
                        * VisionConstants.SPEED_TOLERANCE_MULTIPLIER
                        * clampedDt
                    + VisionConstants.BASE_JUMP_TOLERANCE_METERS;
            double displacement =
                visionPose2d
                    .getTranslation()
                    .getDistance(lastAcceptedPose[cameraIndex].getTranslation());
            if (displacement > maxDisplacement) {
              rejectPose = true;
            }
          }
        }

        // Cross-camera consistency check — if other cameras have recently accepted
        // poses, reject this observation if it disagrees significantly with their
        // consensus. This catches a single camera giving an obviously wrong position
        // (bad extrinsic for the current angle, phantom tag, flip) when the others
        // agree. Falls back gracefully when only one camera is active.
        if (!rejectPose) {
          Translation2d crossCameraSum = Translation2d.kZero;
          int crossCameraCount = 0;
          for (int j = 0; j < io.length; j++) {
            if (j == cameraIndex) continue;
            if (lastAcceptedPose[j] == null) continue;
            double age = observation.timestamp() - lastAcceptedTimestamp[j];
            if (age < 0.0 || age > VisionConstants.CROSS_CAMERA_MAX_AGE_SECONDS) continue;
            crossCameraSum = crossCameraSum.plus(lastAcceptedPose[j].getTranslation());
            crossCameraCount++;
          }
          if (crossCameraCount >= 1) {
            Translation2d crossCameraCenter = crossCameraSum.div(crossCameraCount);
            double crossCameraDiscrepancy =
                visionPose2d.getTranslation().getDistance(crossCameraCenter);
            Logger.recordOutput(
                "Vision/Camera" + cameraLabel + "/CrossCameraDiscrepancyMeters",
                crossCameraDiscrepancy);
            if (crossCameraDiscrepancy > VisionConstants.MAX_CROSS_CAMERA_DISCREPANCY_METERS) {
              rejectPose = true;
            }
          }
        }

        // Log per-observation odometry discrepancy for tuning and diagnostics
        Logger.recordOutput(
            "Vision/Camera" + cameraLabel + "/OdometryDiscrepancyMeters", odometryDiscrepancy);

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations following the PhotonVision example heuristic:
        // start from a per-tag-count baseline, then scale up with distance squared.
        Matrix<N3, N1> stdDevs = observation.tagCount() > 1 ? MULTI_TAG_STDDEV : SINGLE_TAG_STDDEV;
        stdDevs =
            stdDevs.times(
                1.0 + (observation.averageTagDistance() * observation.averageTagDistance() / 30.0));
        if (cameraIndex < CAMERA_STDDEV_FACTORS.length) {
          stdDevs = stdDevs.times(CAMERA_STDDEV_FACTORS[cameraIndex]);
        }

        // Send vision observation
        consumer.accept(observation.pose().toPose2d(), observation.timestamp(), stdDevs);

        // Update per-camera flip detection state
        lastAcceptedPose[cameraIndex] = visionPose2d;
        lastAcceptedTimestamp[cameraIndex] = observation.timestamp();
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + cameraLabel + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraLabel + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraLabel + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraLabel + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
