// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.constants.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private static final double SNAPSHOT_MAX_AGE_SECONDS = 0.5;
  private static final double DISABLED_AUTO_RESEED_MIN_INTERVAL_SECONDS = 0.5;
  private static final double DISABLED_AUTO_RESEED_DELTA_METERS = 0.25;
  private static final int DISABLED_AUTO_RESEED_MIN_TAG_COUNT = 2;
  private static final int MULTITAG_INIT_STABLE_POSES_REQUIRED = 5;
  private static final double MULTITAG_INIT_MAX_TRANSLATION_DELTA_METERS = 0.20;
  private static final double MULTITAG_INIT_MAX_HEADING_DELTA_DEGREES = 10.0;

  private final VisionConsumer consumer;
  private final Consumer<Pose2d> poseResetConsumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Supplier<Pose2d> robotPoseSupplier;

  // Per-camera flip/cross-camera consistency state
  private final Pose2d[] lastAcceptedPose;
  private final double[] lastAcceptedTimestamp;

  // Latest accepted observation snapshot used by superstructure/diagnostics.
  private Pose2d lastAcceptedPoseGlobal = null;
  private int[] lastAcceptedTagIdsGlobal = new int[0];
  private double lastAcceptedTimestampGlobal = -1.0;

  // Tighten translation trust while actively aiming.
  private boolean aiming = false;
  private boolean visionInitializationComplete = false;
  private boolean wasDisabledLastLoop = false;
  private boolean hasAutoReseededThisDisabledCycle = false;
  private double lastDisabledAutoReseedTime = Double.NEGATIVE_INFINITY;
  private Pose2d lastStableMultitagPose = null;
  private double lastStableMultitagTimestamp = Double.NEGATIVE_INFINITY;
  private int stableMultitagPoseCount = 0;

  public Vision(
      VisionConsumer consumer,
      Consumer<Pose2d> poseResetConsumer,
      Supplier<Pose2d> robotPoseSupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.poseResetConsumer = poseResetConsumer;
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

  public void setAiming(boolean aiming) {
    this.aiming = aiming;
  }

  public Optional<AcceptedObservationSnapshot> getLatestAcceptedObservationSnapshot() {
    if (lastAcceptedPoseGlobal == null
        || (Timer.getFPGATimestamp() - lastAcceptedTimestampGlobal) > SNAPSHOT_MAX_AGE_SECONDS) {
      return Optional.empty();
    }
    return Optional.of(
        new AcceptedObservationSnapshot(
            lastAcceptedPoseGlobal,
            Arrays.copyOf(lastAcceptedTagIdsGlobal, lastAcceptedTagIdsGlobal.length),
            lastAcceptedTimestampGlobal));
  }

  public void setHeadingProvider(VisionIOPhotonVision.VisionHeadingProvider headingProvider) {
    for (VisionIO visionIO : io) {
      if (visionIO instanceof VisionIOPhotonVision photon) {
        photon.setHeadingProvider(headingProvider);
      }
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      String camName = VisionConstants.APTAG_CAMERA_NAMES[i];
      Logger.processInputs(
          "Vision/Camera" + i + "_" + camName.charAt(camName.length() - 1), inputs[i]);
    }

    List<Pose3d> allTagPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    List<Pose3d> allRobotPosesRejected = new ArrayList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
      String camDirName = VisionConstants.APTAG_CAMERA_NAMES[cameraIndex];
      String cameraLabel = cameraIndex + "_" + camDirName.charAt(camDirName.length() - 1);

      List<Pose3d> tagPoses = new ArrayList<>();
      List<Pose3d> robotPoses = new ArrayList<>();
      List<Pose3d> robotPosesAccepted = new ArrayList<>();
      List<Pose3d> robotPosesRejected = new ArrayList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        Pose2d visionPose2d = observation.pose().toPose2d();
        Pose2d currentEstimate = robotPoseSupplier.get();
        double odometryDiscrepancy =
            visionPose2d.getTranslation().getDistance(currentEstimate.getTranslation());

        boolean rejectPose =
            rejectionReason(observation).isPresent()
                || odometryDiscrepancy > VisionConstants.MAX_POSE_DISCREPANCY_METERS
                || odometryDiscrepancy > VisionConstants.MAX_POSE_DELTA_METERS;

        if (!rejectPose && DriverStation.isTeleopEnabled()) {
          // Keep conservative vision rejection while traversing rough field geometry.
          // Pitch/roll is available from estimated pose here, not drivetrain IMU.
          double pitchAbs = Math.abs(Math.toDegrees(observation.pose().getRotation().getY()));
          double rollAbs = Math.abs(Math.toDegrees(observation.pose().getRotation().getX()));
          if (pitchAbs > 45.0 || rollAbs > 45.0) {
            rejectPose = true;
          }
        }

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

        Logger.recordOutput(
            "Vision/Camera" + cameraLabel + "/OdometryDiscrepancyMeters", odometryDiscrepancy);

        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        if (rejectPose) {
          continue;
        }

        Matrix<N3, N1> stdDevs =
            standardDeviations(observation, cameraIndex, aiming, inputs[cameraIndex].tagIds);

        consumer.accept(observation.pose().toPose2d(), observation.timestamp(), stdDevs);
        trackMultitagInitialization(
            observation, visionPose2d, VisionConstants.APTAG_CAMERA_NAMES[cameraIndex]);

        lastAcceptedPose[cameraIndex] = visionPose2d;
        lastAcceptedTimestamp[cameraIndex] = observation.timestamp();

        if (observation.timestamp() > lastAcceptedTimestampGlobal) {
          lastAcceptedPoseGlobal = visionPose2d;
          lastAcceptedTagIdsGlobal =
              Arrays.copyOf(inputs[cameraIndex].tagIds, inputs[cameraIndex].tagIds.length);
          lastAcceptedTimestampGlobal = observation.timestamp();
        }
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

    Logger.recordOutput("Vision/Aiming", aiming);

    Logger.recordOutput("Vision/Initialization/StableMultitagPoseCount", stableMultitagPoseCount);
    Logger.recordOutput("Vision/Initialization/Complete", visionInitializationComplete);

    maybeAutoReseedWhileDisabled();
  }

  private void maybeAutoReseedWhileDisabled() {
    boolean disabled = DriverStation.isDisabled();
    if (!disabled) {
      wasDisabledLastLoop = false;
      hasAutoReseededThisDisabledCycle = false;
      return;
    }

    Optional<AcceptedObservationSnapshot> snapshot = getLatestAcceptedObservationSnapshot();
    if (snapshot.isEmpty()) {
      wasDisabledLastLoop = true;
      return;
    }

    int tagCount = snapshot.get().tagIDs().length;
    if (tagCount < DISABLED_AUTO_RESEED_MIN_TAG_COUNT) {
      wasDisabledLastLoop = true;
      return;
    }

    Pose2d currentPose = robotPoseSupplier.get();
    Pose2d visionPose = snapshot.get().pose();
    double poseDeltaMeters = currentPose.getTranslation().getDistance(visionPose.getTranslation());
    double now = Timer.getFPGATimestamp();
    boolean needsInitialReseed = !hasAutoReseededThisDisabledCycle;
    boolean intervalElapsed =
        (now - lastDisabledAutoReseedTime) >= DISABLED_AUTO_RESEED_MIN_INTERVAL_SECONDS;
    boolean drifted = poseDeltaMeters > DISABLED_AUTO_RESEED_DELTA_METERS;

    if ((needsInitialReseed || drifted) && intervalElapsed) {
      poseResetConsumer.accept(visionPose);
      hasAutoReseededThisDisabledCycle = true;
      lastDisabledAutoReseedTime = now;
      markVisionInitializationComplete();
    }

    wasDisabledLastLoop = true;
  }

  private void markVisionInitializationComplete() {
    if (visionInitializationComplete) {
      return;
    }

    visionInitializationComplete = true;
    for (VisionIO visionIo : io) {
      if (visionIo instanceof VisionIOPhotonVision photonVisionIo) {
        photonVisionIo.markVisionInitializationComplete();
      }
    }
  }

  private void trackMultitagInitialization(
      VisionIO.PoseObservation observation, Pose2d pose2d, String cameraName) {
    if (visionInitializationComplete) {
      return;
    }

    boolean isMultitagCoprocessor = isMultitagInitCandidate(observation);
    if (!isMultitagCoprocessor) {
      stableMultitagPoseCount = 0;
      lastStableMultitagPose = null;
      lastStableMultitagTimestamp = Double.NEGATIVE_INFINITY;
      return;
    }

    double translationDelta = 0.0;
    double headingDeltaDeg = 0.0;
    boolean isStable = true;
    if (lastStableMultitagPose != null) {
      translationDelta =
          pose2d.getTranslation().getDistance(lastStableMultitagPose.getTranslation());
      headingDeltaDeg =
          Math.abs(pose2d.getRotation().minus(lastStableMultitagPose.getRotation()).getDegrees());
      isStable =
          isStableMultitagStep(
              observation.timestamp(),
              lastStableMultitagTimestamp,
              translationDelta,
              headingDeltaDeg);
    }

    stableMultitagPoseCount = nextStableMultitagPoseCount(stableMultitagPoseCount, isStable);
    lastStableMultitagPose = pose2d;
    lastStableMultitagTimestamp = observation.timestamp();

    if (stableMultitagPoseCount >= MULTITAG_INIT_STABLE_POSES_REQUIRED) {
      markVisionInitializationComplete();
    }
  }

  static boolean isMultitagInitCandidate(VisionIO.PoseObservation observation) {
    return observation.type() == VisionIO.PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR
        && observation.tagCount() >= DISABLED_AUTO_RESEED_MIN_TAG_COUNT;
  }

  static boolean isStableMultitagStep(
      double timestamp,
      double previousTimestamp,
      double translationDeltaMeters,
      double headingDeltaDegrees) {
    return timestamp > previousTimestamp
        && translationDeltaMeters <= MULTITAG_INIT_MAX_TRANSLATION_DELTA_METERS
        && headingDeltaDegrees <= MULTITAG_INIT_MAX_HEADING_DELTA_DEGREES;
  }

  static int nextStableMultitagPoseCount(int currentCount, boolean isStableStep) {
    return isStableStep ? (currentCount + 1) : 1;
  }

  static int requiredStableMultitagPosesForInitialization() {
    return MULTITAG_INIT_STABLE_POSES_REQUIRED;
  }

  public boolean forceReseedFromVision() {
    Optional<AcceptedObservationSnapshot> snapshot = getLatestAcceptedObservationSnapshot();
    if (snapshot.isEmpty()) {
      return false;
    }
    poseResetConsumer.accept(snapshot.get().pose());
    markVisionInitializationComplete();
    return true;
  }

  static Optional<String> rejectionReason(VisionIO.PoseObservation observation) {
    if (observation.tagCount() == 0) {
      return Optional.of("NO_TAGS");
    }

    Pose3d pose = observation.pose();
    if (Math.abs(pose.getZ()) > MAX_Z_ERROR) {
      return Optional.of("Z=" + pose.getZ());
    }

    Pose2d pose2d = pose.toPose2d();
    if (pose2d.getX() < 0.0
        || pose2d.getX() > FieldConstants.APTAG_FIELD_LAYOUT.getFieldLength()
        || pose2d.getY() < 0.0
        || pose2d.getY() > FieldConstants.APTAG_FIELD_LAYOUT.getFieldWidth()) {
      return Optional.of("OUT_OF_BOUNDS");
    }

    if (observation.tagCount() == 1 && observation.ambiguity() > SINGLE_TAG_MAX_AMBIGUITY) {
      return Optional.of("AMBIGUITY=" + observation.ambiguity());
    }

    if (observation.averageTagDistance() > MAX_AVG_TAG_DISTANCE_METERS) {
      return Optional.of("DISTANCE=" + observation.averageTagDistance());
    }

    return Optional.empty();
  }

  static Matrix<N3, N1> standardDeviations(
      VisionIO.PoseObservation observation, int cameraIndex, boolean aiming) {
    return standardDeviations(observation, cameraIndex, aiming, new int[0]);
  }

  static Matrix<N3, N1> standardDeviations(
      VisionIO.PoseObservation observation, int cameraIndex, boolean aiming, int[] cameraTagIds) {
    double tagCount = Math.max(observation.tagCount(), 1);
    double rawDistance = observation.averageTagDistance();
    double distance = rawDistance > 0.0 ? rawDistance : MAX_AVG_TAG_DISTANCE_METERS;
    double factor = (distance * distance) / tagCount;

    double cameraFactor =
        CAMERA_STDDEV_FACTORS[Math.min(cameraIndex, CAMERA_STDDEV_FACTORS.length - 1)];
    double aimFactor = aiming ? AIM_LINEAR_STDDEV_MULTIPLIER : 1.0;
    boolean coplanarPenaltyApplies = APPLY_COPLANAR_PENALTY && areTagsCoplanar(cameraTagIds);
    double singleTagFactor =
        (observation.tagCount() == 1 || coplanarPenaltyApplies)
            ? SINGLE_TAG_LINEAR_STDDEV_MULTIPLIER
            : 1.0;

    double linearStdDev =
        LINEAR_STDDEV_BASELINE * factor * cameraFactor * aimFactor * singleTagFactor;
    linearStdDev = Math.max(linearStdDev, 1e-6);

    return VecBuilder.fill(linearStdDev, linearStdDev, HEADING_STDDEV_IGNORE);
  }

  private static boolean areTagsCoplanar(int[] tagIDs) {
    if (tagIDs == null || tagIDs.length <= 1) {
      return true;
    }
    var firstOpt = FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(tagIDs[0]);
    if (firstOpt.isEmpty()) {
      return true;
    }
    Rotation3d referenceNormal = firstOpt.get().getRotation();
    double thresholdRad = Math.toRadians(COPLANAR_ANGLE_THRESHOLD_DEG);
    for (int i = 1; i < tagIDs.length; i++) {
      var tagOpt = FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(tagIDs[i]);
      if (tagOpt.isEmpty()) {
        continue;
      }
      Rotation3d diff = tagOpt.get().getRotation().minus(referenceNormal);
      if (Math.abs(diff.getAngle()) > thresholdRad) {
        return false;
      }
    }
    return true;
  }

  public static record AcceptedObservationSnapshot(Pose2d pose, int[] tagIDs, double timestamp) {}

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
