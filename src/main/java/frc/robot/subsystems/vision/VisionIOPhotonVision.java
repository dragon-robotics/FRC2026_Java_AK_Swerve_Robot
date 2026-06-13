// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator poseEstimator;
  private VisionHeadingProvider headingProvider;

  private static final TargetObservation NO_TARGET =
      new TargetObservation(new Rotation2d(), new Rotation2d());

  private static final int MAX_RESULTS_PER_UPDATE = 2;
  private static final String STRATEGY_MODE_PROPERTY = "vision.photon.strategyMode";
  private static final String HYBRID_STRATEGY_MODE = "HYBRID";
  private static final double HYBRID_TRANSLATION_SPEED_THRESHOLD_METERS_PER_SECOND = 0.5;

  public interface VisionHeadingProvider {
    Optional<Rotation2d> getHeadingAtTimestamp(double fpgaTimestampSeconds);

    Optional<edu.wpi.first.math.geometry.Pose3d> getSeedPoseAtTimestamp(
        double fpgaTimestampSeconds);

    double getAngularRateRadPerSec();

    double getLinearSpeedMetersPerSecond();
  }

  private final List<PoseObservation> poseObservations = new ArrayList<>(4);
  private int[] tagIdBuffer = new int[16];
  private int tagIdCount = 0;
  private boolean preferMultitagUntilInitialized = true;

  private static final PoseObservation[] EMPTY_POSE_OBSERVATIONS = new PoseObservation[0];
  private static final int[] EMPTY_TAG_IDS = new int[0];

  private static record StrategyEstimate(EstimatedRobotPose estimate, PoseObservationType type) {}

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.poseEstimator =
        new PhotonPoseEstimator(
            frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT, robotToCamera);
  }

  public String getCameraName() {
    return camera.getName();
  }

  public void setHeadingProvider(VisionHeadingProvider headingProvider) {
    this.headingProvider = headingProvider;
  }

  public void markVisionInitializationComplete() {
    preferMultitagUntilInitialized = false;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    poseObservations.clear();
    tagIdCount = 0;

    var allResults = camera.getAllUnreadResults();
    if (allResults.isEmpty()) {
      inputs.poseObservations = EMPTY_POSE_OBSERVATIONS;
      inputs.tagIds = EMPTY_TAG_IDS;
      inputs.latestTargetObservation = NO_TARGET;
      return;
    }

    int startIndex = Math.max(0, allResults.size() - MAX_RESULTS_PER_UPDATE);
    for (int resultIndex = startIndex; resultIndex < allResults.size(); resultIndex++) {
      processResult(allResults.get(resultIndex), inputs);
    }

    inputs.poseObservations =
        poseObservations.isEmpty()
            ? EMPTY_POSE_OBSERVATIONS
            : poseObservations.toArray(EMPTY_POSE_OBSERVATIONS);
    inputs.tagIds = tagIdCount == 0 ? EMPTY_TAG_IDS : Arrays.copyOf(tagIdBuffer, tagIdCount);
  }

  private void processResult(PhotonPipelineResult result, VisionIOInputs inputs) {
    if (!result.hasTargets()) {
      inputs.latestTargetObservation = NO_TARGET;
      return;
    }

    PhotonTrackedTarget best = result.getBestTarget();
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(best.getYaw()), Rotation2d.fromDegrees(best.getPitch()));

    List<PhotonTrackedTarget> targets = result.getTargets();
    if (targets.isEmpty() || allTargetsBeyondMaxRange(targets)) {
      return;
    }

    Optional<StrategyEstimate> strategyEstimate = estimateWithConfiguredStrategies(result);
    strategyEstimate.ifPresent(
        estimate ->
            addPoseObservation(
                estimate.estimate(), estimate.estimate().targetsUsed, estimate.type()));
  }

  private Optional<StrategyEstimate> estimateWithConfiguredStrategies(PhotonPipelineResult result) {
    for (PoseStrategy strategy : resolveStrategyOrder(result)) {
      Optional<EstimatedRobotPose> estimate;
      switch (strategy) {
        case MULTI_TAG_PNP_ON_COPROCESSOR:
          estimate = poseEstimator.estimateCoprocMultiTagPose(result);
          break;
        case CONSTRAINED_SOLVEPNP:
          estimate = estimateConstrainedFallbackPose(result);
          break;
        case PNP_DISTANCE_TRIG_SOLVE:
          estimate = estimatePnpDistanceTrigSolvePose(result);
          break;
        case LOWEST_AMBIGUITY:
          estimate = poseEstimator.estimateLowestAmbiguityPose(result);
          break;
        default:
          estimate = Optional.empty();
          break;
      }
      if (estimate.isPresent()) {
        PoseObservationType type =
            strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                ? PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR
                : PoseObservationType.PHOTONVISION;
        return Optional.of(new StrategyEstimate(estimate.get(), type));
      }
    }
    return Optional.empty();
  }

  private PoseStrategy[] resolveStrategyOrder(PhotonPipelineResult result) {
    if (preferMultitagUntilInitialized) {
      return new PoseStrategy[] {
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseStrategy.CONSTRAINED_SOLVEPNP,
        PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    if (HYBRID_STRATEGY_MODE.equalsIgnoreCase(System.getProperty(STRATEGY_MODE_PROPERTY, ""))) {
      return resolveHybridStrategyOrder(result);
    }

    return parseStrategyOrder(
        System.getProperty("vision.photon.strategyOrder", PHOTON_POSE_STRATEGY_ORDER));
  }

  private PoseStrategy[] resolveHybridStrategyOrder(PhotonPipelineResult result) {
    double linearSpeedMetersPerSecond =
        headingProvider == null ? 0.0 : headingProvider.getLinearSpeedMetersPerSecond();
    double angularRateRadPerSec =
        headingProvider == null ? 0.0 : Math.abs(headingProvider.getAngularRateRadPerSec());
    int visibleTargetCount = result.getTargets().size();

    return hybridStrategyOrderForTest(
        visibleTargetCount, linearSpeedMetersPerSecond, angularRateRadPerSec);
  }

  static PoseStrategy[] hybridStrategyOrderForTest(
      int visibleTargetCount, double linearSpeedMetersPerSecond, double angularRateRadPerSec) {

    if (angularRateRadPerSec > CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC) {
      if (visibleTargetCount >= 2) {
        return new PoseStrategy[] {
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
          PoseStrategy.LOWEST_AMBIGUITY
        };
      }
      return new PoseStrategy[] {
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    if (visibleTargetCount >= 2) {
      return new PoseStrategy[] {
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseStrategy.CONSTRAINED_SOLVEPNP,
        PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    if (linearSpeedMetersPerSecond > HYBRID_TRANSLATION_SPEED_THRESHOLD_METERS_PER_SECOND) {
      return new PoseStrategy[] {
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseStrategy.CONSTRAINED_SOLVEPNP,
        PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    return new PoseStrategy[] {
      PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
      PoseStrategy.CONSTRAINED_SOLVEPNP,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      PoseStrategy.LOWEST_AMBIGUITY
    };
  }

  private Optional<EstimatedRobotPose> estimatePnpDistanceTrigSolvePose(
      PhotonPipelineResult result) {
    if (headingProvider == null) {
      return Optional.empty();
    }

    if (Math.abs(headingProvider.getAngularRateRadPerSec()) > TRIG_MAX_ANGULAR_RATE_RAD_PER_SEC) {
      return Optional.empty();
    }

    Optional<Rotation2d> headingSample =
        headingProvider.getHeadingAtTimestamp(result.getTimestampSeconds());
    if (headingSample.isEmpty()) {
      return Optional.empty();
    }

    poseEstimator.addHeadingData(result.getTimestampSeconds(), headingSample.get());
    return poseEstimator.estimatePnpDistanceTrigSolvePose(result);
  }

  private Optional<EstimatedRobotPose> estimateConstrainedFallbackPose(
      PhotonPipelineResult result) {
    if (!ENABLE_CONSTRAINED_FALLBACK || headingProvider == null) {
      return Optional.empty();
    }

    if (Math.abs(headingProvider.getAngularRateRadPerSec())
        > CONSTRAINED_MAX_ANGULAR_RATE_RAD_PER_SEC) {
      return Optional.empty();
    }

    Optional<Rotation2d> headingSample =
        headingProvider.getHeadingAtTimestamp(result.getTimestampSeconds());
    if (headingSample.isEmpty()) {
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> seedEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
    Optional<edu.wpi.first.math.geometry.Pose3d> seedPose =
        seedEstimate.map(estimate -> estimate.estimatedPose);
    if (seedPose.isEmpty()) {
      seedPose = headingProvider.getSeedPoseAtTimestamp(result.getTimestampSeconds());
    }
    if (seedPose.isEmpty()) {
      return Optional.empty();
    }

    Optional<Matrix<N3, N3>> cameraMatrix = camera.getCameraMatrix();
    Optional<Matrix<N8, N1>> distCoeffs = camera.getDistCoeffs();
    if (cameraMatrix.isEmpty() || distCoeffs.isEmpty()) {
      return Optional.empty();
    }

    poseEstimator.addHeadingData(result.getTimestampSeconds(), headingSample.get());

    return poseEstimator.estimateConstrainedSolvepnpPose(
        result,
        cameraMatrix.get(),
        distCoeffs.get(),
        seedPose.get(),
        false,
        CONSTRAINED_HEADING_SCALE_FACTOR);
  }

  private static PoseStrategy[] parseStrategyOrder(String rawOrder) {
    List<PoseStrategy> parsed = new ArrayList<>();
    for (String token : rawOrder.split(",")) {
      String candidate = token.trim();
      if (candidate.isEmpty()) {
        continue;
      }
      try {
        PoseStrategy strategy = PoseStrategy.valueOf(candidate);
        if (strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            || strategy == PoseStrategy.CONSTRAINED_SOLVEPNP
            || strategy == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
            || strategy == PoseStrategy.LOWEST_AMBIGUITY) {
          parsed.add(strategy);
        }
      } catch (IllegalArgumentException ignored) {
        // Ignore unknown strategy names from the property string.
      }
    }

    if (parsed.isEmpty()) {
      return new PoseStrategy[] {
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseStrategy.CONSTRAINED_SOLVEPNP,
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        PoseStrategy.LOWEST_AMBIGUITY
      };
    }

    return parsed.toArray(new PoseStrategy[0]);
  }

  private boolean allTargetsBeyondMaxRange(List<PhotonTrackedTarget> targets) {
    for (PhotonTrackedTarget target : targets) {
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      if (cameraToTarget != null && cameraToTarget.getTranslation().getNorm() <= MAX_TAG_DISTANCE) {
        return false;
      }
    }
    return true;
  }

  private void addPoseObservation(
      EstimatedRobotPose estimatedPose,
      List<PhotonTrackedTarget> targets,
      PoseObservationType observationType) {
    int observedTagCount = 0;
    int distanceSampleCountAll = 0;
    int distanceSampleCountInRange = 0;
    double totalDistanceAll = 0.0;
    double totalDistanceInRange = 0.0;
    double totalAmbiguity = 0.0;

    for (PhotonTrackedTarget target : targets) {
      int tagId = target.getFiducialId();
      if (tagId <= 0) {
        continue;
      }

      observedTagCount++;
      addTagId(tagId);

      Transform3d cameraToTarget = target.getBestCameraToTarget();
      if (cameraToTarget != null) {
        double distanceMeters = cameraToTarget.getTranslation().getNorm();
        totalDistanceAll += distanceMeters;
        distanceSampleCountAll++;
        if (distanceMeters <= MAX_AVG_TAG_DISTANCE_METERS) {
          totalDistanceInRange += distanceMeters;
          distanceSampleCountInRange++;
        }
      }

      totalAmbiguity += Math.max(0.0, target.getPoseAmbiguity());
    }

    if (observedTagCount == 0) {
      return;
    }

    double averageTagDistanceMeters;
    if (distanceSampleCountInRange > 0) {
      averageTagDistanceMeters = totalDistanceInRange / distanceSampleCountInRange;
    } else if (distanceSampleCountAll > 0) {
      averageTagDistanceMeters = totalDistanceAll / distanceSampleCountAll;
    } else {
      averageTagDistanceMeters = Double.POSITIVE_INFINITY;
    }

    poseObservations.add(
        new PoseObservation(
            estimatedPose.timestampSeconds,
            estimatedPose.estimatedPose,
            totalAmbiguity / observedTagCount,
            observedTagCount,
            averageTagDistanceMeters,
            observationType));
  }

  private void addTagId(int tagId) {
    for (int i = 0; i < tagIdCount; i++) {
      if (tagIdBuffer[i] == tagId) {
        return;
      }
    }

    if (tagIdCount >= tagIdBuffer.length) {
      tagIdBuffer = Arrays.copyOf(tagIdBuffer, tagIdBuffer.length * 2);
    }
    tagIdBuffer[tagIdCount++] = tagId;
  }
}
