// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.constants.FieldConstants;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator poseEstimator;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.poseEstimator = new PhotonPoseEstimator(FieldConstants.APTAG_FIELD_LAYOUT, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      // Track all visible tag IDs for logging
      for (var target : result.targets) {
        tagIds.add((short) target.getFiducialId());
      }

      // Estimate pose: prefer coprocessor multi-tag, fall back to lowest-ambiguity single-tag.
      // estimateCoprocMultiTagPose uses the onboard PnP result sent from the coprocessor.
      // estimateLowestAmbiguityPose picks the single-tag transform with the lowest ambiguity score.
      Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);
      if (estimate.isEmpty()) {
        estimate = poseEstimator.estimateLowestAmbiguityPose(result);
      }

      estimate.ifPresent(
          est -> {
            // Average distance from robot to each tag used in the estimate
            double avgDist = 0;
            for (var tgt : est.targetsUsed) {
              var tagPose = FieldConstants.APTAG_FIELD_LAYOUT.getTagPose(tgt.getFiducialId());
              if (tagPose.isPresent()) {
                avgDist +=
                    tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(est.estimatedPose.toPose2d().getTranslation());
              }
            }
            int numTags = est.targetsUsed.size();
            if (numTags > 0) avgDist /= numTags;

            double ambiguity =
                result.hasTargets() ? result.getBestTarget().getPoseAmbiguity() : 0.0;

            poseObservations.add(
                new PoseObservation(
                    est.timestampSeconds,
                    est.estimatedPose,
                    ambiguity,
                    numTags,
                    avgDist,
                    PoseObservationType.PHOTONVISION));
          });
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    // Save tag IDs to inputs object
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
