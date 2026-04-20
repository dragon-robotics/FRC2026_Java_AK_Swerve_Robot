// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.util.constants.FieldConstants.APTAG_FIELD_LAYOUT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;
  private static double lastSimUpdateTimestamp = -1.0;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(APTAG_FIELD_LAYOUT);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(800, 600, Rotation2d.fromDegrees(72));
    cameraProperties.setCalibError(0.38, 0.1);
    cameraProperties.setFPS(45);
    cameraProperties.setAvgLatencyMs(10);
    cameraProperties.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProperties, APTAG_FIELD_LAYOUT);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Only update the shared VisionSystemSim once per robot cycle.
    // With 4 cameras sharing the same static sim, calling update() on every
    // camera would re-render all cameras N times (16 renders instead of 4).
    double now = Timer.getFPGATimestamp();
    if (now != lastSimUpdateTimestamp) {
      visionSim.update(poseSupplier.get());
      lastSimUpdateTimestamp = now;
    }
    super.updateInputs(inputs);
  }
}
