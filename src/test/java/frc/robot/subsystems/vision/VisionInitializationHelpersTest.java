package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;

class VisionInitializationHelpersTest {

  @Test
  void multitagInitializationCandidateRequiresCoprocessorMultiTagObservation() {
    VisionIO.PoseObservation candidate =
        new VisionIO.PoseObservation(
            1.0,
            new Pose3d(1.0, 2.0, 0.0, new Rotation3d()),
            0.05,
            2,
            2.0,
            VisionIO.PoseObservationType.PHOTONVISION_MULTITAG_COPROCESSOR);

    VisionIO.PoseObservation nonCandidate =
        new VisionIO.PoseObservation(
            1.0,
            new Pose3d(1.0, 2.0, 0.0, new Rotation3d()),
            0.05,
            2,
            2.0,
            VisionIO.PoseObservationType.PHOTONVISION);

    assertTrue(Vision.isMultitagInitCandidate(candidate));
    assertFalse(Vision.isMultitagInitCandidate(nonCandidate));
  }

  @Test
  void stableMultitagSequenceRequiresMonotonicTimeAndSmallMotion() {
    assertTrue(Vision.isStableMultitagStep(2.0, 1.0, 0.05, 2.0));
    assertFalse(Vision.isStableMultitagStep(1.0, 2.0, 0.05, 2.0));
    assertFalse(Vision.isStableMultitagStep(2.0, 1.0, 0.25, 2.0));
    assertFalse(Vision.isStableMultitagStep(2.0, 1.0, 0.05, 15.0));
  }

  @Test
  void stableMultitagCountResetsOnUnstableStep() {
    assertEquals(4, Vision.nextStableMultitagPoseCount(3, true));
    assertEquals(1, Vision.nextStableMultitagPoseCount(3, false));
    assertEquals(5, Vision.requiredStableMultitagPosesForInitialization());
  }
}
