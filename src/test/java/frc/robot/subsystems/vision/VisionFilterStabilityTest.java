package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import org.junit.jupiter.api.Test;

class VisionFilterStabilityTest {

  private static final double DT = 0.02;
  private static final int CYCLES = 250;
  private static final double MAX_SINGLE_CYCLE_JUMP_M = 0.5;
  private static final double MAX_MEAN_DISCREPANCY_M = 0.3;

  private static final Pose2d GROUND_TRUTH = new Pose2d(4.0, 4.0, new Rotation2d());

  @Test
  void fusedPoseStaysStableUnderRealisticVisionNoise() throws IOException {
    Random rng = new Random(42);
    Rotation2d gyro = new Rotation2d();
    SwerveModulePosition[] modules = zeroedModules();
    SwerveDrivePoseEstimator estimator =
        new SwerveDrivePoseEstimator(dummyKinematics(), gyro, modules, GROUND_TRUTH);

    List<String> csv = new ArrayList<>();
    csv.add("cycle,t,fusedX,fusedY,jump,injected,acceptedThisCycle,lastReason");

    Pose2d prev = estimator.getEstimatedPosition();
    double maxJump = 0.0;
    double sumDiscrepancy = 0.0;
    int acceptedCount = 0;
    int rejectedCount = 0;

    for (int cycle = 0; cycle < CYCLES; cycle++) {
      double t = cycle * DT;

      estimator.updateWithTime(t, gyro, modules);

      List<PoseObservation> stream = new ArrayList<>();
      stream.add(
          VisionScenarios.goodMultiTag(
              noisy(rng, GROUND_TRUTH.getX()), noisy(rng, GROUND_TRUTH.getY()), 0.0, t));
      stream.add(
          VisionScenarios.goodMultiTag(
              noisy(rng, GROUND_TRUTH.getX()), noisy(rng, GROUND_TRUTH.getY()), 0.0, t));

      String injected = "good";
      if (cycle % 25 == 0) {
        stream.add(
            VisionScenarios.flippedSingleTag(
                GROUND_TRUTH.getX() + 3.0, GROUND_TRUTH.getY() + 3.0, Math.PI, t));
        injected = "flipped";
      }
      if (cycle % 17 == 0) {
        stream.add(VisionScenarios.outOfBounds(t));
        stream.add(VisionScenarios.highZ(GROUND_TRUTH.getX(), GROUND_TRUTH.getY(), t));
        stream.add(VisionScenarios.tooFar(GROUND_TRUTH.getX(), GROUND_TRUTH.getY(), t));
        stream.add(
            VisionScenarios.singleTagHighAmbiguity(GROUND_TRUTH.getX(), GROUND_TRUTH.getY(), t));
      }

      String lastReason = "-";
      boolean acceptedThisCycle = false;
      for (PoseObservation obs : stream) {
        Optional<String> reason = Vision.rejectionReason(obs);
        if (reason.isPresent()) {
          rejectedCount++;
          lastReason = reason.get();
          continue;
        }
        acceptedThisCycle = true;
        acceptedCount++;
        estimator.addVisionMeasurement(
            obs.pose().toPose2d(), obs.timestamp(), Vision.standardDeviations(obs, 0, false));
      }

      Pose2d fused = estimator.getEstimatedPosition();
      double jump = fused.getTranslation().getDistance(prev.getTranslation());
      maxJump = Math.max(maxJump, jump);
      sumDiscrepancy += fused.getTranslation().getDistance(GROUND_TRUTH.getTranslation());
      prev = fused;

      csv.add(
          String.format(
              "%d,%.3f,%.4f,%.4f,%.4f,%s,%b,%s",
              cycle, t, fused.getX(), fused.getY(), jump, injected, acceptedThisCycle, lastReason));
    }

    double meanDiscrepancy = sumDiscrepancy / CYCLES;
    writeCsv("filter-test.csv", csv);

    assertTrue(acceptedCount > 0);
    assertTrue(rejectedCount > 0);
    assertTrue(maxJump <= MAX_SINGLE_CYCLE_JUMP_M);
    assertTrue(meanDiscrepancy <= MAX_MEAN_DISCREPANCY_M);
  }

  @Test
  void adversarialObservationsAreRejected() {
    assertTrue(Vision.rejectionReason(VisionScenarios.outOfBounds(0.0)).isPresent());
    assertTrue(Vision.rejectionReason(VisionScenarios.highZ(4.0, 4.0, 0.0)).isPresent());
    assertTrue(Vision.rejectionReason(VisionScenarios.tooFar(4.0, 4.0, 0.0)).isPresent());
    assertTrue(
        Vision.rejectionReason(VisionScenarios.singleTagHighAmbiguity(4.0, 4.0, 0.0)).isPresent());
  }

  @Test
  void goodMultiTagObservationsAreAccepted() {
    assertTrue(Vision.rejectionReason(VisionScenarios.goodMultiTag(4.0, 4.0, 0.0, 0.0)).isEmpty());
  }

  @Test
  void singleTagTranslationStdDevAppliesTheConfiguredMultiplier() {
    double distance = 2.0;
    Pose3d pose = new Pose3d(4.0, 4.0, 0.0, new Rotation3d());
    PoseObservation singleTag =
        new PoseObservation(0.0, pose, 0.1, 1, distance, PoseObservationType.PHOTONVISION);

    double singleStdDev = Vision.standardDeviations(singleTag, 0, false).get(0, 0);

    double singleTagBaseline = VisionConstants.LINEAR_STDDEV_BASELINE * distance * distance;
    double observedMultiplier = singleStdDev / singleTagBaseline;

    assertEquals(
        VisionConstants.SINGLE_TAG_LINEAR_STDDEV_MULTIPLIER,
        observedMultiplier,
        1e-9,
        "Single-tag translation std-dev must apply configured distrust multiplier");
  }

  @Test
  void hybridOrderDropsConstrainedAtHighAngularRate() {
    var order = VisionIOPhotonVision.hybridStrategyOrderForTest(1, 0.0, 0.75);
    assertTrue(
        java.util.Arrays.stream(order)
            .noneMatch(
                s -> s == org.photonvision.PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP));
  }

  private static SwerveDriveKinematics dummyKinematics() {
    double o = 0.3;
    return new SwerveDriveKinematics(
        new Translation2d(o, o),
        new Translation2d(o, -o),
        new Translation2d(-o, o),
        new Translation2d(-o, -o));
  }

  private static SwerveModulePosition[] zeroedModules() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
  }

  private static double noisy(Random rng, double value) {
    return value + (rng.nextDouble() - 0.5) * 0.04;
  }

  private static void writeCsv(String name, List<String> lines) throws IOException {
    Path dir = Path.of("build", "vision-stability");
    Files.createDirectories(dir);
    Files.write(dir.resolve(name), lines);
  }
}
