package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("sim")
class VisionDynamicStrategyBakeoffTest {

  private static final double DT = 0.02;
  private static final int WARMUP_CYCLES = 50;
  private static final int MEASURE_CYCLES = 250;
  private static final double DEFAULT_SPIN_RATE_RAD_PER_SEC = Units.rotationsToRadians(0.75);
  private static final double SHUTTLE_SPEED_METERS_PER_SEC =
      TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond) * 0.8;
  private static final double SHUTTLE_MIN_Y = 2.5;
  private static final double SHUTTLE_MAX_Y = 5.5;
  private static final double MAX_EXCESS_JUMP_M = 0.25;
  private static final double CATASTROPHIC_EXCESS_JUMP_M = 0.40;
  private static final int MAX_CATASTROPHIC_OUTLIERS = 0;
  private static final double MIN_VISION_COVERAGE = 0.30;

  private static final List<StrategyConfig> STRATEGIES =
      List.of(
          new StrategyConfig(
              "MULTI_TAG_PNP_ON_COPROCESSOR",
              "MULTI_TAG_PNP_ON_COPROCESSOR,LOWEST_AMBIGUITY",
              false),
          new StrategyConfig(
              "CONSTRAINED_SOLVEPNP", "CONSTRAINED_SOLVEPNP,LOWEST_AMBIGUITY", false),
          new StrategyConfig(
              "PNP_DISTANCE_TRIG_SOLVE", "PNP_DISTANCE_TRIG_SOLVE,LOWEST_AMBIGUITY", false));

  private static final StrategyConfig HYBRID_STRATEGY = new StrategyConfig("HYBRID", "", true);

  private static boolean halReady = false;
  private static RobotContainer container;

  private record DynamicScenario(
      String name, Pose2d startPose, boolean shuttle, double angularRateRadPerSec) {}

  private record StrategyConfig(String name, String order, boolean hybridMode) {}

  private record GroundTruthState(Pose2d pose, boolean movingTowardLow) {}

  private record MotionStep(
      GroundTruthState nextTruth,
      double fieldVx,
      double fieldVy,
      double omegaRadiansPerSecond,
      double expectedJumpMeters) {}

  private record BakeoffResult(
      DynamicScenario scenario,
      List<VisionStrategyComparisonSupport.StrategySummary> strategySummaries,
      List<VisionStrategyComparisonSupport.RankedStrategy> rankedStrategies) {

    List<String> strategyNames() {
      return strategySummaries.stream()
          .map(VisionStrategyComparisonSupport.StrategySummary::strategyName)
          .toList();
    }
  }

  @FunctionalInterface
  private interface ThrowingSupplier<T> {
    T get() throws IOException;
  }

  @BeforeAll
  static void setUpHal() {
    try {
      halReady = HAL.initialize(500, 0);
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      container = new RobotContainer();
    } catch (Throwable t) {
      halReady = false;
    }
  }

  @AfterAll
  static void tearDownHal() {
    try {
      CommandScheduler.getInstance().cancelAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    } catch (Throwable ignored) {
    }
    if (halReady) {
      HAL.shutdown();
    }
  }

  @Test
  void bakeoffSupportCanRankStrategiesHardGatesFirst() {
    var constrained =
        new VisionStrategyComparisonSupport.StrategySummary(
            "CONSTRAINED_SOLVEPNP", 0.08, 0, 0.92, 0.60, 0.22);
    var trig =
        new VisionStrategyComparisonSupport.StrategySummary(
            "PNP_DISTANCE_TRIG_SOLVE", 0.20, 1, 0.95, 0.55, 0.18);

    var ranked =
        VisionStrategyComparisonSupport.rankStrategies(List.of(constrained, trig), 0.15, 0, 0.50);

    assertEquals(2, ranked.size());
    assertEquals("CONSTRAINED_SOLVEPNP", ranked.get(0).strategyName());
    assertTrue(ranked.get(0).passesHardGates());
    assertFalse(ranked.get(1).passesHardGates());
  }

  @Test
  void leftSpinBakeoffProducesSummariesForAllStrategies() throws IOException {
    var result = runBakeoff(leftSpinScenario());

    assertEquals(3, result.strategySummaries().size());
    assertTrue(result.strategyNames().contains("MULTI_TAG_PNP_ON_COPROCESSOR"));
    assertTrue(result.strategyNames().contains("CONSTRAINED_SOLVEPNP"));
    assertTrue(result.strategyNames().contains("PNP_DISTANCE_TRIG_SOLVE"));
  }

  @Test
  void hybridBakeoffIncludesHybridAlongsideThreeSingleStrategies() throws IOException {
    var result = runBakeoffWithHybrid(shuttleSpinScenario());

    assertEquals(4, result.strategySummaries().size());
    assertTrue(result.strategyNames().contains("HYBRID"));
  }

  @Test
  void hybridOrderDropsConstrainedAtHighAngularRate() {
    var order = VisionIOPhotonVision.hybridStrategyOrderForTest(1, 0.0, 0.75);

    assertFalse(
        List.of(order)
            .contains(org.photonvision.PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP));
  }

  private static DynamicScenario leftSpinScenario() {
    return new DynamicScenario(
        "left_spin", new Pose2d(2.5, 5.5, Rotation2d.kZero), false, DEFAULT_SPIN_RATE_RAD_PER_SEC);
  }

  private static DynamicScenario shuttleSpinScenario() {
    return new DynamicScenario(
        "shuttle_spin",
        new Pose2d(2.5, 5.5, Rotation2d.kZero),
        true,
        DEFAULT_SPIN_RATE_RAD_PER_SEC);
  }

  private BakeoffResult runBakeoff(DynamicScenario scenario) throws IOException {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");
    Assumptions.assumeTrue(container != null, "RobotContainer failed to initialize");

    List<VisionStrategyComparisonSupport.StrategySummary> summaries = new ArrayList<>();
    for (StrategyConfig strategy : STRATEGIES) {
      summaries.add(runScenarioForStrategy(scenario, strategy));
    }

    return new BakeoffResult(
        scenario,
        summaries,
        VisionStrategyComparisonSupport.rankStrategies(
            summaries, MAX_EXCESS_JUMP_M, MAX_CATASTROPHIC_OUTLIERS, MIN_VISION_COVERAGE));
  }

  private BakeoffResult runBakeoffWithHybrid(DynamicScenario scenario) throws IOException {
    List<VisionStrategyComparisonSupport.StrategySummary> summaries = new ArrayList<>();
    for (StrategyConfig strategy : STRATEGIES) {
      summaries.add(runScenarioForStrategy(scenario, strategy));
    }
    summaries.add(runScenarioForStrategy(scenario, HYBRID_STRATEGY));

    return new BakeoffResult(
        scenario,
        summaries,
        VisionStrategyComparisonSupport.rankStrategies(
            summaries, MAX_EXCESS_JUMP_M, MAX_CATASTROPHIC_OUTLIERS, MIN_VISION_COVERAGE));
  }

  private VisionStrategyComparisonSupport.StrategySummary runScenarioForStrategy(
      DynamicScenario scenario, StrategyConfig strategy) throws IOException {
    return withStrategyConfig(
        strategy,
        () -> {
          CommandScheduler.getInstance().cancelAll();
          container.getDrive().setPose(scenario.startPose());

          List<String> csv = new ArrayList<>();
          csv.add(
              "cycle,phase,truthX,truthY,truthYawDeg,odomX,odomY,odomYawDeg,visionX,visionY,visionYawDeg,"
                  + "odomJump,expectedJump,excessJump,visionDeviation,hasVision,strategy");

          GroundTruthState truthState = new GroundTruthState(scenario.startPose(), true);
          Pose2d previousOdom = scenario.startPose();
          double maxExcessJump = 0.0;
          int catastrophicOutliers = 0;
          int visionAcceptedCycles = 0;
          double maxVisionDeviation = 0.0;
          double sumVisionDeviation = 0.0;
          Set<String> acceptedTagSignatures = new LinkedHashSet<>();

          int totalCycles = WARMUP_CYCLES + MEASURE_CYCLES;
          for (int cycle = 0; cycle < totalCycles; cycle++) {
            CommandScheduler.getInstance().run();

            MotionStep motionStep = advanceGroundTruth(scenario, truthState);
            ChassisSpeeds robotRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    motionStep.fieldVx(),
                    motionStep.fieldVy(),
                    motionStep.omegaRadiansPerSecond(),
                    truthState.pose().getRotation());
            container.getDrive().runVelocity(robotRelativeSpeeds);

            SimHooks.stepTiming(DT);

            truthState = motionStep.nextTruth();
            Pose2d odom = container.getDrive().getPose();
            Optional<Vision.AcceptedObservationSnapshot> vis =
                container.getVision().getLatestAcceptedObservationSnapshot();

            double odomJump = odom.getTranslation().getDistance(previousOdom.getTranslation());
            double excessJump = Math.max(0.0, odomJump - motionStep.expectedJumpMeters());
            boolean measuring = cycle >= WARMUP_CYCLES;

            double visionDeviation = Double.NaN;
            if (vis.isPresent()) {
              visionDeviation =
                  vis.get().pose().getTranslation().getDistance(truthState.pose().getTranslation());
              if (measuring) {
                visionAcceptedCycles++;
                sumVisionDeviation += visionDeviation;
                maxVisionDeviation = Math.max(maxVisionDeviation, visionDeviation);
                acceptedTagSignatures.add(java.util.Arrays.toString(vis.get().tagIDs()));
              }
            }

            if (measuring) {
              maxExcessJump = Math.max(maxExcessJump, excessJump);
              if (excessJump > CATASTROPHIC_EXCESS_JUMP_M) {
                catastrophicOutliers++;
              }
            }

            csv.add(
                String.format(
                    Locale.US,
                    "%d,%s,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%b,%s",
                    cycle,
                    measuring ? "measure" : "warmup",
                    truthState.pose().getX(),
                    truthState.pose().getY(),
                    truthState.pose().getRotation().getDegrees(),
                    odom.getX(),
                    odom.getY(),
                    odom.getRotation().getDegrees(),
                    vis.map(v -> v.pose().getX()).orElse(Double.NaN),
                    vis.map(v -> v.pose().getY()).orElse(Double.NaN),
                    vis.map(v -> v.pose().getRotation().getDegrees()).orElse(Double.NaN),
                    odomJump,
                    motionStep.expectedJumpMeters(),
                    excessJump,
                    visionDeviation,
                    vis.isPresent(),
                    strategy.name()));

            previousOdom = odom;
          }

          writeCsv(
              "dynamic-" + scenario.name() + "-" + strategy.name().toLowerCase() + ".csv", csv);

          double visionCoverageRatio = visionAcceptedCycles / (double) MEASURE_CYCLES;
          double meanVisionDeviation =
              visionAcceptedCycles == 0
                  ? Double.POSITIVE_INFINITY
                  : sumVisionDeviation / visionAcceptedCycles;
          var summary =
              new VisionStrategyComparisonSupport.StrategySummary(
                  strategy.name(),
                  maxExcessJump,
                  catastrophicOutliers,
                  visionCoverageRatio,
                  maxVisionDeviation,
                  meanVisionDeviation);

          System.out.println(
              VisionStrategyComparisonSupport.formatSummaryLine(
                  "[VisionDynamic]", scenario.name(), summary));
          System.out.printf(
              "[VisionDynamicTags]|%s|%s acceptedTags=%s%n",
              scenario.name(),
              strategy.name(),
              acceptedTagSignatures.isEmpty() ? "-" : String.join(";", acceptedTagSignatures));
          return summary;
        });
  }

  private MotionStep advanceGroundTruth(DynamicScenario scenario, GroundTruthState current) {
    Rotation2d nextHeading =
        current
            .pose()
            .getRotation()
            .plus(Rotation2d.fromRadians(scenario.angularRateRadPerSec() * DT));
    if (!scenario.shuttle()) {
      return new MotionStep(
          new GroundTruthState(
              new Pose2d(current.pose().getTranslation(), nextHeading), current.movingTowardLow()),
          0.0,
          0.0,
          scenario.angularRateRadPerSec(),
          0.0);
    }

    double direction = current.movingTowardLow() ? -1.0 : 1.0;
    double unclampedY = current.pose().getY() + (direction * SHUTTLE_SPEED_METERS_PER_SEC * DT);
    double nextY = Math.max(SHUTTLE_MIN_Y, Math.min(SHUTTLE_MAX_Y, unclampedY));
    boolean nextMovingTowardLow = current.movingTowardLow();
    if (nextY <= SHUTTLE_MIN_Y) {
      nextMovingTowardLow = false;
    } else if (nextY >= SHUTTLE_MAX_Y) {
      nextMovingTowardLow = true;
    }

    Pose2d nextPose = new Pose2d(current.pose().getX(), nextY, nextHeading);
    double expectedJump = nextPose.getTranslation().getDistance(current.pose().getTranslation());
    return new MotionStep(
        new GroundTruthState(nextPose, nextMovingTowardLow),
        0.0,
        direction * SHUTTLE_SPEED_METERS_PER_SEC,
        scenario.angularRateRadPerSec(),
        expectedJump);
  }

  private static <T> T withStrategyConfig(StrategyConfig strategy, ThrowingSupplier<T> supplier)
      throws IOException {
    String previousOrder = System.getProperty("vision.photon.strategyOrder");
    String previousMode = System.getProperty("vision.photon.strategyMode");
    if (strategy.hybridMode()) {
      System.setProperty("vision.photon.strategyMode", "HYBRID");
      System.clearProperty("vision.photon.strategyOrder");
    } else {
      System.clearProperty("vision.photon.strategyMode");
      System.setProperty("vision.photon.strategyOrder", strategy.order());
    }
    try {
      return supplier.get();
    } finally {
      if (previousOrder == null) {
        System.clearProperty("vision.photon.strategyOrder");
      } else {
        System.setProperty("vision.photon.strategyOrder", previousOrder);
      }
      if (previousMode == null) {
        System.clearProperty("vision.photon.strategyMode");
      } else {
        System.setProperty("vision.photon.strategyMode", previousMode);
      }
    }
  }

  private static void writeCsv(String name, List<String> lines) throws IOException {
    Path dir = Path.of("build", "vision-stability");
    Files.createDirectories(dir);
    Files.write(dir.resolve(name), lines);
  }
}
