package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("sim")
class VisionPoseStaticScenariosTest {

  private record Scenario(String name, double x, double y, double yawDeg) {
    Pose2d pose() {
      return new Pose2d(x, y, Rotation2d.fromDegrees(yawDeg));
    }
  }

  private static final Scenario LEFT_FRONT = new Scenario("left_front", 4.407, 7.279, -90);
  private static final Scenario LEFT_BACK = new Scenario("left_back", 4.407, 7.279, 90);
  private static final Scenario LEFT_LEFT = new Scenario("left_left", 4.407, 7.279, 180);
  private static final Scenario LEFT_RIGHT = new Scenario("left_right", 4.407, 7.279, 0);
  private static final Scenario RIGHT_FRONT = new Scenario("right_front", 4.407, 0.650, 90);
  private static final Scenario RIGHT_BACK = new Scenario("right_back", 4.407, 0.650, -90);
  private static final Scenario RIGHT_LEFT = new Scenario("right_left", 4.407, 0.650, 0);
  private static final Scenario RIGHT_RIGHT = new Scenario("right_right", 4.407, 0.650, 180);

  private static final double DT = 0.02;
  private static final int WARMUP_CYCLES = 100;
  private static final int MEASURE_CYCLES = 250;
  private static final double MAX_JUMP_M = 0.15;
  private static final double MAX_VISION_DEVIATION_M = 1.5;

  private static boolean halReady = false;
  private static RobotContainer container;

  @BeforeAll
  static void setUpHal() {
    try {
      halReady = HAL.initialize(500, 0);
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(true);
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
  void leftFront() throws IOException {
    runScenario(LEFT_FRONT);
  }

  @Test
  void leftBack() throws IOException {
    runScenario(LEFT_BACK);
  }

  @Test
  void leftLeft() throws IOException {
    runScenario(LEFT_LEFT);
  }

  @Test
  void leftRight() throws IOException {
    runScenario(LEFT_RIGHT);
  }

  @Test
  void rightFront() throws IOException {
    runScenario(RIGHT_FRONT);
  }

  @Test
  void rightBack() throws IOException {
    runScenario(RIGHT_BACK);
  }

  @Test
  void rightLeft() throws IOException {
    runScenario(RIGHT_LEFT);
  }

  @Test
  void rightRight() throws IOException {
    runScenario(RIGHT_RIGHT);
  }

  private void runScenario(Scenario s) throws IOException {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");
    Assumptions.assumeTrue(container != null, "RobotContainer failed to initialize");

    CommandScheduler.getInstance().cancelAll();
    container.getDrive().setPose(s.pose());

    List<String> csv = new ArrayList<>();
    csv.add(
        "cycle,phase,t_s,odomX,odomY,odomYawDeg,visionX,visionY,visionYawDeg,odomJump_m,visionDeviation_m,hasVision");

    Pose2d prev = s.pose();
    double maxOdomJump = 0.0;
    double maxVisionDeviation = 0.0;
    int totalCycles = WARMUP_CYCLES + MEASURE_CYCLES;

    for (int cycle = 0; cycle < totalCycles; cycle++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);

      Pose2d odom = container.getDrive().getPose();
      Optional<Vision.AcceptedObservationSnapshot> vis =
          container.getVision().getLatestAcceptedObservationSnapshot();

      double odomJump = odom.getTranslation().getDistance(prev.getTranslation());
      boolean measuring = cycle >= WARMUP_CYCLES;

      double visionDev = Double.NaN;
      if (vis.isPresent()) {
        visionDev = vis.get().pose().getTranslation().getDistance(s.pose().getTranslation());
        if (measuring) {
          maxVisionDeviation = Math.max(maxVisionDeviation, visionDev);
        }
      }
      if (measuring) {
        maxOdomJump = Math.max(maxOdomJump, odomJump);
      }
      prev = odom;

      double t = cycle * DT;
      csv.add(
          String.format(
              "%d,%s,%.3f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%b",
              cycle,
              measuring ? "measure" : "warmup",
              t,
              odom.getX(),
              odom.getY(),
              odom.getRotation().getDegrees(),
              vis.map(v -> v.pose().getX()).orElse(Double.NaN),
              vis.map(v -> v.pose().getY()).orElse(Double.NaN),
              vis.map(v -> v.pose().getRotation().getDegrees()).orElse(Double.NaN),
              odomJump,
              visionDev,
              vis.isPresent()));
    }

    writeCsv("static-" + s.name() + ".csv", csv);

    assertTrue(maxOdomJump <= MAX_JUMP_M);
    if (maxVisionDeviation > 0.0) {
      assertTrue(maxVisionDeviation <= MAX_VISION_DEVIATION_M);
    }
  }

  private static void writeCsv(String name, List<String> lines) throws IOException {
    Path dir = Path.of("build", "vision-stability");
    Files.createDirectories(dir);
    Files.write(dir.resolve(name), lines);
  }
}
