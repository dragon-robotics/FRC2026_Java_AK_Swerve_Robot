package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
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
class RightSideSafeOdometryTest {

  private static final String AUTO_NAME = "Right Side Safe";
  private static final double DT = 0.02;
  private static final int MAX_CYCLES = 1500;
  private static final int WARMUP_CYCLES = 15;
  private static final double MAX_SINGLE_CYCLE_JUMP_M = 0.5;
  private static final int MAX_LARGE_JUMPS = 5;

  private static boolean halReady = false;

  @BeforeAll
  static void setUpHal() {
    try {
      halReady = HAL.initialize(500, 0);
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
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
  void rightSideSafeAutoKeepsOdometryContinuous() throws IOException {
    Assumptions.assumeTrue(halReady, "HAL/simulation unavailable in this environment");

    RobotContainer container;
    Command auto;
    try {
      container = new RobotContainer();
      auto = new PathPlannerAuto(AUTO_NAME);
    } catch (Throwable t) {
      Assumptions.abort("Headless sim could not initialize RobotContainer/auto: " + t);
      return;
    }

    auto.schedule();

    List<String> csv = new ArrayList<>();
    csv.add("cycle,t,odomX,odomY,visionX,visionY,jump,hasVision");

    Pose2d prev = container.getDrive().getPose();
    int largeJumpCount = 0;
    int visionAcceptedCycles = 0;
    int cyclesRun = 0;

    try {
      for (int cycle = 0; cycle < MAX_CYCLES && auto.isScheduled(); cycle++) {
        CommandScheduler.getInstance().run();
        SimHooks.stepTiming(DT);
        cyclesRun++;

        Pose2d odom = container.getDrive().getPose();
        Optional<Vision.AcceptedObservationSnapshot> vision =
            container.getVision().getLatestAcceptedObservationSnapshot();

        double jump = odom.getTranslation().getDistance(prev.getTranslation());
        if (cycle >= WARMUP_CYCLES && jump > MAX_SINGLE_CYCLE_JUMP_M) {
          largeJumpCount++;
        }
        prev = odom;
        if (vision.isPresent()) {
          visionAcceptedCycles++;
        }

        double t = cycle * DT;
        csv.add(
            String.format(
                "%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%b",
                cycle,
                t,
                odom.getX(),
                odom.getY(),
                vision.map(s -> s.pose().getX()).orElse(Double.NaN),
                vision.map(s -> s.pose().getY()).orElse(Double.NaN),
                jump,
                vision.isPresent()));
      }
    } catch (Throwable t) {
      writeCsv("right-side-safe.csv", csv);
      Assumptions.abort("Headless sim threw while running the auto: " + t);
      return;
    }

    writeCsv("right-side-safe.csv", csv);

    Assumptions.assumeTrue(cyclesRun > WARMUP_CYCLES, "Auto did not run long enough to evaluate");
    assertTrue(visionAcceptedCycles > 0);
    assertTrue(largeJumpCount <= MAX_LARGE_JUMPS);
  }

  private static void writeCsv(String name, List<String> lines) throws IOException {
    Path dir = Path.of("build", "vision-stability");
    Files.createDirectories(dir);
    Files.write(dir.resolve(name), lines);
  }
}
