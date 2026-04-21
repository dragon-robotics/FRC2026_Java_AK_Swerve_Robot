package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class OnTheFlyAutoRecoveryCommandTest {
  @BeforeEach
  void setup() {
    HAL.initialize(500, 0);
  }

  @Test
  void triggersRecoveryWhenRobotIsBumpedOffPath() {
    AtomicReference<Pose2d> robotPose =
        new AtomicReference<>(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    List<Pose2d> expectedPathPoses =
        List.of(
            new Pose2d(0.0, 0.0, Rotation2d.kZero),
            new Pose2d(1.0, 0.0, Rotation2d.kZero),
            new Pose2d(2.0, 0.0, Rotation2d.kZero));

    AtomicInteger followCreatedCount = new AtomicInteger();
    AtomicInteger recoveryCreatedCount = new AtomicInteger();

    AtomicInteger followInterruptedCount = new AtomicInteger();

    Supplier<Command> followSupplier =
        () -> {
          followCreatedCount.incrementAndGet();
          return new FakeCommand(false, followInterruptedCount, null);
        };

    Supplier<Command> recoverySupplier =
        () -> {
          recoveryCreatedCount.incrementAndGet();
          return new FakeCommand(true, null, null);
        };

    OnTheFlyAutoRecoveryCommand commandUnderTest =
        new OnTheFlyAutoRecoveryCommand(
            new SubsystemBase() {},
            robotPose::get,
            () -> expectedPathPoses,
            followSupplier,
            recoverySupplier,
            () -> true,
            0.5,
            0.0,
            1,
            false);

    commandUnderTest.initialize();
    commandUnderTest.execute();

    // Simulate a bump that knocks the robot far from the expected path.
    robotPose.set(new Pose2d(1.0, 2.0, Rotation2d.kZero));
    commandUnderTest.execute();

    assertEquals(1, followCreatedCount.get());
    assertEquals(1, recoveryCreatedCount.get());
    assertEquals(1, followInterruptedCount.get());

    while (!commandUnderTest.isFinished()) {
      commandUnderTest.execute();
    }
  }

  @Test
  void distanceToExpectedPathIsNearestPoseDistance() {
    Pose2d actualPose = new Pose2d(3.0, 4.0, Rotation2d.kZero);
    List<Pose2d> expectedPathPoses =
        List.of(
            new Pose2d(0.0, 0.0, Rotation2d.kZero),
            new Pose2d(2.0, 4.0, Rotation2d.kZero),
            new Pose2d(10.0, 10.0, Rotation2d.kZero));

    double distance =
        OnTheFlyAutoRecoveryCommand.distanceToExpectedPathMeters(actualPose, expectedPathPoses);

    assertEquals(1.0, distance, 1e-9);
  }

  @Test
  void supportsMultipleRecoveryAttemptsInSingleRunWhenConfigured() {
    AtomicReference<Pose2d> robotPose =
        new AtomicReference<>(new Pose2d(0.0, 0.0, Rotation2d.kZero));
    List<Pose2d> expectedPathPoses =
        List.of(
            new Pose2d(0.0, 0.0, Rotation2d.kZero),
            new Pose2d(1.0, 0.0, Rotation2d.kZero),
            new Pose2d(2.0, 0.0, Rotation2d.kZero));

    AtomicInteger followCreatedCount = new AtomicInteger();
    AtomicInteger recoveryCreatedCount = new AtomicInteger();

    Supplier<Command> followSupplier =
        () -> {
          int index = followCreatedCount.incrementAndGet();
          // First two follow commands run indefinitely to allow a bump-triggered recovery.
          if (index <= 2) {
            return new FakeCommand(false, null, null);
          }
          // Final follow command completes, allowing the top-level command to finish.
          return new FakeCommand(true, null, null);
        };

    Supplier<Command> recoverySupplier =
        () -> {
          recoveryCreatedCount.incrementAndGet();
          return new FakeCommand(true, null, null);
        };

    BooleanSupplier recoveryEnabledSupplier = () -> true;

    OnTheFlyAutoRecoveryCommand commandUnderTest =
        new OnTheFlyAutoRecoveryCommand(
            new SubsystemBase() {},
            robotPose::get,
            () -> expectedPathPoses,
            followSupplier,
            recoverySupplier,
            recoveryEnabledSupplier,
            0.5,
            0.0,
            2,
            true);

    commandUnderTest.initialize();

    // Trigger first recovery.
    robotPose.set(new Pose2d(1.0, 2.0, Rotation2d.kZero));
    commandUnderTest.execute();
    commandUnderTest.execute();
    commandUnderTest.execute();
    commandUnderTest.execute();

    // Trigger second recovery.
    robotPose.set(new Pose2d(1.0, -2.0, Rotation2d.kZero));
    commandUnderTest.execute();
    commandUnderTest.execute();
    commandUnderTest.execute();
    commandUnderTest.execute();

    // Return pose to path so the final follow command completes naturally.
    robotPose.set(new Pose2d(2.0, 0.0, Rotation2d.kZero));
    while (!commandUnderTest.isFinished()) {
      commandUnderTest.execute();
    }

    assertEquals(2, recoveryCreatedCount.get());
    assertEquals(3, followCreatedCount.get());
  }

  private static class FakeCommand extends Command {
    private final boolean finishesAfterThreeExecutes;
    private final AtomicInteger interruptedCounter;
    private final AtomicInteger executeCounter;

    private int executeCount = 0;

    FakeCommand(
        boolean finishesAfterThreeExecutes,
        AtomicInteger interruptedCounter,
        AtomicInteger executeCounter) {
      this.finishesAfterThreeExecutes = finishesAfterThreeExecutes;
      this.interruptedCounter = interruptedCounter;
      this.executeCounter = executeCounter;
    }

    @Override
    public void execute() {
      executeCount++;
      if (executeCounter != null) {
        executeCounter.incrementAndGet();
      }
    }

    @Override
    public void end(boolean interrupted) {
      if (interrupted && interruptedCounter != null) {
        interruptedCounter.incrementAndGet();
      }
    }

    @Override
    public boolean isFinished() {
      return finishesAfterThreeExecutes && executeCount >= 3;
    }
  }
}
