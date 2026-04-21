// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Wraps a path-following command with one-shot on-the-fly recovery. If the robot drifts farther
 * than the configured threshold from the expected path poses, the active follow command is
 * interrupted and replaced with a pathfind-then-follow recovery command.
 */
public class OnTheFlyAutoRecoveryCommand extends Command {
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<List<Pose2d>> expectedPathPosesSupplier;
  private final Supplier<Command> initialFollowCommandSupplier;
  private final Supplier<Command> recoveryCommandSupplier;
  private final BooleanSupplier recoveryEnabledSupplier;
  private final double maxDeviationMeters;
  private final double recoveryCooldownSeconds;
  private final int maxRecoveryAttempts;
  private final boolean resumeFollowAfterRecovery;

  private final Timer timer = new Timer();
  private Command activeCommand;
  private boolean inRecovery = false;
  private int recoveryAttemptCount = 0;
  private double lastRecoveryTriggerTimestampSeconds = Double.NEGATIVE_INFINITY;

  public OnTheFlyAutoRecoveryCommand(
      Drive drive,
      PathPlannerPath path,
      PathConstraints recoveryConstraints,
      double maxDeviationMeters,
      double recoveryCooldownSeconds) {
    this(
        drive,
        path,
        recoveryConstraints,
        maxDeviationMeters,
        recoveryCooldownSeconds,
        () -> true,
        1);
  }

  public OnTheFlyAutoRecoveryCommand(
      Drive drive,
      PathPlannerPath path,
      PathConstraints recoveryConstraints,
      double maxDeviationMeters,
      double recoveryCooldownSeconds,
      BooleanSupplier recoveryEnabledSupplier,
      int maxRecoveryAttempts) {
    this(
        drive,
        drive::getPose,
        () -> {
          PathPlannerPath expectedPath = AutoBuilder.shouldFlip() ? path.flipPath() : path;
          return expectedPath.getPathPoses();
        },
        () -> AutoBuilder.followPath(path),
        () -> AutoBuilder.pathfindThenFollowPath(path, recoveryConstraints),
        recoveryEnabledSupplier,
        maxDeviationMeters,
        recoveryCooldownSeconds,
        maxRecoveryAttempts,
        false);
  }

  OnTheFlyAutoRecoveryCommand(
      Subsystem requirement,
      Supplier<Pose2d> poseSupplier,
      Supplier<List<Pose2d>> expectedPathPosesSupplier,
      Supplier<Command> initialFollowCommandSupplier,
      Supplier<Command> recoveryCommandSupplier,
      BooleanSupplier recoveryEnabledSupplier,
      double maxDeviationMeters,
      double recoveryCooldownSeconds,
      int maxRecoveryAttempts,
      boolean resumeFollowAfterRecovery) {
    this.poseSupplier = poseSupplier;
    this.expectedPathPosesSupplier = expectedPathPosesSupplier;
    this.initialFollowCommandSupplier = initialFollowCommandSupplier;
    this.recoveryCommandSupplier = recoveryCommandSupplier;
    this.recoveryEnabledSupplier = recoveryEnabledSupplier;
    this.maxDeviationMeters = maxDeviationMeters;
    this.recoveryCooldownSeconds = recoveryCooldownSeconds;
    this.maxRecoveryAttempts = maxRecoveryAttempts;
    this.resumeFollowAfterRecovery = resumeFollowAfterRecovery;
    addRequirements(requirement);
    setName("OnTheFlyAutoRecovery");
  }

  @Override
  public void initialize() {
    inRecovery = false;
    recoveryAttemptCount = 0;
    lastRecoveryTriggerTimestampSeconds = Double.NEGATIVE_INFINITY;
    timer.restart();
    activeCommand = initialFollowCommandSupplier.get();
    activeCommand.initialize();
    Logger.recordOutput("Auto/Recovery/Enabled", recoveryEnabledSupplier.getAsBoolean());
    Logger.recordOutput("Auto/Recovery/InRecovery", false);
    Logger.recordOutput("Auto/Recovery/AttemptCount", 0);
    Logger.recordOutput("Auto/Recovery/DistanceFromPathMeters", 0.0);
  }

  @Override
  public void execute() {
    if (activeCommand == null) {
      return;
    }

    Logger.recordOutput("Auto/Recovery/Enabled", recoveryEnabledSupplier.getAsBoolean());

    activeCommand.execute();

    if (activeCommand.isFinished()) {
      activeCommand.end(false);
      if (inRecovery && resumeFollowAfterRecovery) {
        inRecovery = false;
        Logger.recordOutput("Auto/Recovery/InRecovery", false);
        activeCommand = initialFollowCommandSupplier.get();
        activeCommand.initialize();
      } else {
        activeCommand = null;
      }
      return;
    }

    if (inRecovery || !recoveryEnabledSupplier.getAsBoolean()) {
      return;
    }

    boolean hasAttemptsRemaining =
        maxRecoveryAttempts <= 0 || recoveryAttemptCount < maxRecoveryAttempts;
    if (!hasAttemptsRemaining) {
      return;
    }

    double nowSeconds = timer.get();
    if (nowSeconds - lastRecoveryTriggerTimestampSeconds < recoveryCooldownSeconds) {
      return;
    }

    double distanceFromPath =
        distanceToExpectedPathMeters(poseSupplier.get(), expectedPathPosesSupplier.get());
    Logger.recordOutput("Auto/Recovery/DistanceFromPathMeters", distanceFromPath);
    if (distanceFromPath > maxDeviationMeters) {
      activeCommand.end(true);
      activeCommand = recoveryCommandSupplier.get();
      activeCommand.initialize();
      inRecovery = true;
      recoveryAttemptCount++;
      lastRecoveryTriggerTimestampSeconds = nowSeconds;
      Logger.recordOutput("Auto/Recovery/InRecovery", true);
      Logger.recordOutput("Auto/Recovery/AttemptCount", recoveryAttemptCount);
    }
  }

  @Override
  public boolean isFinished() {
    return activeCommand == null;
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.end(interrupted);
    }
  }

  static double distanceToExpectedPathMeters(Pose2d pose, List<Pose2d> expectedPathPoses) {
    if (expectedPathPoses == null || expectedPathPoses.isEmpty()) {
      return 0.0;
    }

    double minDistance = Double.POSITIVE_INFINITY;
    for (Pose2d expectedPose : expectedPathPoses) {
      double distance = pose.getTranslation().getDistance(expectedPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
      }
    }
    return minDistance;
  }
}
