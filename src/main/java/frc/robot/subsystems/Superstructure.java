// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** Coordinates intake, hopper, shooter and aiming behavior into unified robot-level states. */
public class Superstructure extends SubsystemBase {
  public enum SuperState {
    DRIVE_STARTING_CONFIG,
    DRIVE,
    INTAKE,
    OUTTAKE,
    SHOOT,
    SHOOT_WITH_AIM,
    SHOOT_NO_AIM,
    MANUAL_SHOOT,
    PURGE
  }

  public enum ShootMode {
    DEFAULT_SHOOT_WITH_AIM,
    MANUAL_BUMPER_UP,
    MANUAL_TRENCH
  }

  private final Drive drive;
  private final Intake intake;
  private final Hopper hopper;
  private final Shooter shooter;
  private final Vision vision;

  private Optional<Rotation2d> currentHeading = Optional.empty();
  private double rotationLastTriggered = 0.0;

  private static final double ALIGNMENT_TOLERANCE_DEGREES = 5.0;
  private boolean alignedToTarget = false;
  private boolean allianceConfirmed = false;
  private FieldZones currentZone;

  private static final double MANUAL_BUMPER_UP_RPM = 2500.0;
  private static final double MANUAL_BUMPER_UP_HOOD = 0.0;
  private static final double MANUAL_TRENCH_RPM = 2900.0;
  private static final double MANUAL_TRENCH_HOOD = 0.75;
  private static final double NEUTRAL_ZONE_HOOD_LOCK = 2.0;

  private static final String[] ZONE_NAMES;

  static {
    FieldZones[] zones = FieldZones.values();
    ZONE_NAMES = new String[zones.length];
    for (int i = 0; i < zones.length; i++) {
      ZONE_NAMES[i] = zones[i].name();
    }
  }

  private SuperState state;
  private ShootMode shootMode = ShootMode.DEFAULT_SHOOT_WITH_AIM;
  private DriverStation.Alliance alliance;

  public Superstructure(Drive drive, Intake intake, Hopper hopper, Shooter shooter, Vision vision) {
    this.drive = drive;
    this.intake = intake;
    this.hopper = hopper;
    this.shooter = shooter;
    this.vision = vision;

    state = SuperState.DRIVE_STARTING_CONFIG;
    alliance = DriverStation.Alliance.Blue;
    currentZone = null;
    refreshAlliance();

    intake.setDefaultCommand(
        intake
            .runOnce(() -> intake.setDesiredState(IntakeState.DEPLOYED))
            .withName("Intake.Default(DEPLOYED)"));
    hopper.setDefaultCommand(
        hopper
            .runOnce(() -> hopper.setDesiredState(HopperState.STOP))
            .withName("Hopper.Default(STOP)"));
    shooter.setDefaultCommand(
        shooter
            .runOnce(() -> shooter.setDesiredState(ShooterState.PREPFUEL))
            .withName("Shooter.Default(PREPFUEL)"));
  }

  private void setAlliance(DriverStation.Alliance newAlliance) {
    alliance = newAlliance;
  }

  static Translation2d resolveAimTargetForZone(
      boolean allianceConfirmed, FieldZones zone, DriverStation.Alliance alliance) {
    if (!allianceConfirmed || zone == null) {
      return alliance == DriverStation.Alliance.Red
          ? FieldConstants.Hub.RED_CENTER_POSE
          : FieldConstants.Hub.BLUE_CENTER_POSE;
    }

    boolean isRed = alliance == DriverStation.Alliance.Red;
    return switch (zone) {
      case NEUTRAL_LEFT_SHOOT ->
          isRed
              ? FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT
              : FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT;
      case NEUTRAL_RIGHT_SHOOT ->
          isRed
              ? FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT
              : FieldConstants.AimPoints.BLUE_RIGHT_SHOOT_POINT;
      case NEUTRAL_LEFT_PURGE ->
          isRed
              ? FieldConstants.AimPoints.RED_LEFT_PURGE_POINT
              : FieldConstants.AimPoints.BLUE_LEFT_PURGE_POINT;
      case NEUTRAL_RIGHT_PURGE ->
          isRed
              ? FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT
              : FieldConstants.AimPoints.BLUE_RIGHT_PURGE_POINT;
      default -> isRed ? FieldConstants.Hub.RED_CENTER_POSE : FieldConstants.Hub.BLUE_CENTER_POSE;
    };
  }

  private Translation2d getCurrentAimTarget() {
    return resolveAimTargetForZone(allianceConfirmed, currentZone, alliance);
  }

  public Translation2d getCachedHubTarget() {
    return getCurrentAimTarget();
  }

  private boolean isShootAllowedZone() {
    if (!allianceConfirmed || currentZone == null) {
      return false;
    }

    return switch (currentZone) {
      case ALLIANCE_LEFT,
              ALLIANCE_RIGHT,
              NEUTRAL_LEFT_SHOOT,
              NEUTRAL_RIGHT_SHOOT,
              NEUTRAL_LEFT_PURGE,
              NEUTRAL_RIGHT_PURGE ->
          true;
      default -> false;
    };
  }

  private boolean isPurgeZone() {
    if (!allianceConfirmed || currentZone == null) {
      return false;
    }

    return switch (currentZone) {
      case NEUTRAL_LEFT_PURGE, NEUTRAL_RIGHT_PURGE -> true;
      default -> false;
    };
  }

  private boolean isNeutralShootOrPurgeZone() {
    if (!allianceConfirmed || currentZone == null) {
      return false;
    }

    return switch (currentZone) {
      case NEUTRAL_LEFT_SHOOT, NEUTRAL_RIGHT_SHOOT, NEUTRAL_LEFT_PURGE, NEUTRAL_RIGHT_PURGE -> true;
      default -> false;
    };
  }

  private Command createShootStateCommand(boolean withAim) {
    return Commands.run(
            () -> {
              setDesiredSuperState(withAim ? SuperState.SHOOT_WITH_AIM : SuperState.SHOOT_NO_AIM);
              shooter.setDesiredState(ShooterState.SHOOT);
              if (shooter.getCurrentState() == ShooterState.SHOOT && isAlignedToTarget()) {
                hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
              } else {
                hopper.setDesiredState(HopperState.STOP);
              }
            },
            shooter,
            hopper)
        .withName(withAim ? "SuperState(SHOOT_WITH_AIM)" : "SuperState(SHOOT_NO_AIM)");
  }

  private Command createPurgeStateCommand() {
    return Commands.run(
            () -> {
              setDesiredSuperState(SuperState.PURGE);
              intake.setDesiredState(IntakeState.OUTTAKE);
              shooter.setDesiredState(ShooterState.SHOOT);
              if (shooter.getCurrentState() == ShooterState.SHOOT && isAlignedToTarget()) {
                hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
              } else {
                hopper.setDesiredState(HopperState.STOP);
              }
            },
            intake,
            shooter,
            hopper)
        .withName("SuperState(PURGE)");
  }

  private Command createManualShootStateCommand(double shooterRpm, double hoodAngle) {
    return Commands.run(
            () -> {
              shooter.setSetpoint(shooterRpm, hoodAngle);
              setDesiredSuperState(SuperState.MANUAL_SHOOT);
              shooter.setDesiredState(ShooterState.SHOOT);
              if (shooter.getCurrentState() == ShooterState.SHOOT) {
                hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
              } else {
                hopper.setDesiredState(HopperState.STOP);
              }
            },
            shooter,
            hopper)
        .withName("SuperState(MANUAL_SHOOT)");
  }

  public Command selectedShootModeCmd() {
    return switch (shootMode) {
      case DEFAULT_SHOOT_WITH_AIM -> {
        if (!isShootAllowedZone()) {
          yield Commands.idle().withName("SuperState(SHOOT_WITH_AIM:DISALLOWED)");
        }
        if (isPurgeZone()) {
          yield createPurgeStateCommand().withName("SuperState(SHOOT_WITH_AIM->PURGE)");
        }
        yield createShootStateCommand(true);
      }
      case MANUAL_BUMPER_UP ->
          createManualShootStateCommand(MANUAL_BUMPER_UP_RPM, MANUAL_BUMPER_UP_HOOD)
              .withName("SuperState(SHOOT->MANUAL_BUMPER_UP)");
      case MANUAL_TRENCH ->
          createManualShootStateCommand(MANUAL_TRENCH_RPM, MANUAL_TRENCH_HOOD)
              .withName("SuperState(SHOOT->MANUAL_TRENCH)");
    };
  }

  public boolean shouldUsePurgeDuringShoot() {
    return shootMode == ShootMode.DEFAULT_SHOOT_WITH_AIM && isPurgeZone();
  }

  public Command purgeShootCmd() {
    if (!isPurgeZone()) {
      return Commands.idle().withName("SuperState(SHOOT->PURGE:DISALLOWED)");
    }
    return createPurgeStateCommand().withName("SuperState(SHOOT->PURGE)");
  }

  private void refreshAlliance() {
    DriverStation.getAlliance()
        .ifPresent(
            dsAlliance -> {
              boolean allianceChanged = dsAlliance != alliance;
              setAlliance(dsAlliance);
              allianceConfirmed = true;
              if (allianceChanged) {
                Logger.recordOutput("Superstructure/AllianceConfirmed", dsAlliance.name());
              }
            });
  }

  public Optional<Rotation2d> getCurrentHeading() {
    return currentHeading;
  }

  public void setCurrentHeading(Optional<Rotation2d> heading) {
    this.currentHeading = heading;
  }

  public double getRotationLastTriggered() {
    return rotationLastTriggered;
  }

  public void setRotationLastTriggered(double t) {
    this.rotationLastTriggered = t;
  }

  public void setDesiredSuperState(SuperState state) {
    this.state = state;
  }

  public Command setStateCmd(SuperState desiredState) {
    switch (desiredState) {
      case DRIVE_STARTING_CONFIG:
        return Commands.run(
                () -> {
                  setDesiredSuperState(SuperState.DRIVE_STARTING_CONFIG);
                  intake.setDesiredState(IntakeState.HOME);
                  hopper.setDesiredState(HopperState.STOP);
                  shooter.setDesiredState(ShooterState.PREPFUEL);
                },
                intake,
                hopper,
                shooter)
            .withName("SuperState(DRIVE_STARTING_CONFIG)");
      case DRIVE:
        return Commands.run(
                () -> {
                  setDesiredSuperState(SuperState.DRIVE);
                  intake.setDesiredState(IntakeState.DEPLOYED);
                  hopper.setDesiredState(HopperState.STOP);
                  shooter.setDesiredState(ShooterState.PREPFUEL);
                },
                intake,
                hopper,
                shooter)
            .withName("SuperState(DRIVE)");
      case INTAKE:
        return Commands.run(
                () -> {
                  setDesiredSuperState(SuperState.INTAKE);
                  intake.setDesiredState(IntakeState.INTAKE);
                  hopper.setDesiredState(HopperState.STOP);
                  shooter.setDesiredState(ShooterState.PREPFUEL);
                },
                intake,
                hopper,
                shooter)
            .withName("SuperState(INTAKE)");
      case OUTTAKE:
        return Commands.run(
                () -> {
                  setDesiredSuperState(SuperState.OUTTAKE);
                  intake.setDesiredState(IntakeState.OUTTAKE);
                  hopper.setDesiredState(HopperState.INDEXTOINTAKE);
                  shooter.setDesiredState(ShooterState.PREPFUEL);
                },
                intake,
                hopper,
                shooter)
            .withName("SuperState(OUTTAKE)");
      case SHOOT:
        return selectedShootModeCmd();
      case SHOOT_WITH_AIM:
        if (!isShootAllowedZone()) {
          return Commands.idle().withName("SuperState(SHOOT_WITH_AIM:DISALLOWED)");
        }
        if (isPurgeZone()) {
          return createPurgeStateCommand().withName("SuperState(SHOOT_WITH_AIM->PURGE)");
        }
        return createShootStateCommand(true);
      case SHOOT_NO_AIM:
        if (!isShootAllowedZone()) {
          return Commands.idle().withName("SuperState(SHOOT_NO_AIM:DISALLOWED)");
        }
        return createShootStateCommand(false);
      case MANUAL_SHOOT:
        return createManualShootStateCommand(MANUAL_BUMPER_UP_RPM, MANUAL_BUMPER_UP_HOOD);
      case PURGE:
        if (!isPurgeZone()) {
          return Commands.idle().withName("SuperState(PURGE:DISALLOWED)");
        }
        return createPurgeStateCommand();
      default:
        return Commands.none();
    }
  }

  public Command intakeOverrideCmd(IntakeState intakeState) {
    return Commands.run(() -> intake.setDesiredState(intakeState), intake)
        .withName("IntakeOverride(" + intakeState.name() + ")");
  }

  public Command hopperOverrideCmd(HopperState hopperState) {
    return Commands.runOnce(() -> hopper.setDesiredState(hopperState), hopper)
        .withName("HopperOverride(" + hopperState.name() + ")");
  }

  public Command shooterOverrideCmd(ShooterState shooterState) {
    return Commands.runOnce(() -> shooter.setDesiredState(shooterState), shooter)
        .withName("ShooterOverride(" + shooterState.name() + ")");
  }

  /** Snap odometry to the latest accepted vision pose, if one is available. */
  public Command forceReseedFromVisionCmd() {
    return new InstantCommand(
        () -> {
          if (vision == null) {
            return;
          }
          vision.forceReseedFromVision();
        },
        drive);
  }

  public ShootMode getShootMode() {
    return shootMode;
  }

  public void setShootMode(ShootMode shootMode) {
    this.shootMode = shootMode;
    Logger.recordOutput("Superstructure/ShootMode", shootMode.name());
  }

  public Command setShootModeCmd(ShootMode shootMode) {
    return Commands.runOnce(() -> setShootMode(shootMode));
  }

  public Command toggleShootModeCmd(ShootMode manualMode) {
    return Commands.runOnce(
        () -> {
          ShootMode nextMode =
              shootMode == manualMode ? ShootMode.DEFAULT_SHOOT_WITH_AIM : manualMode;
          setShootMode(nextMode);
        });
  }

  public Command toggleManualShooterDistanceOverrideCmd() {
    return Commands.runOnce(
            () -> {
              boolean override = !shooterOverrideEnabled();
              shooter.setManualDistanceOverride(override);
              Logger.recordOutput("Shooter/ManualDistanceOverride", override);
            },
            shooter)
        .withName("ShooterOverride(Toggle Manual Distance Override)");
  }

  private boolean shooterOverrideEnabled() {
    // Manual shoot modes own their setpoint; distance interpolation should be bypassed.
    return shootMode == ShootMode.MANUAL_BUMPER_UP || shootMode == ShootMode.MANUAL_TRENCH;
  }

  public boolean isAlignedToTarget() {
    return alignedToTarget;
  }

  static double resolveGeometricTargetHeadingRadians(Pose2d currentPose, Translation2d target) {
    double dx = target.getX() - currentPose.getX();
    double dy = target.getY() - currentPose.getY();
    return Math.atan2(dy, dx);
  }

  static double resolveOperatorPerspectiveTargetHeadingRadians(
      Pose2d currentPose, Translation2d hubTarget, DriverStation.Alliance alliance) {
    double targetAngleRad = resolveGeometricTargetHeadingRadians(currentPose, hubTarget);
    if (alliance == DriverStation.Alliance.Red) {
      targetAngleRad += Math.PI;
      targetAngleRad = Math.IEEEremainder(targetAngleRad, 2.0 * Math.PI);
    }
    return targetAngleRad;
  }

  static boolean isHeadingAlignedToTarget(
      Pose2d currentPose, Translation2d target, double toleranceDegrees) {
    double targetAngleRad = resolveGeometricTargetHeadingRadians(currentPose, target);

    double headingErrorRad = currentPose.getRotation().getRadians() - targetAngleRad;
    headingErrorRad = Math.IEEEremainder(headingErrorRad, 2.0 * Math.PI);

    return Math.abs(Math.toDegrees(headingErrorRad)) < toleranceDegrees;
  }

  private void updateAlignmentStatus(Pose2d currentPose, Translation2d target) {
    alignedToTarget = isHeadingAlignedToTarget(currentPose, target, ALIGNMENT_TOLERANCE_DEGREES);
  }

  public Optional<Rotation2d> getZoneLockedHeading() {
    if (!allianceConfirmed || currentZone == null) {
      return Optional.empty();
    }

    double leftLockDegrees = alliance == DriverStation.Alliance.Red ? 135.0 : -45.0;
    double rightLockDegrees = alliance == DriverStation.Alliance.Red ? -135.0 : 45.0;

    return switch (currentZone) {
      case ALLIANCE_LEFT, NEUTRAL_LEFT_SHOOT, NEUTRAL_LEFT_PURGE, NEUTRAL_LEFT, OPPONENT_LEFT ->
          Optional.of(Rotation2d.fromDegrees(leftLockDegrees));
      case ALLIANCE_RIGHT,
              NEUTRAL_RIGHT_SHOOT,
              NEUTRAL_RIGHT_PURGE,
              NEUTRAL_RIGHT,
              OPPONENT_RIGHT ->
          Optional.of(Rotation2d.fromDegrees(rightLockDegrees));
      default -> Optional.empty();
    };
  }

  public boolean isHubActive() {
    return HubShiftUtil.getShiftedShiftInfo().active();
  }

  public double getShiftTimeRemaining() {
    return HubShiftUtil.getShiftedShiftInfo().remainingTime();
  }

  @Override
  public void periodic() {
    refreshAlliance();

    Pose2d currentPose = drive.getPose();

    if (allianceConfirmed) {
      currentZone = FieldZones.fromPose(currentPose, alliance);
      Logger.recordOutput("Superstructure/Zone", ZONE_NAMES[currentZone.ordinal()]);
    }

    Translation2d aimTarget = getCurrentAimTarget();
    if (aimTarget != null) {
      double distanceToTarget = currentPose.getTranslation().getDistance(aimTarget);
      Logger.recordOutput(
          "Superstructure/Distance to Target (feet)", Units.metersToFeet(distanceToTarget));

      if (state != SuperState.MANUAL_SHOOT && !shooterOverrideEnabled()) {
        shooter.setSetpointForDistance(distanceToTarget);
        if (isNeutralShootOrPurgeZone()) {
          shooter.setSetpoint(shooter.getTargetRPM(), NEUTRAL_ZONE_HOOD_LOCK);
        }
      }
      updateAlignmentStatus(currentPose, aimTarget);
    }

    if (vision != null) {
      vision.setAiming(state == SuperState.SHOOT_WITH_AIM || state == SuperState.SHOOT_NO_AIM);
    }

    ShiftInfo officialShift = HubShiftUtil.getOfficialShiftInfo();
    ShiftInfo shiftedShift = HubShiftUtil.getShiftedShiftInfo();

    Logger.recordOutput("HubShift/Official/CurrentShift", officialShift.currentShift().name());
    Logger.recordOutput("HubShift/Official/Active", officialShift.active());
    Logger.recordOutput("HubShift/Official/ElapsedTime", officialShift.elapsedTime());
    Logger.recordOutput("HubShift/Official/RemainingTime", officialShift.remainingTime());

    Logger.recordOutput("HubShift/Shifted/CurrentShift", shiftedShift.currentShift().name());
    Logger.recordOutput("HubShift/Shifted/Active", shiftedShift.active());
    Logger.recordOutput("HubShift/Shifted/ElapsedTime", shiftedShift.elapsedTime());
    Logger.recordOutput("HubShift/Shifted/RemainingTime", shiftedShift.remainingTime());

    Logger.recordOutput(
        "HubShift/FirstActiveAlliance", HubShiftUtil.getFirstActiveAlliance().name());

    Logger.recordOutput("Superstructure/CurrentState", state.toString());
    Logger.recordOutput("Superstructure/IsAlignedToTarget", alignedToTarget);
    Logger.recordOutput(
        "Superstructure/ShootMode/DefaultShootWithAim",
        shootMode == ShootMode.DEFAULT_SHOOT_WITH_AIM);
    Logger.recordOutput(
        "Superstructure/ShootMode/ManualBumperUp", shootMode == ShootMode.MANUAL_BUMPER_UP);
    Logger.recordOutput(
        "Superstructure/ShootMode/ManualTrench", shootMode == ShootMode.MANUAL_TRENCH);
  }
}
