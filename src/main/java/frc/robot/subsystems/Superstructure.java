// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// Coordinates Intake, Hopper, and Shooter state machines into coherent
// "super-states". Ported from dragon-robotics/FRC2026_Java_Swerve_Robot
// (feature/shift-timers), adapted to AdvantageKit IO pattern.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * The Superstructure coordinates the Intake, Hopper, and Shooter subsystems into unified
 * "super-states" (DRIVE, INTAKE, OUTTAKE, SHOOT). Each super-state sets the desired states of all
 * three subsystems simultaneously via WPILib command requirements.
 *
 * <p>When a command ends (e.g. button released), the scheduler automatically resumes the default
 * commands on each subsystem — no god loop needed.
 */
public class Superstructure extends SubsystemBase {

  // ──────────────────────────────────────────────────────────────────────────
  // State Enum
  // ──────────────────────────────────────────────────────────────────────────

  public enum SuperState {
    DRIVE_STARTING_CONFIG,
    DRIVE,
    INTAKE,
    OUTTAKE,
    SHOOT
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Subsystem References
  // ──────────────────────────────────────────────────────────────────────────

  private final Drive drive;
  private final Intake intake;
  private final Hopper hopper;
  private final Shooter shooter;

  // ──────────────────────────────────────────────────────────────────────────
  // Heading Tracking
  // ──────────────────────────────────────────────────────────────────────────

  private Optional<Rotation2d> currentHeading = Optional.empty();
  private double rotationLastTriggered = 0.0;

  // ──────────────────────────────────────────────────────────────────────────
  // Alignment & Targeting
  // ──────────────────────────────────────────────────────────────────────────

  private static final double ALIGNMENT_TOLERANCE_DEGREES = 3.0;
  private boolean alignedToTarget = false;
  private Translation2d cachedHubTarget;
  private boolean allianceConfirmed = false;
  private FieldZones currentZone;
  private boolean manualShooterDistanceOverride = false;

  // Pre-cached zone name strings — avoids .name() heap allocation every cycle
  private static final String[] ZONE_NAMES;

  static {
    FieldZones[] zones = FieldZones.values();
    ZONE_NAMES = new String[zones.length];
    for (int i = 0; i < zones.length; i++) {
      ZONE_NAMES[i] = zones[i].name();
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // State Machine
  // ──────────────────────────────────────────────────────────────────────────

  private SuperState state;

  // ──────────────────────────────────────────────────────────────────────────
  // Alliance
  // ──────────────────────────────────────────────────────────────────────────
  private DriverStation.Alliance alliance;

  // ──────────────────────────────────────────────────────────────────────────
  // Constructor
  // ──────────────────────────────────────────────────────────────────────────

  public Superstructure(Drive drive, Intake intake, Hopper hopper, Shooter shooter) {
    this.drive = drive;
    this.intake = intake;
    this.hopper = hopper;
    this.shooter = shooter;

    state = SuperState.DRIVE_STARTING_CONFIG;

    // Default to blue until DriverStation alliance becomes available
    alliance = DriverStation.Alliance.Blue;
    cachedHubTarget = FieldConstants.Hub.BLUE_CENTER_POSE;
    currentZone = null;
    refreshAllianceAndCachedHubTarget();

    // ── Default Commands ─────────────────────────────────────────────────
    // These run whenever no other command requires the subsystem.
    // They define the "DRIVE" superstate behavior.
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
    cachedHubTarget =
        alliance == DriverStation.Alliance.Red
            ? FieldConstants.Hub.RED_CENTER_POSE
            : FieldConstants.Hub.BLUE_CENTER_POSE;
  }

  private void refreshAllianceAndCachedHubTarget() {
    DriverStation.getAlliance()
        .ifPresent(
            dsAlliance -> {
              setAlliance(dsAlliance);
              allianceConfirmed = true;
              Logger.recordOutput("Robot/AllianceConfirmed", dsAlliance.name());
            });
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Heading Accessors (used by drive commands)
  // ──────────────────────────────────────────────────────────────────────────

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

  // ──────────────────────────────────────────────────────────────────────────
  // State Commands
  // ──────────────────────────────────────────────────────────────────────────

  public void setDesiredSuperState(SuperState state) {
    this.state = state;
  }

  /**
   * Returns a command that transitions relevant subsystems to the requested SuperState. Each
   * subsystem is controlled through proper WPILib command requirements — not direct calls from
   * periodic().
   *
   * <p>When the returned command ends (button released), the scheduler resumes the default commands
   * on each required subsystem automatically.
   */
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
        return Commands.run(
                () -> {
                  setDesiredSuperState(SuperState.SHOOT);
                  shooter.setDesiredState(ShooterState.SHOOT);
                  if (manualShooterDistanceOverride) {
                    // Override active: shoot in place
                    if (shooter.getCurrentState() == ShooterState.SHOOT) {
                      hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
                    } else {
                      hopper.setDesiredState(HopperState.STOP);
                    }
                  } else {
                    // Normal: require alignment before feeding
                    if (shooter.getCurrentState() == ShooterState.SHOOT && isAlignedToTarget()) {
                      hopper.setDesiredState(HopperState.INDEXTOSHOOTER);
                    } else {
                      hopper.setDesiredState(HopperState.STOP);
                    }
                  }
                },
                shooter,
                hopper)
            .withName("SuperState(SHOOT)");

      default:
        return Commands.none();
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Individual Subsystem Override Commands
  // ──────────────────────────────────────────────────────────────────────────

  /** Override intake independently — preempts default, doesn't touch hopper/shooter. */
  public Command intakeOverrideCmd(IntakeState intakeState) {
    return Commands.run(() -> intake.setDesiredState(intakeState), intake)
        .withName("IntakeOverride(" + intakeState.name() + ")");
  }

  /** Override hopper independently. */
  public Command hopperOverrideCmd(HopperState hopperState) {
    return Commands.runOnce(() -> hopper.setDesiredState(hopperState), hopper)
        .withName("HopperOverride(" + hopperState.name() + ")");
  }

  /** Override shooter independently. */
  public Command shooterOverrideCmd(ShooterState shooterState) {
    return Commands.runOnce(() -> shooter.setDesiredState(shooterState), shooter)
        .withName("ShooterOverride(" + shooterState.name() + ")");
  }

  public Command toggleManualShooterDistanceOverrideCmd() {
    return Commands.runOnce(
            () -> {
              manualShooterDistanceOverride = !manualShooterDistanceOverride;
              shooter.setManualDistanceOverride(manualShooterDistanceOverride);
              Logger.recordOutput("Shooter/ManualDistanceOverride", manualShooterDistanceOverride);
            },
            shooter)
        .withName("ShooterOverride(Toggle Manual Distance Override)");
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Alignment
  // ──────────────────────────────────────────────────────────────────────────

  public boolean isAlignedToTarget() {
    return alignedToTarget;
  }

  /** Zero-allocation alignment check using raw atan2 math. */
  private void updateAlignmentStatus(Pose2d currentPose, Translation2d hubTarget) {
    double dx = hubTarget.getX() - currentPose.getX();
    double dy = hubTarget.getY() - currentPose.getY();
    double targetAngleRad = Math.atan2(dy, dx);

    double headingErrorRad = currentPose.getRotation().getRadians() - targetAngleRad;
    headingErrorRad = Math.IEEEremainder(headingErrorRad, 2.0 * Math.PI);

    alignedToTarget = Math.abs(Math.toDegrees(headingErrorRad)) < ALIGNMENT_TOLERANCE_DEGREES;
  }

  /**
   * Returns the desired locked heading angle based on the current field zone and alliance. Returns
   * empty if no lock is defined for the zone.
   */
  public Optional<Rotation2d> getZoneLockedHeading() {
    if (!allianceConfirmed || currentZone == null) {
      return Optional.empty();
    }

    boolean isRed = alliance == DriverStation.Alliance.Red;

    switch (currentZone) {
      case ALLIANCE_LEFT, NEUTRAL_LEFT, OPPONENT_LEFT:
        return Optional.of(
            isRed
                ? Rotation2d.fromDegrees(-45).rotateBy(Rotation2d.kPi)
                : Rotation2d.fromDegrees(-45));
      case ALLIANCE_RIGHT, NEUTRAL_RIGHT, OPPONENT_RIGHT:
        return Optional.of(
            isRed
                ? Rotation2d.fromDegrees(45.0).rotateBy(Rotation2d.kPi)
                : Rotation2d.fromDegrees(45.0));
      default:
        return Optional.empty();
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Hub Shift Accessors
  // ──────────────────────────────────────────────────────────────────────────

  /**
   * Returns true if our hub is currently active and we should be shooting. Uses the shifted
   * (fudged) timing so fuel arrives within the active window.
   */
  public boolean isHubActive() {
    return HubShiftUtil.getShiftedShiftInfo().active();
  }

  /**
   * Returns the time remaining in the current shift (using shifted timing). Drivers can use this to
   * decide whether to commit to a scoring cycle.
   */
  public double getShiftTimeRemaining() {
    return HubShiftUtil.getShiftedShiftInfo().remainingTime();
  }

  /** Returns the cached hub target position (alliance-aware). */
  public Translation2d getCachedHubTarget() {
    return cachedHubTarget;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Periodic — telemetry and vision reseed ONLY, no subsystem state writes
  // ──────────────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // ── Alliance (poll until confirmed, then never again) ──────────────
    if (!allianceConfirmed) {
      refreshAllianceAndCachedHubTarget();
    }

    // Always compute telemetry regardless of alliance confirmation
    Pose2d currentPose = drive.getPose();

    // ── Zone detection ────────────────────────────────────────────────
    if (allianceConfirmed) {
      currentZone = FieldZones.fromPose(currentPose, alliance);
      Logger.recordOutput("Robot/Zone", ZONE_NAMES[currentZone.ordinal()]);
    }

    // ── Distance + alignment (needed by SHOOT command group) ──────────
    if (cachedHubTarget != null) {
      double distanceToHub = currentPose.getTranslation().getDistance(cachedHubTarget);
      Logger.recordOutput(
          "Superstructure/Distance to Hub (feet)", Units.metersToFeet(distanceToHub));
      shooter.setSetpointForDistance(distanceToHub);
      updateAlignmentStatus(currentPose, cachedHubTarget);
    }

    // ── Hub Shift Tracking ────────────────────────────────────────────
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
  }
}
