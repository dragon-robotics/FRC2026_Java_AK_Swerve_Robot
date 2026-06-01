// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.constants.FieldConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/** Owns the maple-sim Rebuilt arena, field fuel, robot intake, projectiles, and opponent robots. */
public class MapleSimGameSimulation {
  private enum OpponentMode {
    DISABLED,
    AUTO_CYCLE,
    JOYSTICK_DEFENSE
  }

  private static final String FUEL_TYPE = "Fuel";
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_BUMPER_LENGTH_METERS = 0.9;
  private static final double ROBOT_BUMPER_WIDTH_METERS = 0.9;
  private static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.2;
  private static final Pose2d USER_ROBOT_STARTING_POSE =
      new Pose2d(2.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.kZero);

  private static final Translation2d SHOOTER_OFFSET_FROM_CENTER = new Translation2d(0.32, 0.0);
  private static final double SHOOTER_HEIGHT_METERS = 0.45;
  private static final double BASE_SHOT_PITCH_DEGREES = 55.0;
  private static final double SHOT_COOLDOWN_SECS = 0.18;

  private static final Pose2d[] OPPONENT_QUEENING_POSES =
      new Pose2d[] {
        new Pose2d(-6.0, 0.0, Rotation2d.kZero),
        new Pose2d(-5.0, 0.0, Rotation2d.kZero),
        new Pose2d(-4.0, 0.0, Rotation2d.kZero)
      };

  private static final Pose2d[] OPPONENT_STARTING_POSES =
      new Pose2d[] {
        new Pose2d(
            FieldConstants.FIELD_LENGTH - 1.8, FieldConstants.FIELD_WIDTH * 0.72, Rotation2d.kPi),
        new Pose2d(
            FieldConstants.FIELD_LENGTH - 1.8, FieldConstants.FIELD_WIDTH * 0.28, Rotation2d.kPi)
      };

  private static final Translation2d[][] OPPONENT_AUTO_WAYPOINTS =
      new Translation2d[][] {
        new Translation2d[] {
          new Translation2d(FieldConstants.FIELD_LENGTH - 2.0, FieldConstants.FIELD_WIDTH * 0.72),
          new Translation2d(FieldConstants.FIELD_LENGTH * 0.56, FieldConstants.FIELD_WIDTH * 0.72),
          new Translation2d(FieldConstants.FIELD_LENGTH * 0.56, FieldConstants.FIELD_WIDTH * 0.55),
          new Translation2d(FieldConstants.FIELD_LENGTH - 2.0, FieldConstants.FIELD_WIDTH * 0.55)
        },
        new Translation2d[] {
          new Translation2d(FieldConstants.FIELD_LENGTH - 2.0, FieldConstants.FIELD_WIDTH * 0.28),
          new Translation2d(FieldConstants.FIELD_LENGTH * 0.56, FieldConstants.FIELD_WIDTH * 0.28),
          new Translation2d(FieldConstants.FIELD_LENGTH * 0.56, FieldConstants.FIELD_WIDTH * 0.45),
          new Translation2d(FieldConstants.FIELD_LENGTH - 2.0, FieldConstants.FIELD_WIDTH * 0.45)
        }
      };

  private final Arena2026Rebuilt arena;
  private final SelfControlledSwerveDriveSimulation userDriveSimulation;
  private final IntakeSimulation intakeSimulation;
  private final OpponentRobot[] opponents;
  private final SendableChooser<OpponentMode> opponentModeChooser = new SendableChooser<>();
  private final XboxController opponentController = new XboxController(2);

  private OpponentMode lastOpponentMode = null;
  private boolean hopperFeedingShooter = false;
  private double lastUserShotTimestamp = -999.0;

  public MapleSimGameSimulation(SelfControlledSwerveDriveSimulation userDriveSimulation) {
    this.userDriveSimulation = userDriveSimulation;

    // Pass false so the ramp area is NOT treated as a solid wall collider.
    // maple-sim cannot simulate ramp physics (15° up/down), so AddRampCollider=true
    // blocks the robot entirely; false allows traversal while keeping the hub blocked.
    arena = new Arena2026Rebuilt(false);
    arena.setEfficiencyMode(true);
    SimulatedArena.overrideInstance(arena);
    arena.addDriveTrainSimulation(userDriveSimulation.getDriveTrainSimulation());

    userDriveSimulation.setSimulationWorldPose(USER_ROBOT_STARTING_POSE);
    userDriveSimulation.resetOdometry(USER_ROBOT_STARTING_POSE);

    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            FUEL_TYPE,
            userDriveSimulation.getDriveTrainSimulation(),
            Meters.of(0.4),
            Meters.of(0.22),
            IntakeSide.FRONT,
            20);

    opponents =
        new OpponentRobot[] {
          new OpponentRobot(0, OPPONENT_QUEENING_POSES[0]),
          new OpponentRobot(1, OPPONENT_QUEENING_POSES[1])
        };

    configureDashboardControls();
    resetField();
  }

  public static SelfControlledSwerveDriveSimulation createUserDriveSimulation() {
    return createDriveSimulation(USER_ROBOT_STARTING_POSE, ROBOT_MASS_KG);
  }

  private static SelfControlledSwerveDriveSimulation createDriveSimulation(
      Pose2d startingPose, double robotMassKg) {
    SwerveModuleSimulationConfig moduleConfig =
        new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60Foc(1),
            DCMotor.getKrakenX60Foc(1),
            TunerConstants.FrontLeft.DriveMotorGearRatio,
            TunerConstants.FrontLeft.SteerMotorGearRatio,
            Volts.of(0.2),
            Volts.of(0.2),
            Meters.of(TunerConstants.FrontLeft.WheelRadius),
            KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
            WHEEL_COEFFICIENT_OF_FRICTION);

    DriveTrainSimulationConfig driveTrainConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(robotMassKg))
            .withBumperSize(
                Meters.of(ROBOT_BUMPER_LENGTH_METERS), Meters.of(ROBOT_BUMPER_WIDTH_METERS))
            .withCustomModuleTranslations(Drive.getModuleTranslations())
            .withSwerveModule(moduleConfig)
            .withGyro(COTS.ofPigeon2());

    return new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(driveTrainConfig, startingPose))
        .withSteerPID(new PIDController(8.0, 0.0, 0.0))
        .withCurrentLimits(Amps.of(60.0), Amps.of(20.0));
  }

  private void configureDashboardControls() {
    opponentModeChooser.setDefaultOption("Disabled", OpponentMode.DISABLED);
    opponentModeChooser.addOption("Auto Cycle", OpponentMode.AUTO_CYCLE);
    opponentModeChooser.addOption("Joystick Defense", OpponentMode.JOYSTICK_DEFENSE);
    SmartDashboard.putData("FieldSimulation/Opponent Mode", opponentModeChooser);
    SmartDashboard.putData(
        "FieldSimulation/Reset Field", Commands.runOnce(this::resetField).ignoringDisable(true));
  }

  public void resetField() {
    arena.resetFieldForAuto();
    intakeSimulation.setGamePiecesCount(0);
    hopperFeedingShooter = false;
    lastUserShotTimestamp = -999.0;
    userDriveSimulation.setSimulationWorldPose(USER_ROBOT_STARTING_POSE);
    userDriveSimulation.resetOdometry(USER_ROBOT_STARTING_POSE);
    setOpponentMode(OpponentMode.DISABLED);
    Logger.recordOutput("FieldSimulation/RobotSuccessfulShotTrajectory", new Pose3d[] {});
    Logger.recordOutput("FieldSimulation/RobotMissedShotTrajectory", new Pose3d[] {});
    Logger.recordOutput("FieldSimulation/OpponentShotTrajectory", new Pose3d[] {});
  }

  public void setIntakeRunning(boolean running) {
    if (running) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }

  public void setHopperFeedingShooter(boolean feedingShooter) {
    hopperFeedingShooter = feedingShooter;
  }

  public boolean isHopperFeedingShooter() {
    return hopperFeedingShooter;
  }

  public boolean tryLaunchUserFuel(double shooterRpm, double hoodPositionRotations) {
    double now = Timer.getFPGATimestamp();
    if (!hopperFeedingShooter || now - lastUserShotTimestamp < SHOT_COOLDOWN_SECS) {
      return false;
    }
    if (!intakeSimulation.obtainGamePieceFromIntake()) {
      return false;
    }

    Pose2d robotPose = userDriveSimulation.getActualPoseInSimulationWorld();
    double launchSpeedMetersPerSecond = MathUtil.clamp(shooterRpm / 6000.0 * 20.0, 4.0, 20.0);
    double shotPitchDegrees =
        MathUtil.clamp(BASE_SHOT_PITCH_DEGREES + hoodPositionRotations * 10.0, 35.0, 70.0);

    arena.addGamePieceProjectile(
        new RebuiltFuelOnFly(
                robotPose.getTranslation(),
                SHOOTER_OFFSET_FROM_CENTER,
                userDriveSimulation.getActualSpeedsFieldRelative(),
                robotPose.getRotation(),
                Meters.of(SHOOTER_HEIGHT_METERS),
                MetersPerSecond.of(launchSpeedMetersPerSecond),
                Degrees.of(shotPitchDegrees))
            .withProjectileTrajectoryDisplayCallBack(
                poses ->
                    Logger.recordOutput(
                        "FieldSimulation/RobotSuccessfulShotTrajectory",
                        poses.toArray(Pose3d[]::new)),
                poses ->
                    Logger.recordOutput(
                        "FieldSimulation/RobotMissedShotTrajectory",
                        poses.toArray(Pose3d[]::new))));

    lastUserShotTimestamp = now;
    return true;
  }

  public void simulationInit() {
    resetField();
  }

  public void simulationPeriodic() {
    updateOpponentRobots();
    arena.simulationPeriodic();

    for (OpponentRobot opponent : opponents) {
      opponent.driveSimulation.periodic();
    }

    Logger.recordOutput("FieldSimulation/Fuel", arena.getGamePiecesArrayByType(FUEL_TYPE));
    Logger.recordOutput(
        "FieldSimulation/UserRobotActualPose",
        new Pose3d(userDriveSimulation.getActualPoseInSimulationWorld()));
    Logger.recordOutput("FieldSimulation/OpponentRobots", getOpponentPoseArray());
    Logger.recordOutput("FieldSimulation/HeldFuel", intakeSimulation.getGamePiecesAmount());
    Logger.recordOutput("FieldSimulation/HopperFeedingShooter", hopperFeedingShooter);
    Logger.recordOutput("FieldSimulation/BlueScore", arena.getScore(DriverStation.Alliance.Blue));
    Logger.recordOutput("FieldSimulation/RedScore", arena.getScore(DriverStation.Alliance.Red));
  }

  private Pose3d[] getOpponentPoseArray() {
    Pose3d[] poses = new Pose3d[opponents.length];
    for (int i = 0; i < opponents.length; i++) {
      poses[i] = new Pose3d(opponents[i].driveSimulation.getActualPoseInSimulationWorld());
    }
    return poses;
  }

  private void updateOpponentRobots() {
    OpponentMode selectedMode = opponentModeChooser.getSelected();
    if (selectedMode == null) {
      selectedMode = OpponentMode.DISABLED;
    }

    if (selectedMode != lastOpponentMode) {
      setOpponentMode(selectedMode);
    }

    switch (selectedMode) {
      case DISABLED:
        stopOpponents();
        break;
      case AUTO_CYCLE:
        updateOpponentAutoCycle();
        break;
      case JOYSTICK_DEFENSE:
        updateJoystickOpponent();
        break;
      default:
        break;
    }
  }

  private void setOpponentMode(OpponentMode mode) {
    lastOpponentMode = mode;
    for (int i = 0; i < opponents.length; i++) {
      switch (mode) {
        case AUTO_CYCLE:
          opponents[i].reset(OPPONENT_STARTING_POSES[i]);
          break;
        case JOYSTICK_DEFENSE:
          opponents[i].reset(i == 0 ? OPPONENT_STARTING_POSES[0] : opponents[i].queeningPose);
          break;
        case DISABLED:
        default:
          opponents[i].reset(opponents[i].queeningPose);
          break;
      }
    }
  }

  private void stopOpponents() {
    for (OpponentRobot opponent : opponents) {
      opponent.driveSimulation.runChassisSpeeds(
          new ChassisSpeeds(), Translation2d.kZero, false, false);
    }
  }

  private void updateOpponentAutoCycle() {
    for (OpponentRobot opponent : opponents) {
      Translation2d[] waypoints = OPPONENT_AUTO_WAYPOINTS[opponent.id];
      Translation2d target = waypoints[opponent.waypointIndex];
      Pose2d pose = opponent.driveSimulation.getActualPoseInSimulationWorld();
      Translation2d error = target.minus(pose.getTranslation());

      if (error.getNorm() < 0.35) {
        opponent.waypointIndex = (opponent.waypointIndex + 1) % waypoints.length;
        target = waypoints[opponent.waypointIndex];
        error = target.minus(pose.getTranslation());
      }

      double desiredHeadingRadians = Math.atan2(error.getY(), error.getX());
      double headingErrorRadians =
          Math.IEEEremainder(
              desiredHeadingRadians - pose.getRotation().getRadians(), 2.0 * Math.PI);
      ChassisSpeeds fieldRelativeSpeeds =
          new ChassisSpeeds(
              MathUtil.clamp(error.getX() * 1.6, -2.5, 2.5),
              MathUtil.clamp(error.getY() * 1.6, -2.5, 2.5),
              MathUtil.clamp(headingErrorRadians * 4.0, -4.0, 4.0));

      opponent.driveSimulation.runChassisSpeeds(
          ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, pose.getRotation()),
          Translation2d.kZero,
          false,
          true);

      if (Timer.getFPGATimestamp() - opponent.lastFeedShotTimestamp > 6.0
          && pose.getX() < FieldConstants.FIELD_LENGTH * 0.65) {
        launchOpponentFeedShot(opponent);
      }
    }
  }

  private void updateJoystickOpponent() {
    OpponentRobot opponent = opponents[0];
    Pose2d pose = opponent.driveSimulation.getActualPoseInSimulationWorld();
    double maxSpeed = opponent.driveSimulation.maxLinearVelocity().in(MetersPerSecond);
    double maxAngularSpeed =
        opponent
            .driveSimulation
            .maxAngularVelocity()
            .in(edu.wpi.first.units.Units.RadiansPerSecond);

    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(
            -opponentController.getLeftY() * maxSpeed,
            -opponentController.getLeftX() * maxSpeed,
            -opponentController.getRightX() * maxAngularSpeed);
    opponent.driveSimulation.runChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, pose.getRotation()),
        Translation2d.kZero,
        false,
        true);

    if (opponentController.getRightTriggerAxis() > 0.5
        && Timer.getFPGATimestamp() - opponent.lastFeedShotTimestamp > 0.5) {
      launchOpponentFeedShot(opponent);
    }

    for (int i = 1; i < opponents.length; i++) {
      opponents[i].driveSimulation.runChassisSpeeds(
          new ChassisSpeeds(), Translation2d.kZero, false, false);
    }
  }

  private void launchOpponentFeedShot(OpponentRobot opponent) {
    Pose2d pose = opponent.driveSimulation.getActualPoseInSimulationWorld();
    arena.addGamePieceProjectile(
        new RebuiltFuelOnFly(
                pose.getTranslation(),
                SHOOTER_OFFSET_FROM_CENTER,
                opponent.driveSimulation.getActualSpeedsFieldRelative(),
                pose.getRotation().plus(Rotation2d.kPi),
                Meters.of(0.5),
                MetersPerSecond.of(10.0),
                Degrees.of(45.0))
            .withProjectileTrajectoryDisplayCallBack(
                poses ->
                    Logger.recordOutput(
                        "FieldSimulation/OpponentShotTrajectory", poses.toArray(Pose3d[]::new))));
    opponent.lastFeedShotTimestamp = Timer.getFPGATimestamp();
  }

  private final class OpponentRobot {
    private final int id;
    private final Pose2d queeningPose;
    private final SelfControlledSwerveDriveSimulation driveSimulation;

    private int waypointIndex = 0;
    private double lastFeedShotTimestamp = -999.0;

    private OpponentRobot(int id, Pose2d queeningPose) {
      this.id = id;
      this.queeningPose = queeningPose;
      this.driveSimulation = createDriveSimulation(queeningPose, 55.0);
      arena.addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
    }

    private void reset(Pose2d pose) {
      waypointIndex = 0;
      driveSimulation.setSimulationWorldPose(pose);
      driveSimulation.resetOdometry(pose);
      driveSimulation.runChassisSpeeds(new ChassisSpeeds(), Translation2d.kZero, false, false);
    }
  }
}
