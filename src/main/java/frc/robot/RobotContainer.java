// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.APTAG_CAMERA_NAMES;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.constants.OperatorConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final String OTF_RECOVERY_ENABLED_KEY = "Auto/Recovery/Enabled";

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Hopper hopper;
  private final Shooter shooter;
  private final Superstructure superstructure;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[0], VisionConstants.APTAG_POSE_EST_CAM_F_POS),
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[1], VisionConstants.APTAG_POSE_EST_CAM_R_POS),
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[2], VisionConstants.APTAG_POSE_EST_CAM_B_POS),
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[3], VisionConstants.APTAG_POSE_EST_CAM_L_POS));
        intake = new Intake(new IntakeIOTalonFX());
        hopper = new Hopper(new HopperIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVisionSim(
                    APTAG_CAMERA_NAMES[0],
                    VisionConstants.APTAG_POSE_EST_CAM_F_POS,
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    APTAG_CAMERA_NAMES[1],
                    VisionConstants.APTAG_POSE_EST_CAM_R_POS,
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    APTAG_CAMERA_NAMES[2],
                    VisionConstants.APTAG_POSE_EST_CAM_B_POS,
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    APTAG_CAMERA_NAMES[3],
                    VisionConstants.APTAG_POSE_EST_CAM_L_POS,
                    drive::getPose));
        intake = new Intake(new IntakeIOSim());
        hopper = new Hopper(new HopperIOSim());
        shooter = new Shooter(new ShooterIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement, drive::getPose, new VisionIO() {}, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        hopper = new Hopper(new HopperIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    // Create superstructure coordinator (after all subsystems are initialized)
    superstructure = new Superstructure(drive, intake, hopper, shooter);

    // Register PathPlanner named commands (must be done BEFORE AutoBuilder.buildAutoChooser)
    NamedCommands.registerCommand("Intake", superstructure.setStateCmd(SuperState.INTAKE));
    NamedCommands.registerCommand(
        "Shoot",
        superstructure
            .setStateCmd(SuperState.SHOOT)
            .alongWith(
                Commands.waitSeconds(1.5)
                    .andThen(superstructure.intakeOverrideCmd(IntakeState.JUICER))));
    NamedCommands.registerCommand("Drive", superstructure.setStateCmd(SuperState.DRIVE));
    SmartDashboard.putBoolean(OTF_RECOVERY_ENABLED_KEY, true);
    NamedCommands.registerCommand(
        "OTF Recovery Follow Path",
        DriveCommands.followPathWithOnTheFlyRecovery(
            drive,
            "OTF Recovery Test",
            new PathConstraints(
                3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(1080)),
            0.75,
            0.5,
            () -> SmartDashboard.getBoolean(OTF_RECOVERY_ENABLED_KEY, true),
            1));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ── Default command: field-relative drive with heading lock ────────
    // D-Pad Up = half speed (50%), X button = zone angle lock
    drive.setDefaultCommand(
        DriveCommands.joystickDriveWithHeadingLock(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getHID().getPOV() == 0,
            () ->
                driverController.getHID().getXButton()
                    ? superstructure.getZoneLockedHeading()
                    : java.util.Optional.empty()));

    // ────────────────────────────────────────────────────────────────────
    // Driver Controls
    // ────────────────────────────────────────────────────────────────────

    // Reset field-centric heading on Start + Back
    driverController
        .start()
        .and(driverController.back())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Intake: LT held → INTAKE, released → DRIVE
    driverController
        .leftTrigger(0.2)
        .onTrue(superstructure.setStateCmd(SuperState.INTAKE))
        .onFalse(superstructure.setStateCmd(SuperState.DRIVE));

    // Outtake: RB held → OUTTAKE, released → DRIVE
    driverController
        .rightBumper()
        .onTrue(superstructure.setStateCmd(SuperState.OUTTAKE))
        .onFalse(superstructure.setStateCmd(SuperState.DRIVE));

    // Shoot: RT held → SHOOT state + aim at hub, released → DRIVE
    driverController
        .rightTrigger(0.2)
        .whileTrue(
            superstructure
                .setStateCmd(SuperState.SHOOT)
                .alongWith(
                    DriveCommands.joystickDriveAimAtTarget(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        superstructure::getCachedHubTarget)))
        .onFalse(superstructure.setStateCmd(SuperState.DRIVE));

    // Juicer: B held → JUICER override, released → DEPLOYED
    driverController
        .b()
        .whileTrue(superstructure.intakeOverrideCmd(IntakeState.JUICER))
        .onFalse(superstructure.intakeOverrideCmd(IntakeState.DEPLOYED));

    // Drive starting config: A held
    driverController.a().whileTrue(superstructure.setStateCmd(SuperState.DRIVE_STARTING_CONFIG));

    // ────────────────────────────────────────────────────────────────────
    // Operator Controls
    // ────────────────────────────────────────────────────────────────────

    // Juicer: operator B held → JUICER override, released → DEPLOYED
    operatorController
        .b()
        .whileTrue(superstructure.intakeOverrideCmd(IntakeState.JUICER))
        .onFalse(superstructure.intakeOverrideCmd(IntakeState.DEPLOYED));

    // Toggle manual shooter distance override: operator X + A
    operatorController
        .x()
        .and(operatorController.a())
        .onTrue(superstructure.toggleManualShooterDistanceOverrideCmd());

    // Kicker full power override: operator RB held
    operatorController
        .rightBumper()
        .whileTrue(
            Commands.run(() -> shooter.setKickerPercentDirect(1.0), shooter)
                .withName("Kicker Full Power"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
