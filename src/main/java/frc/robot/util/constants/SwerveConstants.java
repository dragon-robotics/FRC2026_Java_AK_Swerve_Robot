package frc.robot.util.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
  public static final double STEER_KP = 100;
  public static final double STEER_KI = 0;
  public static final double STEER_KD = 0.5;
  public static final double STEER_KS = 0.1;
  public static final double STEER_KV = 1.59;
  public static final double STEER_KA = 0;

  public static final double DRIVE_KP = 0.1;
  public static final double DRIVE_KI = 0;
  public static final double DRIVE_KD = 0;
  public static final double DRIVE_KS = 0;
  public static final double DRIVE_KV = 0.124;
  public static final double DRIVE_KA = 0;

  public static final double HEADING_KP = 8;
  public static final double HEADING_KI = 0;
  public static final double HEADING_KD = 0.3;
  public static final double HEADING_TOLERANCE = 0.01;

  public static final double ANGLE_GEAR_RATIO = 12.8;
  public static final double DRIVE_GEAR_RATIO = 6.12;
  public static final double PULSE_PER_ROTATION = 1;
  public static final double WHEEL_DIAMETER_INCHES = 4.0;
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
  public static final double MAX_SPEED_FEET_PER_SECOND = 18.2;
  public static final double MAX_SPEED_METERS_PER_SECOND =
      Units.feetToMeters(MAX_SPEED_FEET_PER_SECOND);

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
  public static final double CHASSIS_MASS = ROBOT_MASS;
  public static final Translation3d CHASSIS_CG = new Translation3d(0, 0, Units.inchesToMeters(8));
  public static final double LOOP_TIME = 0.13;

  public static final double WHEEL_LOCK_TIME = 10;

  public static final double SWERVE_DEADBAND = 0.1;

  public static final double ODOMETRY_X_STD_METERS = Units.inchesToMeters(2);
  public static final double ODOMETRY_Y_STD_METERS = Units.inchesToMeters(2);
  public static final double ODOMETRY_HEADING_STD_RADIANS = Units.degreesToRadians(1);

  public static final Matrix<N3, N1> ODOMETRY_STD =
      VecBuilder.fill(ODOMETRY_X_STD_METERS, ODOMETRY_Y_STD_METERS, ODOMETRY_HEADING_STD_RADIANS);
}
