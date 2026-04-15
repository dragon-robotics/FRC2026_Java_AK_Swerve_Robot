package frc.robot.util.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {

  public static final int SHOOTER_HOOD_MOTOR_ID = 13;
  public static final int SHOOTER_KICKER_MOTOR_ID = 14;
  public static final int SHOOTER_LEAD_MOTOR_ID = 15;
  public static final int SHOOTER_FOLLOW_MOTOR_ID = 16;

  public static final int SHOOTER_CANCODER_ID = 2;

  public static final double SHOOTER_KICKER_DUTY_CYCLE = 1.0;
  public static final double SHOOTER_KICKER_VOLTAGE = 12.0;
  public static final double SHOOTER_KICKER_RPM = 3000.0;
  public static final double SHOOTER_LEAD_DUTY_CYCLE = 1.0;
  public static final double SHOOTER_LEAD_VOLTAGE = 12.0;
  public static final double SHOOTER_LEAD_RPM = 2500.0;
  public static final double SHOOTER_HOOD_SETTING = 0.0;

  public record ShooterSetpoint(double shooterRPM, double hoodAngle) {}

  /** Interpolating table for shooter RPM based on distance in meters */
  public static final InterpolatingDoubleTreeMap SHOOTER_RPM_MAP = new InterpolatingDoubleTreeMap();

  /** Interpolating table for hood angle (rotations) based on distance in meters */
  public static final InterpolatingDoubleTreeMap SHOOTER_HOOD_MAP =
      new InterpolatingDoubleTreeMap();

  static {
    // Distance (ft) -> RPM
    SHOOTER_RPM_MAP.put(Units.feetToMeters(5), 2450.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(6), 2500.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(7), 2550.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(8), 2700.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(9), 2800.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(10), 2850.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(11), 2900.0);
    SHOOTER_RPM_MAP.put(Units.feetToMeters(12), 3000.0);

    // Distance (ft) -> Hood angle (rotations)
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(5), 0.00);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(6), 0.00);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(7), 0.00);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(8), 0.00);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(9), 0.00);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(10), 0.75);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(11), 0.75);
    SHOOTER_HOOD_MAP.put(Units.feetToMeters(12), 1.25);
  }

  /** Returns interpolated RPM and hood angle for a given distance in meters */
  public static ShooterSetpoint getSetpointForDistance(double distanceMeters) {
    return new ShooterSetpoint(
        SHOOTER_RPM_MAP.get(distanceMeters), SHOOTER_HOOD_MAP.get(distanceMeters));
  }

  public static final TalonFXConfiguration SHOOTER_LEAD_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLowerLimit(Amps.of(20))
                  .withSupplyCurrentLowerTime(Seconds.of(0.25)))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(
              new Slot0Configs()
                  .withKP(8)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(4.325)
                  .withKV(0.013)
                  .withKA(0.0));

  public static final SparkBaseConfig SHOOTER_LEAD_SPARKMAX_CONFIG =
      new SparkMaxConfig()
          .apply(
              new SparkMaxConfig()
                  .voltageCompensation(10)
                  .smartCurrentLimit(60, 20)
                  .secondaryCurrentLimit(80)
                  .openLoopRampRate(0.1)
                  .idleMode(IdleMode.kCoast))
          .apply(
              new ClosedLoopConfig()
                  .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                  .outputRange(-1, 1)
                  .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
                  .apply(
                      new MAXMotionConfig()
                          .cruiseVelocity(4000, ClosedLoopSlot.kSlot0)
                          .maxAcceleration(8000, ClosedLoopSlot.kSlot0)
                          .allowedProfileError(40, ClosedLoopSlot.kSlot0)));

  public static final TalonFXConfiguration SHOOTER_FOLLOW_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLowerLimit(Amps.of(20))
                  .withSupplyCurrentLowerTime(Seconds.of(0.25)))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(12))
                  .withPeakReverseVoltage(Volts.of(-12)))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

  public static final SparkBaseConfig SHOOTER_FOLLOW_SPARKMAX_CONFIG =
      new SparkMaxConfig()
          .apply(
              new SparkMaxConfig()
                  .voltageCompensation(10)
                  .smartCurrentLimit(60, 20)
                  .secondaryCurrentLimit(80)
                  .openLoopRampRate(0.1)
                  .idleMode(IdleMode.kCoast)
                  .follow(SHOOTER_LEAD_MOTOR_ID));

  public static final TalonFXConfiguration SHOOTER_KICKER_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(77))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLowerLimit(Amps.of(30))
                  .withSupplyCurrentLowerTime(Seconds.of(0.25)))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(11))
                  .withPeakReverseVoltage(Volts.of(-11)))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive));

  public static final SparkBaseConfig SHOOTER_KICKER_SPARKMAX_CONFIG =
      new SparkMaxConfig()
          .apply(
              new SparkMaxConfig()
                  .voltageCompensation(10)
                  .smartCurrentLimit(20, 10)
                  .secondaryCurrentLimit(40)
                  .openLoopRampRate(0.1)
                  .idleMode(IdleMode.kCoast))
          .apply(
              new ClosedLoopConfig()
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .outputRange(-1, 1)
                  .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
                  .apply(
                      new MAXMotionConfig()
                          .cruiseVelocity(4000, ClosedLoopSlot.kSlot0)
                          .maxAcceleration(8000, ClosedLoopSlot.kSlot0)
                          .allowedProfileError(40, ClosedLoopSlot.kSlot0)));

  public static final TalonFXConfiguration SHOOTER_HOOD_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(25))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(15)))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(10))
                  .withPeakReverseVoltage(Volts.of(-10)))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(
              new Slot0Configs()
                  .withKP(8.0)
                  .withKI(0.0)
                  .withKD(0.1)
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .withKG(0.4)
                  .withGravityType(GravityTypeValue.Elevator_Static)
                  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

  public static final CANcoderConfiguration SHOOTER_HOOD_CANCODER_CONFIG =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  // choose one; common is signed ±0.5 rotations
                  .withAbsoluteSensorDiscontinuityPoint(0.5)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  // set this to your calibrated zero (rotations)
                  .withMagnetOffset(0.0));

  public static final SparkBaseConfig SHOOTER_HOOD_SPARKMAX_CONFIG =
      new SparkMaxConfig()
          .apply(
              new SparkMaxConfig()
                  .voltageCompensation(10)
                  .smartCurrentLimit(15, 10)
                  .secondaryCurrentLimit(30)
                  .openLoopRampRate(0.1)
                  .idleMode(IdleMode.kBrake))
          .apply(new AbsoluteEncoderConfig().zeroOffset(0).inverted(false))
          .apply(
              new ClosedLoopConfig()
                  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                  .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
                  .outputRange(-1, 1)
                  .apply(
                      new FeedForwardConfig()
                          .kS(0.0, ClosedLoopSlot.kSlot0)
                          .kV(0.0, ClosedLoopSlot.kSlot0)
                          .kA(0.0, ClosedLoopSlot.kSlot0)
                          .kG(0.0, ClosedLoopSlot.kSlot0))
                  .apply(
                      new MAXMotionConfig()
                          .cruiseVelocity(4000, ClosedLoopSlot.kSlot0)
                          .maxAcceleration(8000, ClosedLoopSlot.kSlot0)
                          .allowedProfileError(5, ClosedLoopSlot.kSlot0)
                          .positionMode(
                              MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0)));
}
