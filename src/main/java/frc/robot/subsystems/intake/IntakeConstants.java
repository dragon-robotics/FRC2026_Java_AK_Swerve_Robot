// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;

/**
 * Intake hardware constants and TalonFX configurations. Ported from the upstream feature branch;
 * SparkMax variants are omitted since this project standardizes on TalonFX.
 */
public final class IntakeConstants {
  private IntakeConstants() {}

  // ─── CAN IDs ──────────────────────────────────────────────────────────────
  public static final int INTAKE_ROLLER_MOTOR_ID = 12;
  public static final int INTAKE_ARM_MOTOR_ID = 11;
  public static final int INTAKE_ARM_CANCODER_ID = 0;

  // ─── PID Slots ────────────────────────────────────────────────────────────
  public static final int INTAKE_ARM_FAST_PID_SLOT = 0;
  public static final int INTAKE_ARM_SLOW_PID_SLOT = 1;

  // ─── Physical ─────────────────────────────────────────────────────────────
  public static final double INTAKE_ARM_LENGTH_METERS = Units.inchesToMeters(13.370);
  public static final double INTAKE_ARM_MASS_KG = Units.lbsToKilograms(10);
  public static final double INTAKE_ARM_GEAR_RATIO = 36.0;
  public static final double INTAKE_MIN_ANGLE_RADIANS = Units.degreesToRadians(0);
  public static final double INTAKE_MAX_ANGLE_RADIANS = Units.degreesToRadians(90);
  public static final double INTAKE_STARTING_ANGLE_RADIANS = INTAKE_MIN_ANGLE_RADIANS;

  // ─── Roller setpoints ─────────────────────────────────────────────────────
  public static final double INTAKE_ROLLER_DUTY_CYCLE = 1.0;
  public static final double OUTTAKE_ROLLER_DUTY_CYCLE = -1.0;

  // ─── Arm positions (rotations) ────────────────────────────────────────────
  public static final double INTAKE_ARM_DEPLOYED_POSITION = 0.0;
  public static final double INTAKE_ARM_STOWED_POSITION = 0.37;
  public static final double INTAKE_ARM_WOKTOSS_POSITION = 0.15;

  /**
   * Intermediate juicer position: arm moves quickly here to clear the hopper wall before slowly
   * moving to {@link #INTAKE_ARM_JUICER_FINAL_POSITION}.
   */
  public static final double INTAKE_ARM_JUICER_PRE_POSITION = 0.15;

  public static final double INTAKE_ARM_JUICER_FINAL_POSITION = 0.28;
  public static final double INTAKE_ARM_POSITION_TOLERANCE = 0.025;

  // ─── TalonFX Configurations ───────────────────────────────────────────────
  public static final TalonFXConfiguration INTAKE_ROLLER_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(58))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLowerLimit(Amps.of(20))
                  .withSupplyCurrentLowerTime(Seconds.of(0.2)))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(12))
                  .withPeakReverseVoltage(Volts.of(-12)))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.15))
                  .withTorqueOpenLoopRampPeriod(Seconds.of(0.15))
                  .withVoltageOpenLoopRampPeriod(Seconds.of(0.15)))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive));

  public static final TalonFXConfiguration INTAKE_ARM_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(50))
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(30))
                  .withSupplyCurrentLowerLimit(Amps.of(20))
                  .withSupplyCurrentLowerTime(Seconds.of(0.2)))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(Volts.of(10))
                  .withPeakReverseVoltage(Volts.of(-10)))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.15))
                  .withTorqueOpenLoopRampPeriod(Seconds.of(0.15))
                  .withVoltageOpenLoopRampPeriod(Seconds.of(0.15)))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          // Fast profile for deploying the intake to overcome the constant force
          // spring of the extending hopper.
          .withSlot0(
              new Slot0Configs()
                  .withKP(20)
                  .withKI(0)
                  .withKD(0)
                  .withKS(0)
                  .withKV(2.4)
                  .withKA(0)
                  .withKG(0.45)
                  .withGravityType(GravityTypeValue.Arm_Cosine)
                  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
          // Slow profile for stowing / wok-toss / juicer motions.
          .withSlot1(
              new Slot1Configs()
                  .withKP(7)
                  .withKI(0)
                  .withKD(0)
                  .withKS(0)
                  .withKV(2.4)
                  .withKA(0)
                  .withKG(0.45)
                  .withGravityType(GravityTypeValue.Arm_Cosine)
                  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(0)
                  .withMotionMagicAcceleration(200)
                  .withMotionMagicJerk(200)
                  .withMotionMagicExpo_kV(2.0)
                  .withMotionMagicExpo_kA(2.0))
          .withFeedback(
              new FeedbackConfigs()
                  .withFusedCANcoder(new CoreCANcoder(INTAKE_ARM_CANCODER_ID))
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withSensorToMechanismRatio(1)
                  .withRotorToSensorRatio(INTAKE_ARM_GEAR_RATIO)
                  .withFeedbackRotorOffset(0));

  public static final CANcoderConfiguration INTAKE_ARM_CANCODER_CONFIG =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withAbsoluteSensorDiscontinuityPoint(0.5)
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                  .withMagnetOffset(-0.843506));
}
