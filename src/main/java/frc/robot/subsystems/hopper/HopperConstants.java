// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Hopper hardware constants and TalonFX configurations. */
public final class HopperConstants {
  private HopperConstants() {}

  // ─── CAN IDs ──────────────────────────────────────────────────────────────
  public static final int HOPPER_ROLLER_LEAD_MOTOR_ID = 17;
  public static final int HOPPER_ROLLER_FOLLOW_MOTOR_ID = 18;

  // ─── Setpoints ────────────────────────────────────────────────────────────
  public static final double HOPPER_ROLLER_DUTY_CYCLE = 1.0;

  // ─── TalonFX Configurations ───────────────────────────────────────────────
  private static final double HOPPER_ROLLER_STATOR_CURRENT_LIMIT = 40.0;
  private static final double HOPPER_ROLLER_SUPPLY_CURRENT_LIMIT = 20.0;
  private static final double HOPPER_ROLLER_MAX_VOLTAGE = 12.0;
  private static final double HOPPER_ROLLER_RAMP_RATE = 0.2;

  public static final TalonFXConfiguration HOPPER_ROLLER_LEAD_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(HOPPER_ROLLER_STATOR_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(HOPPER_ROLLER_SUPPLY_CURRENT_LIMIT))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(HOPPER_ROLLER_MAX_VOLTAGE)
                  .withPeakReverseVoltage(-HOPPER_ROLLER_MAX_VOLTAGE))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(HOPPER_ROLLER_RAMP_RATE)
                  .withTorqueOpenLoopRampPeriod(HOPPER_ROLLER_RAMP_RATE)
                  .withVoltageOpenLoopRampPeriod(HOPPER_ROLLER_RAMP_RATE))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive));

  public static final TalonFXConfiguration HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(HOPPER_ROLLER_STATOR_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(HOPPER_ROLLER_SUPPLY_CURRENT_LIMIT))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(HOPPER_ROLLER_MAX_VOLTAGE)
                  .withPeakReverseVoltage(-HOPPER_ROLLER_MAX_VOLTAGE))
          .withOpenLoopRamps(
              new OpenLoopRampsConfigs()
                  .withDutyCycleOpenLoopRampPeriod(HOPPER_ROLLER_RAMP_RATE)
                  .withTorqueOpenLoopRampPeriod(HOPPER_ROLLER_RAMP_RATE)
                  .withVoltageOpenLoopRampPeriod(HOPPER_ROLLER_RAMP_RATE))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive));
}
