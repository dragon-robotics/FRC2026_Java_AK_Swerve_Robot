package frc.robot.util.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class HopperConstants {
  public static final int HOPPER_ROLLER_LEAD_MOTOR_ID = 17;
  public static final int HOPPER_ROLLER_FOLLOW_MOTOR_ID = 18;
  public static final double HOPPER_ROLLER_STATOR_CURRENT_LIMIT = 40.0;
  public static final double HOPPER_ROLLER_SUPPLY_CURRENT_LIMIT = 20.0;
  public static final double HOPPER_ROLLER_MAX_VOLTAGE = 12.0;
  public static final double HOPPER_ROLLER_RAMP_RATE = 0.2;
  public static final double HOPPER_ROLLER_DUTY_CYCLE = 1.0;
  public static final double HOPPER_ROLLER_RPM = 1000.0;
  public static final double HOPPER_ROLLER_REVERSE_RPM = -1000.0;

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
