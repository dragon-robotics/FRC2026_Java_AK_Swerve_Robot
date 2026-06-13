package frc.robot.util.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class HopperConstants {
  public static final int HOPPER_ROLLER_LEAD_MOTOR_ID = 17;
  public static final int HOPPER_ROLLER_FOLLOW_MOTOR_ID = 18;
  public static final Current HOPPER_ROLLER_STATOR_CURRENT_LIMIT = Amps.of(50.0);
  public static final Current HOPPER_ROLLER_SUPPLY_CURRENT_LIMIT = Amps.of(40.0);
  public static final Current HOPPER_ROLLER_SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(20.0);
  public static final Time HOPPER_ROLLER_SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.2);
  public static final Voltage HOPPER_ROLLER_MAX_VOLTAGE = Volts.of(12.0);
  public static final Time HOPPER_ROLLER_RAMP_RATE = Seconds.of(0.5);
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
                  .withSupplyCurrentLimit(HOPPER_ROLLER_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLowerLimit(HOPPER_ROLLER_SUPPLY_CURRENT_LOWER_LIMIT)
                  .withSupplyCurrentLowerTime(HOPPER_ROLLER_SUPPLY_CURRENT_LOWER_TIME))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(HOPPER_ROLLER_MAX_VOLTAGE)
                  .withPeakReverseVoltage(HOPPER_ROLLER_MAX_VOLTAGE.unaryMinus()))
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
                  .withSupplyCurrentLimit(HOPPER_ROLLER_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLowerLimit(HOPPER_ROLLER_SUPPLY_CURRENT_LOWER_LIMIT)
                  .withSupplyCurrentLowerTime(HOPPER_ROLLER_SUPPLY_CURRENT_LOWER_TIME))
          .withVoltage(
              new VoltageConfigs()
                  .withPeakForwardVoltage(HOPPER_ROLLER_MAX_VOLTAGE)
                  .withPeakReverseVoltage(HOPPER_ROLLER_MAX_VOLTAGE.unaryMinus()))
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
