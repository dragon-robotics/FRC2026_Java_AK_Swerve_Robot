// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.HOPPER_ROLLER_FOLLOW_MOTOR_ID;
import static frc.robot.subsystems.hopper.HopperConstants.HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG;
import static frc.robot.subsystems.hopper.HopperConstants.HOPPER_ROLLER_LEAD_MOTOR_ID;
import static frc.robot.subsystems.hopper.HopperConstants.HOPPER_ROLLER_LEAD_TALONFX_CONFIG;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Real TalonFX-based implementation of {@link HopperIO}. */
public class HopperIOTalonFX implements HopperIO {
  private final TalonFX leadTalon;
  private final TalonFX followTalon;

  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(true);

  // Lead signals
  private final StatusSignal<Angle> leadPosition;
  private final StatusSignal<AngularVelocity> leadVelocity;
  private final StatusSignal<Voltage> leadAppliedVolts;
  private final StatusSignal<Current> leadStatorCurrent;
  private final StatusSignal<Temperature> leadTemp;

  // Follow signals
  private final StatusSignal<Angle> followPosition;
  private final StatusSignal<AngularVelocity> followVelocity;
  private final StatusSignal<Voltage> followAppliedVolts;
  private final StatusSignal<Current> followStatorCurrent;
  private final StatusSignal<Temperature> followTemp;

  private final Debouncer leadConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public HopperIOTalonFX() {
    leadTalon = new TalonFX(HOPPER_ROLLER_LEAD_MOTOR_ID);
    followTalon = new TalonFX(HOPPER_ROLLER_FOLLOW_MOTOR_ID);

    tryUntilOk(5, () -> leadTalon.getConfigurator().apply(HOPPER_ROLLER_LEAD_TALONFX_CONFIG, 0.25));
    tryUntilOk(
        5, () -> followTalon.getConfigurator().apply(HOPPER_ROLLER_FOLLOW_TALONFX_CONFIG, 0.25));

    // Follow motor opposes lead (inverted in config already, but follow for duty cycle output)
    followTalon.setControl(new Follower(HOPPER_ROLLER_LEAD_MOTOR_ID, MotorAlignmentValue.Aligned));

    leadPosition = leadTalon.getPosition();
    leadVelocity = leadTalon.getVelocity();
    leadAppliedVolts = leadTalon.getMotorVoltage();
    leadStatorCurrent = leadTalon.getStatorCurrent();
    leadTemp = leadTalon.getDeviceTemp();

    followPosition = followTalon.getPosition();
    followVelocity = followTalon.getVelocity();
    followAppliedVolts = followTalon.getMotorVoltage();
    followStatorCurrent = followTalon.getStatorCurrent();
    followTemp = followTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        leadPosition,
        leadVelocity,
        leadAppliedVolts,
        leadStatorCurrent,
        followPosition,
        followVelocity,
        followAppliedVolts,
        followStatorCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1.0, leadTemp, followTemp);

    ParentDevice.optimizeBusUtilizationForAll(leadTalon, followTalon);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    var leadStatus =
        BaseStatusSignal.refreshAll(
            leadPosition, leadVelocity, leadAppliedVolts, leadStatorCurrent, leadTemp);
    var followStatus =
        BaseStatusSignal.refreshAll(
            followPosition, followVelocity, followAppliedVolts, followStatorCurrent, followTemp);

    inputs.leadConnected = leadConnectedDebounce.calculate(leadStatus.isOK());
    inputs.leadPositionRotations = leadPosition.getValueAsDouble();
    inputs.leadVelocityRotPerSec = leadVelocity.getValueAsDouble();
    inputs.leadAppliedVolts = leadAppliedVolts.getValueAsDouble();
    inputs.leadStatorCurrentAmps = leadStatorCurrent.getValueAsDouble();
    inputs.leadTemperatureCelsius = leadTemp.getValueAsDouble();

    inputs.followConnected = followConnectedDebounce.calculate(followStatus.isOK());
    inputs.followPositionRotations = followPosition.getValueAsDouble();
    inputs.followVelocityRotPerSec = followVelocity.getValueAsDouble();
    inputs.followAppliedVolts = followAppliedVolts.getValueAsDouble();
    inputs.followStatorCurrentAmps = followStatorCurrent.getValueAsDouble();
    inputs.followTemperatureCelsius = followTemp.getValueAsDouble();
  }

  @Override
  public void setRollerPercent(double percent) {
    leadTalon.setControl(dutyCycleRequest.withOutput(percent));
    // Follow motor automatically mirrors lead via Follower control
  }
}
