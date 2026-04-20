// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_CANCODER_CONFIG;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_CANCODER_ID;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_MOTOR_ID;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_TALONFX_CONFIG;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ROLLER_MOTOR_ID;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ROLLER_TALONFX_CONFIG;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Real TalonFX-based implementation of {@link IntakeIO}. */
public class IntakeIOTalonFX implements IntakeIO {
  protected final TalonFX rollerTalon;
  protected final TalonFX armTalon;
  protected final CANcoder armCancoder;

  private final DutyCycleOut rollerDutyCycle = new DutyCycleOut(0.0).withEnableFOC(true);
  private final DutyCycleOut armDutyCycle = new DutyCycleOut(0.0).withEnableFOC(true);
  private final PositionVoltage armPositionRequest = new PositionVoltage(0.0).withEnableFOC(true);

  // Roller signals
  private final StatusSignal<Angle> rollerPosition;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerSupplyCurrent;
  private final StatusSignal<Current> rollerStatorCurrent;
  private final StatusSignal<Temperature> rollerTemp;

  // Arm signals
  private final StatusSignal<Angle> armPosition;
  private final StatusSignal<AngularVelocity> armVelocity;
  private final StatusSignal<Voltage> armAppliedVolts;
  private final StatusSignal<Current> armSupplyCurrent;
  private final StatusSignal<Current> armStatorCurrent;
  private final StatusSignal<Temperature> armTemp;
  private final StatusSignal<Angle> armCancoderAbsPosition;

  private final Debouncer rollerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer armConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer armEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOTalonFX() {
    rollerTalon = new TalonFX(INTAKE_ROLLER_MOTOR_ID);
    armTalon = new TalonFX(INTAKE_ARM_MOTOR_ID);
    armCancoder = new CANcoder(INTAKE_ARM_CANCODER_ID);

    // Apply configs
    tryUntilOk(5, () -> armCancoder.getConfigurator().apply(INTAKE_ARM_CANCODER_CONFIG, 0.25));
    tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(INTAKE_ROLLER_TALONFX_CONFIG, 0.25));
    tryUntilOk(5, () -> armTalon.getConfigurator().apply(INTAKE_ARM_TALONFX_CONFIG, 0.25));

    rollerPosition = rollerTalon.getPosition();
    rollerVelocity = rollerTalon.getVelocity();
    rollerAppliedVolts = rollerTalon.getMotorVoltage();
    rollerSupplyCurrent = rollerTalon.getSupplyCurrent();
    rollerStatorCurrent = rollerTalon.getStatorCurrent();
    rollerTemp = rollerTalon.getDeviceTemp();

    armPosition = armTalon.getPosition();
    armVelocity = armTalon.getVelocity();
    armAppliedVolts = armTalon.getMotorVoltage();
    armSupplyCurrent = armTalon.getSupplyCurrent();
    armStatorCurrent = armTalon.getStatorCurrent();
    armTemp = armTalon.getDeviceTemp();
    armCancoderAbsPosition = armCancoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        rollerPosition,
        rollerVelocity,
        rollerAppliedVolts,
        rollerSupplyCurrent,
        rollerStatorCurrent,
        armPosition,
        armVelocity,
        armAppliedVolts,
        armSupplyCurrent,
        armStatorCurrent,
        armCancoderAbsPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(1.0, rollerTemp, armTemp);

    ParentDevice.optimizeBusUtilizationForAll(rollerTalon, armTalon, armCancoder);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var rollerStatus =
        BaseStatusSignal.refreshAll(
            rollerPosition, rollerVelocity, rollerAppliedVolts, rollerStatorCurrent, rollerTemp);
    var armStatus =
        BaseStatusSignal.refreshAll(
            armPosition, armVelocity, armAppliedVolts, armStatorCurrent, armTemp);
    var armEncoderStatus = BaseStatusSignal.refreshAll(armCancoderAbsPosition);

    inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.rollerPositionRotations = rollerPosition.getValueAsDouble();
    inputs.rollerVelocityRotPerSec = rollerVelocity.getValueAsDouble();
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
    inputs.rollerStatorCurrentAmps = rollerStatorCurrent.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemp.getValueAsDouble();

    inputs.armConnected = armConnectedDebounce.calculate(armStatus.isOK());
    inputs.armEncoderConnected = armEncoderConnectedDebounce.calculate(armEncoderStatus.isOK());
    inputs.armPositionRotations = armPosition.getValueAsDouble();
    inputs.armVelocityRotPerSec = armVelocity.getValueAsDouble();
    inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.armStatorCurrentAmps = armStatorCurrent.getValueAsDouble();
    inputs.armSupplyCurrentAmps = armSupplyCurrent.getValueAsDouble();
    inputs.armTemperatureCelsius = armTemp.getValueAsDouble();
  }

  @Override
  public void setRollerPercent(double percent) {
    rollerTalon.setControl(rollerDutyCycle.withOutput(percent));
  }

  @Override
  public void setArmPosition(double positionRotations, int slotId) {
    armTalon.setControl(armPositionRequest.withPosition(positionRotations).withSlot(slotId));
  }

  @Override
  public void setArmNeutral() {
    armTalon.setControl(armDutyCycle.withOutput(0.0));
  }
}
