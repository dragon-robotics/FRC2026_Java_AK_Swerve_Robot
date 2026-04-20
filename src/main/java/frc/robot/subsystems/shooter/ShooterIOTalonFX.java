// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Real TalonFX-based implementation of {@link ShooterIO}. */
public class ShooterIOTalonFX implements ShooterIO {
  // ── Hardware ────────────────────────────────────────────────────────────
  private final TalonFX shooterLeadTalon;
  private final TalonFX shooterFollowTalon;
  private final TalonFX kickerTalon;
  private final TalonFX hoodTalon;
  private final CANcoder hoodCancoder;

  // ── Control requests ────────────────────────────────────────────────────
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final DutyCycleOut kickerDutyRequest = new DutyCycleOut(0.0).withEnableFOC(true);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0.0).withEnableFOC(true);
  private final NeutralOut neutralOut = new NeutralOut();

  // ── Lead shooter signals ────────────────────────────────────────────────
  private final StatusSignal<Angle> leadPosition;
  private final StatusSignal<AngularVelocity> leadVelocity;
  private final StatusSignal<Voltage> leadAppliedVolts;
  private final StatusSignal<Current> leadStatorCurrent;
  private final StatusSignal<Temperature> leadTemp;

  // ── Follow shooter signals ──────────────────────────────────────────────
  private final StatusSignal<AngularVelocity> followVelocity;
  private final StatusSignal<Voltage> followAppliedVolts;
  private final StatusSignal<Current> followStatorCurrent;
  private final StatusSignal<Temperature> followTemp;

  // ── Kicker signals ──────────────────────────────────────────────────────
  private final StatusSignal<AngularVelocity> kickerVelocity;
  private final StatusSignal<Voltage> kickerAppliedVolts;
  private final StatusSignal<Current> kickerStatorCurrent;
  private final StatusSignal<Temperature> kickerTemp;

  // ── Hood signals ────────────────────────────────────────────────────────
  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodStatorCurrent;
  private final StatusSignal<Temperature> hoodTemp;
  private final StatusSignal<Angle> hoodEncoderPosition;

  // ── Debouncers ──────────────────────────────────────────────────────────
  private final Debouncer leadConnDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followConnDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer kickerConnDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer hoodConnDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer hoodEncoderConnDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ShooterIOTalonFX() {
    shooterLeadTalon = new TalonFX(SHOOTER_LEAD_MOTOR_ID);
    shooterFollowTalon = new TalonFX(SHOOTER_FOLLOW_MOTOR_ID);
    kickerTalon = new TalonFX(SHOOTER_KICKER_MOTOR_ID);
    hoodTalon = new TalonFX(SHOOTER_HOOD_MOTOR_ID);
    hoodCancoder = new CANcoder(SHOOTER_CANCODER_ID);

    // Apply configs
    tryUntilOk(
        5, () -> shooterLeadTalon.getConfigurator().apply(SHOOTER_LEAD_TALONFX_CONFIG, 0.25));
    tryUntilOk(
        5, () -> shooterFollowTalon.getConfigurator().apply(SHOOTER_FOLLOW_TALONFX_CONFIG, 0.25));
    tryUntilOk(5, () -> kickerTalon.getConfigurator().apply(SHOOTER_KICKER_TALONFX_CONFIG, 0.25));
    tryUntilOk(5, () -> hoodTalon.getConfigurator().apply(SHOOTER_HOOD_TALONFX_CONFIG, 0.25));
    tryUntilOk(5, () -> hoodCancoder.getConfigurator().apply(SHOOTER_HOOD_CANCODER_CONFIG, 0.25));

    // Follow motor mirrors lead
    shooterFollowTalon.setControl(new Follower(SHOOTER_LEAD_MOTOR_ID, MotorAlignmentValue.Aligned));

    // Cache status signals
    leadPosition = shooterLeadTalon.getPosition();
    leadVelocity = shooterLeadTalon.getVelocity();
    leadAppliedVolts = shooterLeadTalon.getMotorVoltage();
    leadStatorCurrent = shooterLeadTalon.getStatorCurrent();
    leadTemp = shooterLeadTalon.getDeviceTemp();

    followVelocity = shooterFollowTalon.getVelocity();
    followAppliedVolts = shooterFollowTalon.getMotorVoltage();
    followStatorCurrent = shooterFollowTalon.getStatorCurrent();
    followTemp = shooterFollowTalon.getDeviceTemp();

    kickerVelocity = kickerTalon.getVelocity();
    kickerAppliedVolts = kickerTalon.getMotorVoltage();
    kickerStatorCurrent = kickerTalon.getStatorCurrent();
    kickerTemp = kickerTalon.getDeviceTemp();

    hoodPosition = hoodTalon.getPosition();
    hoodVelocity = hoodTalon.getVelocity();
    hoodAppliedVolts = hoodTalon.getMotorVoltage();
    hoodStatorCurrent = hoodTalon.getStatorCurrent();
    hoodTemp = hoodTalon.getDeviceTemp();
    hoodEncoderPosition = hoodCancoder.getAbsolutePosition();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        leadPosition,
        leadVelocity,
        leadAppliedVolts,
        leadStatorCurrent,
        followVelocity,
        followAppliedVolts,
        followStatorCurrent,
        kickerVelocity,
        kickerAppliedVolts,
        kickerStatorCurrent,
        hoodPosition,
        hoodVelocity,
        hoodAppliedVolts,
        hoodStatorCurrent,
        hoodEncoderPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(1.0, leadTemp, followTemp, kickerTemp, hoodTemp);

    ParentDevice.optimizeBusUtilizationForAll(
        shooterLeadTalon, shooterFollowTalon, kickerTalon, hoodTalon, hoodCancoder);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var leadStatus =
        BaseStatusSignal.refreshAll(
            leadPosition, leadVelocity, leadAppliedVolts, leadStatorCurrent, leadTemp);
    var followStatus =
        BaseStatusSignal.refreshAll(
            followVelocity, followAppliedVolts, followStatorCurrent, followTemp);
    var kickerStatus =
        BaseStatusSignal.refreshAll(
            kickerVelocity, kickerAppliedVolts, kickerStatorCurrent, kickerTemp);
    var hoodStatus =
        BaseStatusSignal.refreshAll(
            hoodPosition, hoodVelocity, hoodAppliedVolts, hoodStatorCurrent, hoodTemp);
    var hoodEncoderStatus = BaseStatusSignal.refreshAll(hoodEncoderPosition);

    // Lead
    inputs.shooterLeadConnected = leadConnDebounce.calculate(leadStatus.isOK());
    inputs.shooterLeadPositionRotations = leadPosition.getValueAsDouble();
    inputs.shooterLeadVelocityRPM = leadVelocity.getValueAsDouble() * 60.0;
    inputs.shooterLeadAppliedVolts = leadAppliedVolts.getValueAsDouble();
    inputs.shooterLeadStatorCurrentAmps = leadStatorCurrent.getValueAsDouble();
    inputs.shooterLeadTemperatureCelsius = leadTemp.getValueAsDouble();

    // Follow
    inputs.shooterFollowConnected = followConnDebounce.calculate(followStatus.isOK());
    inputs.shooterFollowVelocityRPM = followVelocity.getValueAsDouble() * 60.0;
    inputs.shooterFollowAppliedVolts = followAppliedVolts.getValueAsDouble();
    inputs.shooterFollowStatorCurrentAmps = followStatorCurrent.getValueAsDouble();
    inputs.shooterFollowTemperatureCelsius = followTemp.getValueAsDouble();

    // Kicker
    inputs.kickerConnected = kickerConnDebounce.calculate(kickerStatus.isOK());
    inputs.kickerVelocityRPM = kickerVelocity.getValueAsDouble() * 60.0;
    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerStatorCurrentAmps = kickerStatorCurrent.getValueAsDouble();
    inputs.kickerTemperatureCelsius = kickerTemp.getValueAsDouble();

    // Hood
    inputs.hoodConnected = hoodConnDebounce.calculate(hoodStatus.isOK());
    inputs.hoodEncoderConnected = hoodEncoderConnDebounce.calculate(hoodEncoderStatus.isOK());
    inputs.hoodPositionRotations = hoodPosition.getValueAsDouble();
    inputs.hoodVelocityRotPerSec = hoodVelocity.getValueAsDouble();
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.hoodStatorCurrentAmps = hoodStatorCurrent.getValueAsDouble();
    inputs.hoodTemperatureCelsius = hoodTemp.getValueAsDouble();
  }

  @Override
  public void setShooterVelocityRPM(double rpm) {
    // VelocityTorqueCurrentFOC expects rotations per second
    shooterLeadTalon.setControl(velocityRequest.withVelocity(rpm / 60.0));
    // Follow motor automatically mirrors via Follower
  }

  @Override
  public void stopShooter() {
    shooterLeadTalon.setControl(neutralOut);
  }

  @Override
  public void setKickerPercent(double percent) {
    kickerTalon.setControl(kickerDutyRequest.withOutput(percent));
  }

  @Override
  public void setHoodPosition(double positionRotations) {
    hoodTalon.setControl(hoodPositionRequest.withPosition(positionRotations));
  }

  @Override
  public void setHoodNeutral() {
    hoodTalon.setControl(neutralOut);
  }
}
