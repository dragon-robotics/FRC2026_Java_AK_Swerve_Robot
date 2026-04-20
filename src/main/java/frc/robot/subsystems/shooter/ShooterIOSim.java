// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Physics-sim implementation of {@link ShooterIO}. */
public class ShooterIOSim implements ShooterIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  // Flywheel sims (lead + follow share the same setpoint in sim)
  private final DCMotorSim shooterSim;
  private final DCMotorSim kickerSim;

  // Hood sim — modeled as single-jointed arm
  private final SingleJointedArmSim hoodSim;
  private final PIDController hoodPID = new PIDController(50.0, 0.0, 0.5);

  private double shooterAppliedVolts = 0.0;
  private double kickerAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;
  private boolean hoodClosedLoop = false;
  private double hoodSetpointRot = 0.0;

  public ShooterIOSim() {
    DCMotor flywheelMotor = DCMotor.getKrakenX60(2);
    shooterSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(flywheelMotor, 0.005, 1.0),
            flywheelMotor,
            new double[] {0.0, 0.0});

    DCMotor kickerMotor = DCMotor.getKrakenX60(1);
    kickerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kickerMotor, 0.002, 1.0),
            kickerMotor,
            new double[] {0.0, 0.0});

    hoodSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            50.0,
            SingleJointedArmSim.estimateMOI(0.3, 2.0),
            0.3,
            Units.degreesToRadians(-10.0),
            Units.degreesToRadians(90.0),
            true,
            0.0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Flywheel
    shooterSim.setInput(shooterAppliedVolts);
    shooterSim.update(LOOP_PERIOD_SECS);

    // Kicker
    kickerSim.setInput(kickerAppliedVolts);
    kickerSim.update(LOOP_PERIOD_SECS);

    // Hood (closed-loop in sim)
    if (hoodClosedLoop) {
      double currentRot = Units.radiansToRotations(hoodSim.getAngleRads());
      double pidVolts = hoodPID.calculate(currentRot, hoodSetpointRot);
      hoodAppliedVolts = MathUtil.clamp(pidVolts, -10.0, 10.0);
    }
    hoodSim.setInput(hoodAppliedVolts);
    hoodSim.update(LOOP_PERIOD_SECS);

    // Lead flywheel
    inputs.shooterLeadConnected = true;
    inputs.shooterLeadPositionRotations = shooterSim.getAngularPositionRotations();
    inputs.shooterLeadVelocityRPM =
        Units.radiansToRotations(shooterSim.getAngularVelocityRadPerSec()) * 60.0;
    inputs.shooterLeadAppliedVolts = shooterAppliedVolts;
    inputs.shooterLeadStatorCurrentAmps = Math.abs(shooterSim.getCurrentDrawAmps());
    inputs.shooterLeadTemperatureCelsius = 25.0;

    // Follow flywheel (mirrors lead in sim)
    inputs.shooterFollowConnected = true;
    inputs.shooterFollowVelocityRPM = inputs.shooterLeadVelocityRPM;
    inputs.shooterFollowAppliedVolts = shooterAppliedVolts;
    inputs.shooterFollowStatorCurrentAmps = inputs.shooterLeadStatorCurrentAmps;
    inputs.shooterFollowTemperatureCelsius = 25.0;

    // Kicker
    inputs.kickerConnected = true;
    inputs.kickerVelocityRPM =
        Units.radiansToRotations(kickerSim.getAngularVelocityRadPerSec()) * 60.0;
    inputs.kickerAppliedVolts = kickerAppliedVolts;
    inputs.kickerStatorCurrentAmps = Math.abs(kickerSim.getCurrentDrawAmps());
    inputs.kickerTemperatureCelsius = 25.0;

    // Hood
    inputs.hoodConnected = true;
    inputs.hoodEncoderConnected = true;
    inputs.hoodPositionRotations = Units.radiansToRotations(hoodSim.getAngleRads());
    inputs.hoodVelocityRotPerSec = Units.radiansToRotations(hoodSim.getVelocityRadPerSec());
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodStatorCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
    inputs.hoodTemperatureCelsius = 25.0;
  }

  @Override
  public void setShooterVelocityRPM(double rpm) {
    // Simple voltage-proportional approximation for sim
    double maxRPM = 6000.0;
    shooterAppliedVolts = MathUtil.clamp((rpm / maxRPM) * 12.0, -12.0, 12.0);
  }

  @Override
  public void stopShooter() {
    shooterAppliedVolts = 0.0;
  }

  @Override
  public void setKickerPercent(double percent) {
    kickerAppliedVolts = MathUtil.clamp(percent, -1.0, 1.0) * 12.0;
  }

  @Override
  public void setHoodPosition(double positionRotations) {
    hoodClosedLoop = true;
    hoodSetpointRot = positionRotations;
  }

  @Override
  public void setHoodNeutral() {
    hoodClosedLoop = false;
    hoodAppliedVolts = 0.0;
  }
}
