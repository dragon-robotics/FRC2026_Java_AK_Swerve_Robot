// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics-sim implementation of {@link HopperIO}. */
public class HopperIOSim implements HopperIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final DCMotorSim leadSim;
  private final DCMotorSim followSim;
  private double appliedVolts = 0.0;

  public HopperIOSim() {
    DCMotor motor = DCMotor.getKrakenX60(1);
    leadSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.005, 1.0), motor, new double[] {0.0, 0.0});
    followSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, 0.005, 1.0), motor, new double[] {0.0, 0.0});
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    leadSim.setInput(appliedVolts);
    leadSim.update(LOOP_PERIOD_SECS);
    followSim.setInput(appliedVolts);
    followSim.update(LOOP_PERIOD_SECS);

    inputs.leadConnected = true;
    inputs.leadPositionRotations = leadSim.getAngularPositionRotations();
    inputs.leadVelocityRotPerSec = Units.radiansToRotations(leadSim.getAngularVelocityRadPerSec());
    inputs.leadAppliedVolts = appliedVolts;
    inputs.leadStatorCurrentAmps = Math.abs(leadSim.getCurrentDrawAmps());
    inputs.leadTemperatureCelsius = 25.0;

    inputs.followConnected = true;
    inputs.followPositionRotations = followSim.getAngularPositionRotations();
    inputs.followVelocityRotPerSec =
        Units.radiansToRotations(followSim.getAngularVelocityRadPerSec());
    inputs.followAppliedVolts = appliedVolts;
    inputs.followStatorCurrentAmps = Math.abs(followSim.getCurrentDrawAmps());
    inputs.followTemperatureCelsius = 25.0;
  }

  @Override
  public void setRollerPercent(double percent) {
    appliedVolts = MathUtil.clamp(percent, -1.0, 1.0) * 12.0;
  }
}
