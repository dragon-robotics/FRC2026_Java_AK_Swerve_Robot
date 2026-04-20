// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

/**
 * AdvantageKit IO layer for the hopper subsystem (lead roller + follow roller). Both motors run
 * open-loop duty cycle.
 */
public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean leadConnected = false;
    public double leadPositionRotations = 0.0;
    public double leadVelocityRotPerSec = 0.0;
    public double leadAppliedVolts = 0.0;
    public double leadStatorCurrentAmps = 0.0;
    public double leadTemperatureCelsius = 0.0;

    public boolean followConnected = false;
    public double followPositionRotations = 0.0;
    public double followVelocityRotPerSec = 0.0;
    public double followAppliedVolts = 0.0;
    public double followStatorCurrentAmps = 0.0;
    public double followTemperatureCelsius = 0.0;
  }

  /** Refresh all inputs from the hardware. */
  default void updateInputs(HopperIOInputs inputs) {}

  /** Run the lead roller at a percentage duty cycle [-1, 1]. The follow motor mirrors. */
  default void setRollerPercent(double percent) {}
}
