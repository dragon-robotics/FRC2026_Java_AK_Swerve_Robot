// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * AdvantageKit IO layer for the intake subsystem (roller + arm). The arm runs closed-loop position
 * control on the TalonFX (fused CANcoder); the roller runs open-loop duty cycle.
 */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // ── Roller ─────────────────────────────────────────────────────────────
    public boolean rollerConnected = false;
    public double rollerPositionRotations = 0.0;
    public double rollerVelocityRotPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public double rollerTemperatureCelsius = 0.0;

    // ── Arm ────────────────────────────────────────────────────────────────
    public boolean armConnected = false;
    public boolean armEncoderConnected = false;
    public double armPositionRotations = 0.0;
    public double armVelocityRotPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armSupplyCurrentAmps = 0.0;
    public double armStatorCurrentAmps = 0.0;
    public double armTemperatureCelsius = 0.0;
  }

  /** Refresh all inputs from the hardware. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake roller at a percentage duty cycle [-1, 1]. */
  default void setRollerPercent(double percent) {}

  /** Command the arm to a position (rotations) using the given PID slot. */
  default void setArmPosition(double positionRotations, int slotId) {}

  /** Release the arm motor to neutral output (0% duty cycle). */
  default void setArmNeutral() {}
}
