// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * AdvantageKit IO layer for the shooter subsystem. The shooter has four motors: lead flywheel
 * (velocity control), follow flywheel (follower), kicker (duty-cycle), and hood (position control
 * with CANcoder feedback).
 */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // ── Lead flywheel ──────────────────────────────────────────────────────
    public boolean shooterLeadConnected = false;
    public double shooterLeadPositionRotations = 0.0;
    public double shooterLeadVelocityRPM = 0.0;
    public double shooterLeadAppliedVolts = 0.0;
    public double shooterLeadStatorCurrentAmps = 0.0;
    public double shooterLeadTemperatureCelsius = 0.0;

    // ── Follow flywheel ────────────────────────────────────────────────────
    public boolean shooterFollowConnected = false;
    public double shooterFollowVelocityRPM = 0.0;
    public double shooterFollowAppliedVolts = 0.0;
    public double shooterFollowStatorCurrentAmps = 0.0;
    public double shooterFollowTemperatureCelsius = 0.0;

    // ── Kicker ─────────────────────────────────────────────────────────────
    public boolean kickerConnected = false;
    public double kickerVelocityRPM = 0.0;
    public double kickerAppliedVolts = 0.0;
    public double kickerStatorCurrentAmps = 0.0;
    public double kickerTemperatureCelsius = 0.0;

    // ── Hood ───────────────────────────────────────────────────────────────
    public boolean hoodConnected = false;
    public boolean hoodEncoderConnected = false;
    public double hoodPositionRotations = 0.0;
    public double hoodVelocityRotPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodStatorCurrentAmps = 0.0;
    public double hoodTemperatureCelsius = 0.0;
  }

  /** Refresh all inputs from the hardware. */
  default void updateInputs(ShooterIOInputs inputs) {}

  /** Command the lead flywheel to a velocity (RPM). The follow motor mirrors. */
  default void setShooterVelocityRPM(double rpm) {}

  /** Stop the flywheel motors. */
  default void stopShooter() {}

  /** Run the kicker at a percentage duty cycle [-1, 1]. */
  default void setKickerPercent(double percent) {}

  /** Command the hood to a position (rotations). */
  default void setHoodPosition(double positionRotations) {}

  /** Release the hood motor to neutral output. */
  default void setHoodNeutral() {}
}
