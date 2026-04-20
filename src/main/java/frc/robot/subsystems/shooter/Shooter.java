// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public enum ShooterState {
    STOP,
    PREPFUEL,
    SHOOT,
    TRANSITION
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ShooterState currShooterState;
  private ShooterState desiredShooterState;
  private ShooterState lastCommandedState = null;

  // Target setpoints (may be overridden by distance interpolation)
  private double targetRPM = SHOOTER_LEAD_RPM;
  private double hoodAngle = SHOOTER_HOOD_SETTING;

  // Kicker stop timer — delay kicker stop to let fuel clear
  private final Timer kickerStopTimer = new Timer();
  private boolean kickerStopTimerRunning = false;

  // Manual distance override
  private boolean manualDistanceOverride = false;

  public Shooter(ShooterIO io) {
    this.io = io;
    this.currShooterState = ShooterState.STOP;
    this.desiredShooterState = ShooterState.STOP;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Motor commands
  // ──────────────────────────────────────────────────────────────────────────

  private void startShooter() {
    io.setShooterVelocityRPM(targetRPM);
    io.setHoodPosition(hoodAngle);
  }

  private void runKicker() {
    io.setKickerPercent(SHOOTER_KICKER_DUTY_CYCLE);
    kickerStopTimerRunning = false;
  }

  private void stopKickerWithDelay() {
    if (!kickerStopTimerRunning) {
      kickerStopTimer.restart();
      kickerStopTimerRunning = true;
    }
    if (kickerStopTimer.hasElapsed(KICKER_STOP_DELAY_SECS)) {
      io.setKickerPercent(0.0);
      kickerStopTimerRunning = false;
    }
  }

  private void stopAll() {
    io.stopShooter();
    io.setKickerPercent(0.0);
    io.setHoodNeutral();
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Getters / Setters
  // ──────────────────────────────────────────────────────────────────────────

  public ShooterState getCurrentState() {
    return currShooterState;
  }

  public ShooterState getDesiredState() {
    return desiredShooterState;
  }

  public void setManualDistanceOverride(boolean override) {
    this.manualDistanceOverride = override;
  }

  /**
   * Called by Superstructure periodic to update the distance-based setpoints. If manual override is
   * active, defaults are used instead.
   */
  public void setSetpointForDistance(double distanceMeters) {
    if (manualDistanceOverride) {
      targetRPM = SHOOTER_LEAD_RPM;
      hoodAngle = SHOOTER_HOOD_SETTING;
      return;
    }
    ShooterSetpoint sp = getSetpointForDistance(distanceMeters);
    targetRPM = sp.shooterRPM();
    hoodAngle = sp.hoodAngle();
  }

  public boolean isAtTargetRPM() {
    return MathUtil.isNear(targetRPM, inputs.shooterLeadVelocityRPM, 100.0);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // State management
  // ──────────────────────────────────────────────────────────────────────────

  public void setDesiredState(ShooterState state) {
    this.desiredShooterState = state;
    lastCommandedState = null; // Force re-send
  }

  private void handleStateTransition() {
    // Compute the effective state
    switch (desiredShooterState) {
      case STOP:
        currShooterState = ShooterState.STOP;
        break;
      case PREPFUEL:
        currShooterState = ShooterState.PREPFUEL;
        break;
      case SHOOT:
        // TRANSITION until we reach target RPM
        if (currShooterState != ShooterState.SHOOT && currShooterState != ShooterState.TRANSITION) {
          currShooterState = ShooterState.TRANSITION;
        }
        if (currShooterState == ShooterState.TRANSITION && isAtTargetRPM()) {
          currShooterState = ShooterState.SHOOT;
        }
        break;
      default:
        break;
    }

    // Guard against redundant CAN writes
    if (currShooterState == lastCommandedState) {
      return;
    }
    lastCommandedState = currShooterState;

    switch (currShooterState) {
      case STOP:
        stopAll();
        break;
      case PREPFUEL:
        startShooter();
        stopKickerWithDelay();
        break;
      case TRANSITION:
        startShooter();
        stopKickerWithDelay();
        break;
      case SHOOT:
        startShooter();
        runKicker();
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    handleStateTransition();

    Logger.recordOutput("Shooter/CurrentState", currShooterState);
    Logger.recordOutput("Shooter/DesiredState", desiredShooterState);
    Logger.recordOutput("Shooter/TargetRPM", targetRPM);
    Logger.recordOutput("Shooter/HoodAngle", hoodAngle);
    Logger.recordOutput("Shooter/AtTargetRPM", isAtTargetRPM());
    Logger.recordOutput("Shooter/ManualDistanceOverride", manualDistanceOverride);
  }
}
