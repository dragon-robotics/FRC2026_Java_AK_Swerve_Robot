// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.HOPPER_ROLLER_DUTY_CYCLE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  public enum HopperState {
    STOP,
    INDEXTOSHOOTER,
    INDEXTOINTAKE
  }

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private HopperState currHopperState;
  private HopperState desiredHopperState;
  private HopperState lastCommandedState = null;

  public Hopper(HopperIO io) {
    this.io = io;
    this.currHopperState = HopperState.STOP;
    this.desiredHopperState = HopperState.STOP;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Motor commands
  // ──────────────────────────────────────────────────────────────────────────

  public void runHopperRollerPercentage(double percentage) {
    io.setRollerPercent(percentage);
  }

  public void indexToShooter() {
    io.setRollerPercent(HOPPER_ROLLER_DUTY_CYCLE);
  }

  public void indexToIntake() {
    io.setRollerPercent(-HOPPER_ROLLER_DUTY_CYCLE);
  }

  public void stopHopperRoller() {
    io.setRollerPercent(0.0);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Getters
  // ──────────────────────────────────────────────────────────────────────────

  public HopperState getCurrentState() {
    return currHopperState;
  }

  public HopperState getDesiredState() {
    return desiredHopperState;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // State management
  // ──────────────────────────────────────────────────────────────────────────

  public void setDesiredState(HopperState state) {
    this.desiredHopperState = state;
    this.currHopperState = state;
    lastCommandedState = null; // Force re-send on next periodic
  }

  private void handleStateTransition() {
    if (currHopperState == lastCommandedState) {
      return;
    }
    lastCommandedState = currHopperState;

    switch (currHopperState) {
      case STOP:
        stopHopperRoller();
        break;
      case INDEXTOSHOOTER:
        indexToShooter();
        break;
      case INDEXTOINTAKE:
        indexToIntake();
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    handleStateTransition();

    Logger.recordOutput("Hopper/CurrentState", currHopperState);
  }
}
