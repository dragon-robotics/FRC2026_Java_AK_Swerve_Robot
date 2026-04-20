// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// State machine ported from dragon-robotics/FRC2026_Java_Swerve_Robot
// (feature/shift-timers), adapted to the AdvantageKit IO pattern used by this
// project. DogLog calls are replaced with AdvantageKit's Logger.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_DEPLOYED_POSITION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_FAST_PID_SLOT;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_JUICER_FINAL_POSITION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_JUICER_PRE_POSITION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_POSITION_TOLERANCE;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_SLOW_PID_SLOT;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_STOWED_POSITION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_WOKTOSS_POSITION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ROLLER_DUTY_CYCLE;
import static frc.robot.subsystems.intake.IntakeConstants.OUTTAKE_ROLLER_DUTY_CYCLE;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public enum IntakeState {
    HOME,
    INTAKE,
    OUTTAKE,
    DEPLOYED,
    DEPLOYING,
    STOWING,
    WOKTOSS,
    WOKTOSSING,
    AUTO_WOKTOSSING,
    JUICER
  }

  /** Sub-phases for the JUICER state's timed sequence. */
  public enum JuicerPhase {
    PRE_JUICE,
    SQUEEZE
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeState currIntakeState;
  private IntakeState desiredIntakeState;

  /** State for which CAN commands were last sent, so we can skip redundant writes. */
  private IntakeState lastCommandedState = null;

  /** True once the arm has reached stow in HOME and we've switched to coast. */
  private boolean homeCoasting = false;

  private boolean wokTossMovingToDeployed = false;

  // Juicer sub-phase tracking
  private JuicerPhase juicerPhase = JuicerPhase.PRE_JUICE;
  private JuicerPhase lastJuicerPhase = null;

  public Intake(IntakeIO io) {
    this.io = io;
    this.currIntakeState = IntakeState.HOME;
    this.desiredIntakeState = IntakeState.HOME;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Low-level roller / arm commands
  // ──────────────────────────────────────────────────────────────────────────

  public void runIntakeRollerPercentage(double percentage) {
    io.setRollerPercent(percentage);
  }

  public void runIntake() {
    io.setRollerPercent(INTAKE_ROLLER_DUTY_CYCLE);
  }

  public void runOuttake() {
    io.setRollerPercent(OUTTAKE_ROLLER_DUTY_CYCLE);
  }

  public void stopIntake() {
    io.setRollerPercent(0.0);
  }

  public void setIntakeArmSetpoint(double setpointRotations, int slotId) {
    io.setArmPosition(setpointRotations, slotId);
  }

  public void deployIntakeArm() {
    io.setArmPosition(INTAKE_ARM_DEPLOYED_POSITION, INTAKE_ARM_FAST_PID_SLOT);
  }

  /**
   * Release the arm motor to neutral output (0%). For a slapdown intake, gravity holds the arm down
   * once deployed — no PID needed to maintain the down position.
   */
  public void coastIntakeArm() {
    io.setArmNeutral();
  }

  public void stowIntakeArm() {
    io.setArmPosition(INTAKE_ARM_STOWED_POSITION, INTAKE_ARM_SLOW_PID_SLOT);
  }

  public void wokTossIntakeArm() {
    io.setArmPosition(INTAKE_ARM_WOKTOSS_POSITION, INTAKE_ARM_SLOW_PID_SLOT);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Getters
  // ──────────────────────────────────────────────────────────────────────────

  public IntakeState getCurrentState() {
    return currIntakeState;
  }

  public IntakeState getDesiredState() {
    return desiredIntakeState;
  }

  public double getIntakeArmPosition() {
    return inputs.armPositionRotations;
  }

  public double getIntakeRollerSpeed() {
    return inputs.rollerVelocityRotPerSec;
  }

  public boolean isIntakeArmAtDeployed() {
    return Math.abs(INTAKE_ARM_DEPLOYED_POSITION - inputs.armPositionRotations)
        < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntakeArmAtStowed() {
    return Math.abs(INTAKE_ARM_STOWED_POSITION - inputs.armPositionRotations)
        < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntakeArmAtWokToss() {
    return Math.abs(INTAKE_ARM_WOKTOSS_POSITION - inputs.armPositionRotations)
        < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntakeArmAtPreJuice() {
    return Math.abs(INTAKE_ARM_JUICER_PRE_POSITION - inputs.armPositionRotations)
        < INTAKE_ARM_POSITION_TOLERANCE;
  }

  public boolean isIntaking() {
    return getIntakeRollerSpeed() > 5;
  }

  public boolean isOuttaking() {
    return getIntakeRollerSpeed() < -5;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // State management
  // ──────────────────────────────────────────────────────────────────────────

  public void setDesiredState(IntakeState state) {
    this.desiredIntakeState = state;

    if (this.currIntakeState == state) {
      return;
    }

    // State is changing — reset the guard so the new state sends CAN commands on entry.
    lastCommandedState = null;

    switch (desiredIntakeState) {
      case HOME:
        currIntakeState = IntakeState.STOWING;
        break;
      case INTAKE:
        if (currIntakeState == IntakeState.OUTTAKE
            || currIntakeState == IntakeState.DEPLOYED
            || currIntakeState == IntakeState.INTAKE) {
          currIntakeState = IntakeState.INTAKE;
        } else {
          currIntakeState = IntakeState.DEPLOYING;
        }
        break;
      case OUTTAKE:
        if (currIntakeState == IntakeState.INTAKE
            || currIntakeState == IntakeState.DEPLOYED
            || currIntakeState == IntakeState.OUTTAKE) {
          currIntakeState = IntakeState.OUTTAKE;
        } else {
          currIntakeState = IntakeState.DEPLOYING;
        }
        break;
      case DEPLOYED:
        currIntakeState = IntakeState.DEPLOYING;
        break;
      case WOKTOSS:
        currIntakeState = IntakeState.WOKTOSSING;
        wokTossMovingToDeployed = true;
        break;
      case AUTO_WOKTOSSING:
        currIntakeState = IntakeState.AUTO_WOKTOSSING;
        wokTossMovingToDeployed = true;
        break;
      case JUICER:
        currIntakeState = IntakeState.JUICER;
        // Always restart the juicer sequence from PRE_JUICE on (re-)entry.
        juicerPhase = JuicerPhase.PRE_JUICE;
        lastJuicerPhase = null;
        break;
      default:
        break;
    }
  }

  private void handleStateTransition() {
    switch (currIntakeState) {
        // ── Steady states: only send CAN commands on state entry ──
      case HOME:
        if (lastCommandedState != currIntakeState) {
          // First entry -- command arm to stow via PID and stop rollers
          stowIntakeArm();
          stopIntake();
          lastCommandedState = currIntakeState;
          homeCoasting = false;
        } else if (!homeCoasting && isIntakeArmAtStowed()) {
          // Arm has arrived at stow -- coast to save stator current.
          // Gravity + mechanical stops hold the arm in place; PID is unnecessary.
          coastIntakeArm();
          homeCoasting = true;
        }
        break;

      case INTAKE:
        if (lastCommandedState != currIntakeState) {
          deployIntakeArm();
          runIntake();
          lastCommandedState = currIntakeState;
        }
        break;

      case OUTTAKE:
        if (lastCommandedState != currIntakeState) {
          deployIntakeArm();
          runOuttake();
          lastCommandedState = currIntakeState;
        }
        break;

      case DEPLOYED:
        if (lastCommandedState != currIntakeState) {
          deployIntakeArm();
          stopIntake();
          lastCommandedState = currIntakeState;
        }
        break;

      case WOKTOSS:
        if (lastCommandedState != currIntakeState) {
          wokTossIntakeArm();
          runIntake();
          lastCommandedState = currIntakeState;
        }
        break;

        // ── Transition states: send arm + roller commands once on entry ──
      case DEPLOYING:
        if (lastCommandedState != currIntakeState) {
          deployIntakeArm();
          if (DriverStation.isAutonomous()) {
            runIntakeRollerPercentage(-0.5);
          } else {
            stopIntake();
          }
          lastCommandedState = currIntakeState;
        }
        if (isIntakeArmAtDeployed()) {
          lastCommandedState = null;
          if (desiredIntakeState == IntakeState.INTAKE) {
            currIntakeState = IntakeState.INTAKE;
          } else if (desiredIntakeState == IntakeState.OUTTAKE) {
            currIntakeState = IntakeState.OUTTAKE;
          } else {
            currIntakeState = IntakeState.DEPLOYED;
          }
        }
        break;

      case STOWING:
        if (lastCommandedState != currIntakeState) {
          stowIntakeArm();
          stopIntake();
          lastCommandedState = currIntakeState;
        }
        if (isIntakeArmAtStowed()) {
          lastCommandedState = null;
          currIntakeState = IntakeState.HOME;
        }
        break;

      case WOKTOSSING:
        if (lastCommandedState != currIntakeState) {
          wokTossIntakeArm();
          runIntake();
          lastCommandedState = currIntakeState;
        }
        if (isIntakeArmAtWokToss()) {
          lastCommandedState = null;
          wokTossMovingToDeployed = false;
          currIntakeState = IntakeState.WOKTOSS;
        }
        break;

      case AUTO_WOKTOSSING:
        // TODO: oscillate between deployed and woktoss setpoints on a timed schedule.
        break;

      case JUICER:
        if (lastCommandedState != currIntakeState) {
          lastCommandedState = currIntakeState;
          lastJuicerPhase = null; // Reset so first phase sends its arm command
        }
        switch (juicerPhase) {
          case PRE_JUICE:
            if (lastJuicerPhase != juicerPhase) {
              setIntakeArmSetpoint(INTAKE_ARM_JUICER_PRE_POSITION, INTAKE_ARM_FAST_PID_SLOT);
              lastJuicerPhase = juicerPhase;
            }
            if (isIntakeArmAtPreJuice()) {
              juicerPhase = JuicerPhase.SQUEEZE;
            }
            break;
          case SQUEEZE:
            if (lastJuicerPhase != juicerPhase) {
              setIntakeArmSetpoint(INTAKE_ARM_JUICER_FINAL_POSITION, INTAKE_ARM_SLOW_PID_SLOT);
              // Spin rollers at 50% to help clear remaining balls during the squeeze.
              runIntakeRollerPercentage(0.5);
              lastJuicerPhase = juicerPhase;
            }
            break;
        }
        break;

      default:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    handleStateTransition();

    Logger.recordOutput("Intake/CurrentState", currIntakeState);
    Logger.recordOutput("Intake/DesiredState", desiredIntakeState);
    Logger.recordOutput("Intake/HomeCoasting", homeCoasting);
    Logger.recordOutput("Intake/WokTossMovingToDeployed", wokTossMovingToDeployed);
    Logger.recordOutput("Intake/JuicerPhase", juicerPhase);
  }
}
