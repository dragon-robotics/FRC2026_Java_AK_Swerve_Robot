// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_GEAR_RATIO;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_LENGTH_METERS;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ARM_MASS_KG;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_MAX_ANGLE_RADIANS;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_MIN_ANGLE_RADIANS;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_STARTING_ANGLE_RADIANS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Physics-sim implementation of {@link IntakeIO}. Roller is a simple flywheel-ish DC motor model;
 * arm uses a {@link SingleJointedArmSim} with gravity. Control loops run in Java to stand in for
 * the Talon's onboard loop.
 */
public class IntakeIOSim implements IntakeIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final DCMotorSim rollerSim;
  private final SingleJointedArmSim armSim;

  // Roller: open-loop voltage from percent command
  private double rollerAppliedVolts = 0.0;

  // Arm control
  private boolean armClosedLoop = false;
  private double armTargetRotations = INTAKE_STARTING_ANGLE_RADIANS / (2.0 * Math.PI);
  private final PIDController armFastPid = new PIDController(40.0, 0.0, 0.0);
  private final PIDController armSlowPid = new PIDController(14.0, 0.0, 0.0);
  private PIDController activeArmPid = armFastPid;
  private double armAppliedVolts = 0.0;

  public IntakeIOSim() {
    DCMotor rollerMotor = DCMotor.getKrakenX60(1);
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rollerMotor, 0.0025, 1.0),
            rollerMotor,
            new double[] {0.0, 0.0});

    armSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            INTAKE_ARM_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(INTAKE_ARM_LENGTH_METERS, INTAKE_ARM_MASS_KG),
            INTAKE_ARM_LENGTH_METERS,
            INTAKE_MIN_ANGLE_RADIANS,
            INTAKE_MAX_ANGLE_RADIANS,
            true,
            INTAKE_STARTING_ANGLE_RADIANS);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // ── Arm closed loop (runs in Java in sim) ─────────────────────────────
    if (armClosedLoop) {
      double currentRotations = armRotationsFromSim();
      double output = activeArmPid.calculate(currentRotations, armTargetRotations);
      armAppliedVolts = MathUtil.clamp(output, -10.0, 10.0);
    }
    armSim.setInput(armAppliedVolts);
    armSim.update(LOOP_PERIOD_SECS);

    rollerSim.setInput(rollerAppliedVolts);
    rollerSim.update(LOOP_PERIOD_SECS);

    // ── Roller telemetry ──────────────────────────────────────────────────
    inputs.rollerConnected = true;
    inputs.rollerPositionRotations = rollerSim.getAngularPositionRotations();
    inputs.rollerVelocityRotPerSec =
        Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec());
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerStatorCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
    inputs.rollerTemperatureCelsius = 25.0;

    // ── Arm telemetry ─────────────────────────────────────────────────────
    inputs.armConnected = true;
    inputs.armEncoderConnected = true;
    inputs.armPositionRotations = armRotationsFromSim();
    inputs.armVelocityRotPerSec = Units.radiansToRotations(armSim.getVelocityRadPerSec());
    inputs.armAppliedVolts = armAppliedVolts;
    inputs.armStatorCurrentAmps = Math.abs(armSim.getCurrentDrawAmps());
    inputs.armTemperatureCelsius = 25.0;
  }

  private double armRotationsFromSim() {
    // IntakeConstants positions are in mechanism rotations (after CANcoder reduction).
    // The sim reports mechanism angle directly in radians.
    return Units.radiansToRotations(armSim.getAngleRads());
  }

  @Override
  public void setRollerPercent(double percent) {
    rollerAppliedVolts = MathUtil.clamp(percent, -1.0, 1.0) * 12.0;
  }

  @Override
  public void setArmPosition(double positionRotations, int slotId) {
    armClosedLoop = true;
    armTargetRotations = positionRotations;
    activeArmPid = (slotId == IntakeConstants.INTAKE_ARM_FAST_PID_SLOT) ? armFastPid : armSlowPid;
  }

  @Override
  public void setArmNeutral() {
    armClosedLoop = false;
    armAppliedVolts = 0.0;
  }
}
