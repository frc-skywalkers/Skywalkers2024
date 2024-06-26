// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PivotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final ArmFeedforward ffModel;

  private double autoPivotAngle = -1.5;

  /** Creates a new Pivot. */
  public Pivot(PivotIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new ArmFeedforward(0.13, 0.15, 24.0); // need to determine
        io.configurePIDFF(15.0, 0.0, 0.15); // need to determine
        break;
      case REPLAY:
        ffModel = new ArmFeedforward(0.1, 0.05, 0);
        io.configurePIDFF(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new ArmFeedforward(0.0945, 0.5185, 1.5);
        io.configurePIDFF(6.0, 0.0, 0.0);
        break;
      default:
        ffModel = new ArmFeedforward(0.0, 0.0, 0);
        break;
    }
    Logger.recordOutput("Pivot/atPositions", false);
    updateTable();
  }

  public void updateTable() {
    // PivotConstants.pivotAngleMap.put(125.0, 450.0);
    PivotConstants.pivotAngleMap.put(2.405, -0.825);
    PivotConstants.pivotAngleMap.put(2.128, -0.89);
    PivotConstants.pivotAngleMap.put(1.950, -0.925);
    PivotConstants.pivotAngleMap.put(1.448, -1.05);
    PivotConstants.pivotAngleMap.put(2.807, -0.75);
    PivotConstants.pivotAngleMap.put(3.251, -0.72);
    PivotConstants.pivotAngleMap.put(3.658, -0.665);

    // PivotConstants.pivotAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 0.0, -1.05);
    // PivotConstants.pivotAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 25.0, -0.95);
    // PivotConstants.pivotAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 13.5, -1.00);
    // PivotConstants.pivotAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 34.0, -0.90);
    // PivotConstants.pivotAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 43.0, -0.85);
    // PivotConstants.pivotAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 50.0, -0.82);
    // PivotConstants.pivotAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 84.75,
    // -0.73);
    // PivotConstants.pivotAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 68.0,
    // -0.757);
    // PivotConstants.pivotAngleMap.put(FieldConstants.robotSubwooferSpeakerDistance + 101.0,
    // -0.700);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(("Pivot"), inputs);
    // Logger.recordOutput("Pivot/atPosition", atPosition(PivotConstants.handoff));
  }

  public void runVolts(double volts) {
    io.setVoltage(volts + Math.cos(getPositionRad()) * 0.08);
  }

  public void setPosition(double positionRad) {
    inputs.goalPos = positionRad;
    if (Constants.currentMode == Mode.SIM) {
      io.setPosition(positionRad, ffModel.calculate(inputs.setpointPos, inputs.goalVel));
    } else {
      io.setPosition(positionRad, ffModel.calculate(inputs.positionRad, inputs.goalVel));
    }
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput
  public double getVelocityRad() {
    return inputs.velocityRadPerSec;
  }

  @AutoLogOutput
  public double getPositionRad() {
    return inputs.positionRad;
  }

  public double getCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.velocityRadPerSec);
  }

  public boolean atPosition() {
    return Math.abs(getPositionRad() - inputs.goalPos) < PivotConstants.tolerance;
  }

  public void updateAutoPivot(double ang) {
    this.autoPivotAngle = ang;
  }

  public boolean pivotAligned() {
    return atPosition(this.autoPivotAngle);
  }

  public boolean atPosition(double goalPos) {
    boolean ret = Math.abs(getPositionRad() - goalPos) < PivotConstants.tolerance;
    Logger.recordOutput("Pivot/atPositions", ret);
    return ret;
  }
}
