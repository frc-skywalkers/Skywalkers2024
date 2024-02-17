// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import org.littletonrobotics.junction.Logger;

public class IntakeCommands {

  private IntakeCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command runWheelVolts(Intake intake) {
    return Commands.run(
        () -> {
          intake.runWheelVolts(3.0);
        },
        intake);
  }

  public static Command intakePiece(Intake intake) {
    return Commands.run(
            () -> {
              intake.setPosition(IntakeConstants.dropDown);
              intake.runWheel();
              Logger.recordOutput("Intake/intakingPiece", true);
            },
            intake)
        .withTimeout(0.5);
    // .until(() -> intake.hasPiece());
    // .until(() -> intake.atPosition(IntakeConstants.dropDown));
  }

  public static Command gotPiece(Intake intake) {
    return Commands.run(
            () -> {
              intake.setPosition(IntakeConstants.dropDown);
              intake.runWheel();
            })
        .until(() -> intake.hasPiece());
  }

  public static Command passPieceIntake(Intake intake, Pivot pivot, Indexer indexer) {
    return Commands.run(
            () -> {
              intake.setPosition(IntakeConstants.handoff);
              intake.holdPiece();
              pivot.setPosition(PivotConstants.handoff);
            },
            intake,
            pivot,
            indexer)
        .until(
            () ->
                intake.atPosition(IntakeConstants.handoff)
                    && pivot.atPosition(PivotConstants.handoff));
    // .withTimeout(5.0);
  }

  public static Command transferPiece(Intake intake, Indexer indexer) {
    return Commands.run(
            () -> {
              intake.outtakeWheel();
              indexer.runVolts(IndexerConstants.indexVolts);
            },
            intake,
            indexer)
        .until(() -> indexer.hasPiece())
        .andThen(
            () -> {
              indexer.runVolts(IndexerConstants.holdVolts);
              intake.stopWheels();
            });
  }

  public static Command bringOutPiece(Indexer indexer) {
    return Commands.run(
            () -> {
              indexer.runVolts(-0.2);
            },
            indexer)
        .withTimeout(SmartDashboard.getNumber("Indexer Position Back", 0.15))
        .andThen(() -> indexer.stop());
  }

  public static Command intakeHandoff(Intake intake, Indexer indexer, Pivot pivot) {
    return intakePiece(intake)
        .andThen(gotPiece(intake))
        .andThen(passPieceIntake(intake, pivot, indexer))
        .andThen(transferPiece(intake, indexer))
        .andThen(bringOutPiece(indexer));
  }
}
