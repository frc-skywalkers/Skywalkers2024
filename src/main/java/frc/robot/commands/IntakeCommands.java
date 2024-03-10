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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LightstripConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Lightstrip;
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
              intake.runWheelVolts(-7.0);
            },
            intake)
        .withTimeout(3.0)
        .andThen(
            () -> {
              intake.stopWheels();
            },
            intake);
  }

  public static Command homeIntake(Intake intake) {
    return Commands.run(
            () -> {
              intake.runVolts(-2.0);
            },
            intake)
        .until(() -> intake.isHoned())
        .andThen(
            () -> {
              intake.resetPosition();
              intake.stop();
            },
            intake);
  }

  public static Command intakePiece(Intake intake) {
    return Commands.run(
            () -> {
              intake.setPosition(IntakeConstants.dropDown);
              intake.runWheel();
            },
            intake)
        .withTimeout(0.25);
  }

  public static Command gotPiece(Intake intake, Lightstrip lightstrip) {

    return Commands.run(
            () -> {
              intake.setPosition(IntakeConstants.dropDown);
              intake.runWheel();
              Logger.recordOutput("Intake/intakingPiece", true);
            },
            intake)
        // .withTimeout(3.0)
        .until(() -> (intake.hasPiece() && intake.atPosition(IntakeConstants.dropDown)))
        .andThen(
            Commands.runOnce(
                () ->
                    lightstrip.tempColor(
                        LightstripConstants.successSignal, LightstripConstants.Ranges.full),
                lightstrip))
        .andThen(() -> Logger.recordOutput("Intake/intakingPiece", false));
  }

  public static Command passPieceIntake(Intake intake, Pivot pivot, Indexer indexer) {
    return Commands.run(
            () -> {
              intake.setPosition(IntakeConstants.handoff);
              intake.runWheelHalf();
              // intake.holdPiece();
              // pivot.setPosition(PivotConstants.handoff);
              Logger.recordOutput("Intake/intaking", true);
            },
            intake,
            indexer)
        .until(
            () ->
                // (pivot.atPosition(PivotConstants.handoff)
                intake.atPosition(IntakeConstants.handoff))
        .andThen(() -> intake.holdPiece());
    // .withTimeout(5.0);
  }

  public static Command pivotHandoff(Pivot pivot) {
    return Commands.run(
            () -> {
              pivot.setPosition(PivotConstants.handoff);
            },
            pivot)
        .until(() -> (pivot.atPosition(PivotConstants.handoff)));
    // .withTimeout(5.0);
  }

  public static Command transferPiece(Intake intake, Indexer indexer, Pivot pivot) {
    return Commands.run(
            () -> {
              intake.outtakeWheel();
              indexer.runVolts(IndexerConstants.outtakeVolts);
              pivot.setPosition(PivotConstants.handoff);
            },
            intake,
            indexer,
            pivot)
        .until(() -> indexer.hasPiece());
    // .withTimeout(0.75)
    // .andThen(
    //     () -> {
    //       indexer.runVolts(IndexerConstants.holdVolts);
    //     },
    //     indexer,
    //     intake);
  }

  public static Command transferPiecept2(Intake intake, Indexer indexer, Pivot pivot) {
    return Commands.run(
            () -> {
              intake.outtakeWheel();
              indexer.runVolts(IndexerConstants.slowVolts);
              pivot.setPosition(PivotConstants.handoff);
            },
            indexer,
            intake,
            pivot)
        .until(() -> indexer.superHasPiece())
        .andThen(
            () -> {
              intake.stopWheels();
              indexer.runVolts(IndexerConstants.holdVolts);
            });
  }

  public static Command bringOutPiece(Indexer indexer) {
    return Commands.run(
            () -> {
              indexer.runVolts(-0.15);
            },
            indexer)
        .withTimeout(0.10)
        .andThen(() -> indexer.runVolts(IndexerConstants.holdVolts), indexer);
  }

  public static Command intakeHandoff(
      Intake intake, Indexer indexer, Pivot pivot, Lightstrip lightstrip) {
    // return intakePiece(intake)
    //     .andThen(gotPiece(intake))
    //     .andThen(passPieceIntake(intake, pivot, indexer));
    // return gotPiece(intake);
    return gotPiece(intake, lightstrip).andThen(passPieceIntake(intake, pivot, indexer));
    // .andThen(pivotHandoff(pivot))
    // .andThen(transferPiece(intake, indexer, pivot))
    // .andThen(bringOutPiece(indexer));
  }

  public static Command indexSequence(Intake intake, Indexer indexer, Pivot pivot) {
    return transferPiece(intake, indexer, pivot).andThen(transferPiecept2(intake, indexer, pivot));
  }

  public static Command ampPrep(Intake intake, Indexer indexer, Pivot pivot) {
    return pivotHandoff(pivot).andThen(transferPiece(intake, indexer, pivot));
    // .andThen(
    //     Commands.run(
    //         () -> {
    //           lightstrip.toggleOffColor(
    //               LightstripConstants.intake, LightstripConstants.Ranges.full);
    //         },
    //         lightstrip));
  }

  public static Command deepen(Intake intake) {
    return Commands.run(
            () -> {
              intake.runWheel();
            },
            intake)
        .withTimeout(1.5)
        .andThen(
            () -> {
              intake.holdPiece();
            },
            intake);
  }

  public static Command spitpt2(Intake intake) {
    return Commands.run(
            () -> {
              intake.outtakeWheel();
              intake.setPosition(IntakeConstants.handoff);
            },
            intake)
        .until(() -> intake.atPosition(IntakeConstants.handoff))
        .andThen(() -> intake.stopWheels(), intake);
  }

  public static Command spitpt1(Intake intake) {
    return Commands.run(
            () -> {
              intake.setPosition(IntakeConstants.dropDown);
              intake.runWheelVolts(1.0);
            },
            intake)
        .until(() -> intake.atPosition(IntakeConstants.dropDown));
  }

  public static Command spit(Intake intake) {
    return spitpt1(intake).andThen(spitpt2(intake));
  }
}
