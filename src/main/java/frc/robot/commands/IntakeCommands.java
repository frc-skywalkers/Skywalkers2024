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
import frc.robot.subsystems.flywheel.Flywheel;
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
              intake.stopWheels();
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
              intake.runWheelDouble();
              Logger.recordOutput("Intake/intakingPiece", true);
            },
            intake)
        // .withTimeout(3.0)
        .until(() -> (intake.hasPiece() && intake.atDropDown()))
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
              intake.runWheelDouble();
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
        .andThen(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("bro guys whats going on", true);
                  intake.runWheelDouble();
                },
                intake));
    // .withTimeout(5.0);
  }

  public static Command pivotHandoff(
      Pivot pivot, Intake intake, Flywheel flywheel, Indexer indexer) {
    return Commands.run(
            () -> {
              pivot.setPosition(PivotConstants.handoff);
              intake.setPosition(IntakeConstants.handoff);
              indexer.stop();
              flywheel.runVelocity(5000.00);
              // flywheel.stop();
            },
            pivot,
            flywheel)
        .until(
            () ->
                (pivot.atPosition(PivotConstants.handoff)
                    && intake.atPosition(IntakeConstants.handoff)));
    // .withTimeout(5.0);
  }

  public static Command transferPiece(
      Intake intake, Indexer indexer, Pivot pivot, Lightstrip lightstrip, Flywheel flywheel) {
    return Commands.run(
            () -> {
              intake.outtakeWheel();
              indexer.runVolts(IndexerConstants.indexVolts);
              pivot.setPosition(PivotConstants.handoff);
              // flywheel.stop();
              flywheel.runVelocity(5000.00);
            },
            intake,
            indexer,
            pivot,
            flywheel)
        .until(() -> indexer.hasPiece());
  }

  public static Command feeding(Intake intake, Indexer indexer, Pivot pivot, Flywheel flywheel) {
    return Commands.run(
            () -> {
              intake.setPosition(1.0);
              indexer.runVolts(-IndexerConstants.indexVolts);
              flywheel.runVelocity(-1750.000);
              pivot.setPosition(-1.00);
            },
            intake,
            indexer,
            pivot,
            flywheel)
        .until(() -> indexer.hasPiece() && !indexer.superHasPiece())
        .andThen(
            Commands.runOnce(
                () -> {
                  indexer.holdVolt();
                }));

    // .withTimeout(2.5);

    // .andThen(
    //     () -> {
    //       indexer.runVolts(IndexerConstants.holdVolts);
    //     },
    //     indexer,
    //     intake);
  }

  public static Command transferPiece3(Intake intake, Indexer indexer, Pivot pivot) {
    return Commands.run(
            () -> {
              intake.runWheel();
              indexer.runVolts(-IndexerConstants.indexVolts);
              pivot.setPosition(PivotConstants.handoff);
            },
            intake,
            indexer,
            pivot)
        .until(() -> !indexer.hasPiece());
  }

  public static Command transferPiecept2(
      Intake intake, Indexer indexer, Pivot pivot, Flywheel flywheel) {
    return Commands.run(
            () -> {
              // intake.runWheelHalf();
              indexer.runVolts(IndexerConstants.slowVolts);
              // pivot.setPosition(PivotConstants.handoff);
              // if (intake.tofHasPiece()) {
              intake.outtakeWheel();
              // } else {
              // intake.runWheelHalf();
              // }
              pivot.setPosition(-1.4);
              // flywheel.stop();
              flywheel.runVelocity(5000.00);

              // intake.stopWheels();

            },
            indexer,
            intake,
            pivot,
            flywheel)
        .until(() -> indexer.superHasPiece())
        // .withTimeout(0.75)
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

  public static Command indexSequence(
      Intake intake, Indexer indexer, Pivot pivot, Lightstrip lightstrip, Flywheel flywheel) {
    return pivotHandoff(pivot, intake, flywheel, indexer)
        .andThen(transferPiece(intake, indexer, pivot, lightstrip, flywheel))
        // .andThen(transferPiece3(intake, indexer, pivot))
        // .andThen(transferPiece(intake, indexer, pivot, lightstrip))
        .andThen(transferPiecept2(intake, indexer, pivot, flywheel));
  }

  public static Command ampPrep(
      Intake intake, Indexer indexer, Pivot pivot, Lightstrip lightstrip, Flywheel flywheel) {
    return pivotHandoff(pivot, intake, flywheel, indexer)
        .andThen(transferPiece(intake, indexer, pivot, lightstrip, flywheel));
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
        .until(() -> intake.atDropDown());
  }

  public static Command spit(Intake intake) {
    return spitpt1(intake).andThen(spitpt2(intake));
  }

  public static Command resetIntake(Intake intake) {
    return Commands.run(
            () -> {
              intake.setPosition(IntakeConstants.handoff);
              intake.stopWheels();
            },
            intake)
        .until(() -> intake.atPosition(IntakeConstants.handoff));
  }
}
