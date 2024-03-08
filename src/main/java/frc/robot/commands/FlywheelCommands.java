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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import org.littletonrobotics.junction.Logger;

public class FlywheelCommands {

  private static final PIDController alignThetaController = new PIDController(2.0, 0, 0.3);

  private FlywheelCommands() {}

  /** Rev's up shooter depending on distance from goal using a linear regression */
  static Translation2d goalTranslation2d = new Translation2d(0.000, 0.000);

  public static double calcShootRPM(Translation2d curPose, Translation2d goalPose) {
    double shootDistance = curPose.getDistance(goalPose);
    double shootRPM =
        ShooterConstants.RPMEquation[0] * shootDistance + ShooterConstants.RPMEquation[1];

    return shootRPM;
  }

  public static double calcShootTime(Translation2d curPose, Translation2d goalPose) {
    double shootDistance = curPose.getDistance(goalPose);
    double shotTime =
        ShooterConstants.timeEquation[0] * shootDistance + ShooterConstants.timeEquation[1];
    return shotTime;
  }

  public static double calcPivotPos(Translation2d curPose, Translation2d goalPose) {
    double shootDistance = curPose.getDistance(goalPose);
    return PivotConstants.angleEquation[0] * shootDistance + PivotConstants.angleEquation[1];
  }

  public static Command autoShoot(Flywheel shooter, Drive drive) {
    return Commands.run(
        () -> {
          // first we figure out distance necessary to shoot
          Pose2d curPose = drive.getPose();
          Translation2d curTranslation = curPose.getTranslation(); // Translation2d
          Translation2d goalPos;
          if (drive.getShootMode()) {
            goalPos = drive.calcVirtualGoal();
          } else {
            goalPos = FieldConstants.getSpeaker();
          }
          // curTranslation = curPose.getTranslation();
          // Translation2d curTranslation = new Translation2d(5.000, 0.000);
          // double shootDistance = curTranslation.getDistance(goalTranslation2d);
          // double shootRPM =
          // ShooterConstants.RPMEquation[0] * shootDistance + ShooterConstants.RPMEquation[1];
          shooter.runVelocity(calcShootRPM(curTranslation, goalPos));
        },
        shooter);
  }

  public static Command autoPivotAim(Pivot pivot, Drive drive) {
    return Commands.run(
        () -> {
          Pose2d curPose = drive.getPose();
          Translation2d curTranslation = curPose.getTranslation(); // Translation2d
          Translation2d goalPos;
          if (drive.getShootMode()) {
            goalPos = drive.calcVirtualGoal();
          } else {
            goalPos = FieldConstants.getSpeaker();
          }
          pivot.setPosition(calcPivotPos(curTranslation, goalPos));
        },
        pivot);
  }

  public static Command waiting(Pivot pivot, Flywheel flywheel, Intake intake) {
    return Commands.waitUntil(
        () ->
            pivot.atPosition(PivotConstants.handoff)
                && flywheel.atDesiredRPM(5000)
                && intake.atPosition(IntakeConstants.handoff));
  }

  public static Command outtake(Indexer indexer, Intake intake) {
    return Commands.run(
            () -> {
              indexer.runVolts(IndexerConstants.outtakeVolts);
              intake.outtakeWheel();
              Logger.recordOutput("Indexer/outtaking", true);
            },
            indexer)
        .withTimeout(0.75)
        .andThen(
            () -> {
              indexer.stop();
              intake.stopWheels();
              Logger.recordOutput("Indexer/outtaking", false);
            },
            indexer,
            intake);
  }

  public static Command shoot(Pivot pivot, Flywheel flywheel, Indexer indexer, Intake intake) {
    return waiting(pivot, flywheel, intake).andThen(outtake(indexer, intake));
    // return outtake(indexer, intake);
  }

  public static Command aimAmp(Pivot pivot) {
    return Commands.run(
            () -> {
              pivot.setPosition(0.8);
            },
            pivot)
        .until(() -> pivot.atPosition(0.8));
  }

  public static Command outtakeAmp(Indexer indexer, Flywheel flywheel, Pivot pivot) {
    return Commands.run(
            () -> {
              indexer.runVolts(IndexerConstants.outtakeVolts);
              flywheel.runVelocity(1500);
              pivot.setPosition(0.9);
            },
            indexer,
            flywheel,
            pivot)
        .withTimeout(1.1)
        .andThen(() -> indexer.stop(), indexer);
  }

  public static Command prepSubwoofer(Flywheel flywheel, Pivot pivot) {
    return Commands.run(
            () -> {
              Logger.recordOutput("Pivot/aiming", true);
              pivot.setPosition(PivotConstants.handoff);
              flywheel.runVelocity(5000);
            },
            flywheel,
            pivot)
        .handleInterrupt(
            () -> {
              Logger.recordOutput("Pivot/aiming", false);
            });
  }

  public static Command lower(Pivot pivot) {
    return Commands.run(
        () -> {
          pivot.setPosition(-1.5);
        },
        pivot);
  }

  // public static Command subwooferShot(Flywheel flywheel, Pivot pivot, Indexer indexer, Intake
  // intake) {
  //   return Commands.run(
  //     () -> {

  //     },
  //     flywheel, pivot, indexer, intake
  //   );
  // }
}
