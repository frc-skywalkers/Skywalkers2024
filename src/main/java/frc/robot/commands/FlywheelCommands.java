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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IndexerConstants;
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

  static double[] pos;

  public static double calcShootRPM(Translation2d curPose, Translation2d goalPose) {
    double shootDistance = curPose.getDistance(goalPose);
    // shootDistance = pos[0];
    double shootRPM = ShooterConstants.RPMAngleMap.get(shootDistance - 0.02);

    return shootRPM;
  }

  public static double calcShootTime(Translation2d curPose, Translation2d goalPose) {
    double shootDistance = curPose.getDistance(goalPose);
    shootDistance = Units.metersToInches(shootDistance);
    double shotTime =
        ShooterConstants.timeEquation[0] * shootDistance + ShooterConstants.timeEquation[1];
    return shotTime;
  }

  public static double calcPivotPos(Translation2d curPose, Translation2d goalPose) {
    double shootDistance = curPose.getDistance(goalPose);
    // shootDistance = pos[0];
    // shootDistance = Units.metersToInches(shootDistance);

    return PivotConstants.pivotAngleMap.get(shootDistance - 0.02);
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
          double rpm = calcShootRPM(curTranslation, goalPos);
          shooter.updateAutoShoot(rpm);
          shooter.runVelocity(rpm);
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
          double ang = calcPivotPos(curTranslation, goalPos);
          pivot.setPosition(ang);
          pivot.updateAutoPivot(ang);
        },
        pivot);
  }

  public static Command waiting(Pivot pivot, Flywheel flywheel) {
    return Commands.waitUntil(() -> pivot.atPosition(-1.05) && flywheel.atDesiredRPM(5000));
  }

  public static Command superWaiting(Pivot pivot, Flywheel flywheel) {
    return Commands.waitUntil(() -> pivot.pivotAligned() && flywheel.shootRevved());
  }

  public static Command waiting(Pivot pivot, Flywheel flywheel, Drive drive) {
    return Commands.waitUntil(
        () ->
            pivot.atPosition()
                && flywheel.atDesiredRPM()
                && drive.isAligned(FieldConstants.getSpeaker()));
  }

  public static Command outtake(Indexer indexer) {
    return Commands.run(
            () -> {
              indexer.runVolts(IndexerConstants.outtakeVolts);
              Logger.recordOutput("Indexer/outtaking", true);
              // intake.setPosition(1.0);
            },
            indexer)
        .until(() -> !indexer.hasPiece())
        .andThen(
            () -> {
              indexer.stop();
              Logger.recordOutput("Indexer/outtaking", false);
            },
            indexer);
  }

  public static Command shoot(Pivot pivot, Flywheel flywheel, Indexer indexer) {
    return waiting(pivot, flywheel).andThen(outtake(indexer));
    // return outtake(indexer, intake);
  }

  public static Command superShoot(Pivot pivot, Flywheel flywheel, Indexer indexer) {
    return superWaiting(pivot, flywheel).andThen(outtake(indexer));
    // return outtake(indexer, intake);
  }

  public static Command shoot(Pivot pivot, Flywheel flywheel, Indexer indexer, Drive drive) {
    return waiting(pivot, flywheel, drive).andThen(outtake(indexer));
  }

  public static Command deepenIndexer(Indexer indexer) {
    return Commands.run(() -> indexer.runVolts(-4.0), indexer).withTimeout(0.3);
  }

  public static Command aimAmp(Pivot pivot, Flywheel flywheel) {
    return Commands.run(
            () -> {
              pivot.setPosition(0.25);
              flywheel.runVelocity(3500);
            },
            pivot)
        .until(() -> pivot.atPosition(0.25));
  }

  public static Command outtakeAmp(Indexer indexer, Flywheel flywheel, Pivot pivot) {
    return Commands.run(
            () -> {
              indexer.runVolts(-7.0);
              flywheel.runVelocity(3500);
              pivot.setPosition(0.25);
            },
            indexer,
            flywheel,
            pivot)
        .until(() -> (!indexer.hasPiece()))
        .andThen(() -> indexer.stop(), indexer);
  }

  public static Command prepSubwoofer(
      Flywheel flywheel, Pivot pivot, Indexer indexer, Intake intake) {
    return Commands.waitUntil(() -> indexer.hasPiece())
        .andThen(
            Commands.run(
                () -> {
                  Logger.recordOutput("Pivot/aiming", true);
                  pivot.setPosition(-1.05);
                  flywheel.runVelocity(5000);
                  intake.setPosition(1.0);
                },
                flywheel,
                pivot,
                intake))
        .handleInterrupt(
            () -> {
              Logger.recordOutput("Pivot/aiming", false);
            });
  }

  public static Command rev(Flywheel flywheel, Indexer indexer) {
    return Commands.waitUntil(() -> indexer.hasPiece())
        .andThen(
            Commands.run(
                () -> {
                  flywheel.runVelocity(5000);
                }));
  }

  public static Command stupidShooter(Flywheel flywheel, Pivot pivot) {
    return Commands.run(
            () -> {
              flywheel.runVelocity(0.00);
              pivot.setPosition(-1.500);
            },
            flywheel,
            pivot)
        .withTimeout(0.75);
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
