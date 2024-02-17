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
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;

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

  public static Command shoot(Indexer indexer) {
    return Commands.run(
            () -> {
              indexer.runVolts(IndexerConstants.outtakeVolts);
            },
            indexer)
        .withTimeout(1.0)
        .andThen(() -> indexer.stop());
  }
}
