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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;

public class FlywheelCommands {
  private FlywheelCommands() {}

  /** Rev's up shooter depending on distance from goal using a linear regression */
  static Translation2d goalTranslation2d = new Translation2d(0.000, 0.000);

  public static Command autoShoot(Flywheel shooter, Drive drive) {
    return Commands.run(
        () -> {
          // first we figure out distance necessary to shoot
          Pose2d curPose = drive.getPose();
          Translation2d curTranslation = curPose.getTranslation(); // Translation2d
          // curTranslation = curPose.getTranslation();
          // Translation2d curTranslation = new Translation2d(5.000, 0.000);
          double shootDistance = curTranslation.getDistance(goalTranslation2d);
          double shootRPM =
              ShooterConstants.RPMEquation[0] * shootDistance + ShooterConstants.RPMEquation[1];
          shooter.runVelocity(shootRPM);
        },
        shooter);
  }
}
