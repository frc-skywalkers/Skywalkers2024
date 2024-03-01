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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier slowMode,
      DoubleSupplier autoAlign) {
    return Commands.run(
        () -> {
          double scle = 0.95;

          if (slowMode.getAsDouble() > 0.5) {
            scle = 0.7;
          }
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble() * scle, ySupplier.getAsDouble() * scle),
                  DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND) * 0.75;
          //   double omega = 0;
          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega) * drive.getMaxAngularSpeedRadPerSec();
          if (autoAlign.getAsDouble() > 0.5) {
            // omega = drive.getAlignOutput(drive.get)
            omega = drive.getAlignOutput();
          }
          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          Logger.recordOutput("Swerve/linearVelocity", linearVelocity);

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega,
                  drive.getRotation()));
        },
        drive);
  }

  public static Command autoAlignAmp(Drive drive) {
    ProfiledPIDController headingController =
        new ProfiledPIDController(
            3.0,
            0.0,
            0.05,
            new Constraints(
                drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec()));
    ProfiledPIDController xController =
        new ProfiledPIDController(5.0, 0.0, 0.1, new Constraints(3.0, 3.0));
    ProfiledPIDController yController =
        new ProfiledPIDController(5.0, 0.0, 0.1, new Constraints(3.0, 3.0));
    headingController.reset(
        drive.getPose().getRotation().getRadians(), drive.getFieldRelativeSpeed().omega);
    xController.reset(drive.getPose().getX(), drive.getFieldRelativeSpeed().vx);
    yController.reset(drive.getPose().getY(), drive.getFieldRelativeSpeed().vy);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    Pose2d goal = FieldConstants.getAmp();

    return Commands.run(
            () -> {
              double omegaOutput =
                  headingController.calculate(
                      drive.getPose().getRotation().getRadians(), goal.getRotation().getRadians());
              double xOutput = xController.calculate(drive.getPose().getX(), goal.getX());
              double yOutput = yController.calculate(drive.getPose().getY(), goal.getY());
              double omegaSetpoint = headingController.getSetpoint().velocity;
              double xSetpoint = xController.getSetpoint().velocity;
              double ySetpoint = yController.getSetpoint().velocity;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      xOutput + xSetpoint,
                      yOutput + ySetpoint,
                      omegaOutput + omegaSetpoint,
                      drive.getRotation()));
            },
            drive)
        .until(() -> drive.atGoal(goal));
  }
}
