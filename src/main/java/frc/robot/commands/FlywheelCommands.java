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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.FieldRelativeAccel;
import frc.robot.util.FieldRelativeSpeed;
import java.util.function.DoubleSupplier;
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

  public static Command autoShoot(Flywheel shooter, Drive drive, Pivot pivot) {
    return Commands.run(
        () -> {
          // first we figure out distance necessary to shoot
          Pose2d curPose = drive.getPose();
          Translation2d curTranslation = curPose.getTranslation(); // Translation2d
          // curTranslation = curPose.getTranslation();
          // Translation2d curTranslation = new Translation2d(5.000, 0.000);
          // double shootDistance = curTranslation.getDistance(goalTranslation2d);
          // double shootRPM =
          // ShooterConstants.RPMEquation[0] * shootDistance + ShooterConstants.RPMEquation[1];
          shooter.runVelocity(calcShootRPM(curTranslation, goalTranslation2d));
          pivot.setPosition(calcPivotPos(curTranslation, goalTranslation2d));
        },
        shooter,
        pivot);
  }

  static double DEADBAND = 0.1;

  public static double[] autoAlignDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double goalAngle) {
    double scle = 0.5;
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble() * scle, ySupplier.getAsDouble() * scle), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    // double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    // omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Convert to field relative speeds & send command
    double omega = alignThetaController.calculate(drive.getRotation().getRadians(), goalAngle);
    Logger.recordOutput("Shooting/omega", omega);
    // drive.runVelocity(
    //     ChassisSpeeds.fromFieldRelativeSpeeds(
    //         linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
    //         linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
    //         omega,
    //         // omega * drive.getMaxAngularSpeedRadPerSec(),
    //         drive.getRotation()));
    double[] ret = {
      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
      omega
    };
    return ret;
  }

  public static Command shootWhileMove(
      Flywheel shooter, Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    alignThetaController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
        () -> {
          FieldRelativeSpeed robotVel = drive.getFieldRelativeSpeed();
          FieldRelativeAccel robotAccel = drive.getFieldRelativeAccel();
          Pose2d curPose = drive.getPose();
          Translation2d curTranslation = curPose.getTranslation();

          double a = curTranslation.getX();
          double d = curTranslation.getY();

          double b = goalTranslation2d.getX();
          double e = goalTranslation2d.getY();

          double c = robotVel.vx + robotAccel.ax * ShooterConstants.kAccelCompFactor;
          double f = robotVel.vy + robotAccel.ay * ShooterConstants.kAccelCompFactor;

          double w = ShooterConstants.timeEquation[1];
          double x = ShooterConstants.timeEquation[0];

          // now we need to calculate virtual goal position

          // 10 iterations is a good balance between compute heavy + accurate?

          double l = 0.000, r = 2.000;
          double shotTime = 1.000;
          for (int iter = 0; iter < 20; iter++) {
            double mid = (l + r) / 2.0000;
            shotTime = mid;
            Translation2d virtGoal = new Translation2d(b - c * shotTime, e - f * shotTime);
            double actShotTime = calcShootTime(curTranslation, virtGoal);
            if (shotTime > actShotTime) r = mid;
            else l = mid;
          }

          // double y = a * a - 2 * a * b + b * b + d * d - 2 * d * e + e * e;
          // double z = 2 * a * c - 2 * b * c + 2 * d * f - 2 * e * f;
          // double v = c * c + f * f;

          // double num1 = x * Math.sqrt(4 * (v * w * w - v * x * x * y + w * z + y) + x * x * z *
          // z);
          // double num2 = -2 * w + x * x * (-z);
          // double den = 2 * (v * x * x - 1);
          // if (Math.abs(den) < 0.00001) den += 0.00001;

          // double shotTime = (num1 + num2) / den;

          Translation2d virtGoal = new Translation2d(b - c * shotTime, e - f * shotTime);

          drive.updateVirtualGoal(virtGoal);

          shooter.runVelocity(calcShootRPM(curTranslation, virtGoal));

          double ang = Math.atan2(virtGoal.getY() - d, virtGoal.getX() - a);
          Logger.recordOutput("Shooter/rotationAngle", ang);
          double[] inps = autoAlignDrive(drive, xSupplier, ySupplier, ang);
          Logger.recordOutput("Shooting/shotTime", shotTime);
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  inps[0],
                  inps[1],
                  inps[2],
                  // omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation()));
        },
        shooter,
        drive);
  }
}
