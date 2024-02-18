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

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or done of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ShooterConstants {
    public static final double[] RPMEquation = {350.0, 500.000}; // need to tune; [0] -> m, [1] -> b
    public static final double[] timeEquation = {0.05, 0.15}; // need to tune; [0] -> m, [1] -> b
    public static final double kAccelCompFactor = 0.000;
    public static final double tolerance = 25.00;
  }

  public static final class PivotConstants {
    public static final double[] angleEquation = {Math.PI / 27.00, -Math.PI * 1.000 / 3.000};

    // public static InterpolatingDoubleTreeMap pivotAngleMap = new InterpolatingDoubleTreeMap({});

    public static final double mm_cruisevel = 0.4;
    public static final double mm_accel = 0.8;
    public static final double mm_jerk = 1.6;

    public static final double handoff = -0.95;

    public static final double tolerance = 0.075;
  }

  public static final class IntakeConstants {
    public static final double dropDown = 3.0;
    public static final double home = Math.PI / 2;
    public static final double handoff = 0.0;

    public static final double intakeVolts = -7.0;
    public static final double holdVolts = -0.5;
    public static final double outtakeVolts = 4.0;

    public static final double tolerance = 0.05;

    public static final double tofTolerance = 30.00;
  }

  public static final class IndexerConstants {
    public static final double indexVolts = 6.0;
    public static final double outtakeVolts = 7.0;
    public static final double holdVolts = 0.0;
  }

  public static final class FieldConstants {
    public static final Translation2d BLUE_SPEAKER_POSE = new Translation2d(-0.086473, 5.757474);
    public static final Translation2d RED_SPEAKER_POSE = new Translation2d(16.389722, 5.757474);

    public static Translation2d getSpeaker() {
      if (DriverStation.getAlliance().isPresent()) {
        return DriverStation.getAlliance().get() == Alliance.Red
            ? RED_SPEAKER_POSE
            : BLUE_SPEAKER_POSE;
      } else {
        return BLUE_SPEAKER_POSE; // default to blue
      }
    }
  }

  public static final class VisualizerConstants {
    public static final double intakeMountAngle = 90.0;

    public static final double pivotMountAngle = 90.0;

    public static final double shooterAngleOffset = 180.0;
    public static final double shooterMultiplier = -1.0;

    public static final double scale = 1.0 / 29.0;
  }

  public static final class VisionConstants {

    // Fix these constants
    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(
            new Translation3d(-0.06, 0.2, -0.2127),
            new Rotation3d(0.0, degreesToRadians(15.0), degreesToRadians(-3.0)));
  }
}
