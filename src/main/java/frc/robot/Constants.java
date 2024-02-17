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
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ShooterConstants {
    public static final double[] RPMEquation = {100.0, 500.000}; // need to tune; [0] -> m, [1] -> b
  }

  public static final class LimelightConstants {

    // public static int RTheight; //meters
    public static int cameraheight; // meters

    public static double kPx;
    public static double kIx;
    public static double kDx;
    public static double kPy;
    public static double kIy;
    public static double kDy;
    public static double kPr;
    public static double kIr;
    public static double kDr;

    public static double xtolerance;
    public static double ytolerance;
    public static double rtolerance;

    public static int xclamp;
    public static int yclamp;

    public static int rclamp;

    public static double limelightOffsetCenter;
  }

  public static final class VisionConstants {

    // Fix these constants
    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(
            new Translation3d(-0.06, 0.2, -0.2127),
            new Rotation3d(0.0, degreesToRadians(15.0), degreesToRadians(-3.0)));
  }
}
