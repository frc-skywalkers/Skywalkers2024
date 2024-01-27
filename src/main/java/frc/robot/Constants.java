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

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or done of its inner classes) wherever the
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
    public static final double[] RPMEquation = {350.0, 500.000}; // need to tune; [0] -> m, [1] -> b
    public static final double[] timeEquation = {0.1, 0.15}; // need to tune; [0] -> m, [1] -> b
    public static final double kAccelCompFactor = 0.000;
  }

  public static final class PivotConstants {
    public static final double[] angleEquation = {Math.PI / 27.00, -Math.PI * 1.000 / 3.000};
    public static final double mm_cruisevel = 1.5;
    public static final double mm_accel = 3;
    public static final double mm_jerk = 6;
  }
}
