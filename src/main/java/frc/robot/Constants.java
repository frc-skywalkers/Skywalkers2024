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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.lightstrip.LedState;
import frc.robot.lightstrip.Range;
import frc.robot.lightstrip.TempLedState;

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

  public static final Boolean isRed = false;

  public static final class ShooterConstants {
    public static final double[] RPMEquation = {350.0, 500.000}; // need to tune; [0] -> m, [1] -> b
    public static final double[] timeEquation = {0.05, 0.15}; // need to tune; [0] -> m, [1] -> b
    public static final double kAccelCompFactor = 0.000;
    public static final double tolerance = 70.0;
    public static InterpolatingDoubleTreeMap RPMAngleMap = new InterpolatingDoubleTreeMap();
  }

  public static final class PivotConstants {
    public static final double[] angleEquation = {Math.PI / 27.00, -Math.PI * 1.000 / 3.000};

    public static InterpolatingDoubleTreeMap pivotAngleMap = new InterpolatingDoubleTreeMap();

    public static final double mm_cruisevel = 0.4;
    public static final double mm_accel = mm_cruisevel * 10.0;
    public static final double mm_jerk = mm_accel * 10.0;

    public static final double handoff = -1.1;

    public static final double tolerance = 0.075;
  }

  public static final class IntakeConstants {
    public static final double dropDown = 2.95;
    public static final double home = Math.PI / 2;
    public static final double handoff = 0.0;

    public static final double intakeVolts = -6.5;
    public static final double holdVolts = -0.25;
    public static final double outtakeVolts = 5.0;

    public static final double tolerance = 0.05;
    public static final double downtolerance = 0.1;

    public static final double tofTolerance = 30.00;

    public static final double mm_cruisevel = 3.25;
    public static final double mm_accel = mm_cruisevel * 4.5;
    public static final double mm_jerk = mm_accel * 4.0;
  }

  public static final class IndexerConstants {
    public static final double indexVolts = -3.50;
    public static final double outtakeVolts = -9.0;
    public static final double holdVolts = 0.125;
    public static final double slowVolts = -1.0;
  }

  public static final class FieldConstants {
    public static final Translation2d BLUE_SPEAKER_POSE = new Translation2d(-0.086473, 5.757474);
    public static final Translation2d RED_SPEAKER_POSE = new Translation2d(16.389722, 5.757474);
    public static final double robotSubwooferSpeakerDistance = 57.341456693; // in inches

    public static final Pose2d RED_AMP_POSE =
        new Pose2d(14.483249, 7.57, new Rotation2d(1.5 * Math.PI));
    public static final Pose2d BLUE_AMP_POSE =
        new Pose2d(1.82, 7.57, new Rotation2d(0.5 * Math.PI));

    public static Translation2d getSpeaker() {
      // if (DriverStation.getAlliance().isPresent()) {
      //   return DriverStation.getAlliance().get() == Alliance.Red
      //       ? RED_SPEAKER_POSE
      //       : BLUE_SPEAKER_POSE;
      // } else {
      //   return BLUE_SPEAKER_POSE; // default to blue
      // }
      if (isRed) return RED_SPEAKER_POSE;
      else return BLUE_SPEAKER_POSE;
    }

    public static Pose2d getAmp() {
      // if (DriverStation.getAlliance().isPresent()) {
      //   return DriverStation.getAlliance().get() == Alliance.Red ? RED_AMP_POSE : BLUE_AMP_POSE;
      // } else {
      //   return BLUE_AMP_POSE;
      // }
      if (isRed) return RED_AMP_POSE;
      else return BLUE_AMP_POSE;
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

    public static int redprioritytag = 7;
    public static int blueprioritytag = 8;

    public static final Transform3d CAMERA_TO_ROBOT2 =
        new Transform3d(
            new Translation3d(-0.300, 0 - 0.301, -0.234),
            new Rotation3d(0.0, degreesToRadians(-30), degreesToRadians(-15.0)));
    public static final Transform3d CAMERA_TO_ROBOT1 =
        new Transform3d(
            new Translation3d(-0.300, -0.301, -0.234),
            new Rotation3d(0.0, degreesToRadians(30.0), degreesToRadians(15.0)));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.4, 0.4, 0.4);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.4);

    public static final class CameraInformation {
      public final String name;
      public final Transform3d cameraPose;

      public CameraInformation(String name, Transform3d cameraPose) {
        this.name = name;
        this.cameraPose = cameraPose;
      }
    }

    public static final CameraInformation kArducam1Info =
        new CameraInformation("arducam1", CAMERA_TO_ROBOT1.inverse());
    public static final CameraInformation kArducam2Info =
        new CameraInformation("arducam1", CAMERA_TO_ROBOT2.inverse());

    public static final class CameraResult {
      public final Pose2d estimatedPose;
      public final Matrix<N3, N1> stdDevs;
      public final double timestamp;

      public CameraResult(Pose2d estPose, Matrix<N3, N1> stdDevs, double timestamp) {
        this.estimatedPose = estPose;
        this.stdDevs = stdDevs;
        this.timestamp = timestamp;
      }
    }
    /*
    public static final Transform3d CAMERA_TO_ROBOT2 =
      new Transform3d(
          new Translation3d(-0.205, -0.249, -0.229),
          new Rotation3d(0.0, degreesToRadians(-30), degreesToRadians(-60.0)));
    public static final Transform3d CAMERA_TO_ROBOT2 =
      new Transform3d(
          new Translation3d(-0.205, -0.249, -0.229),
          new Rotation3d(0.0, degreesToRadians(30.0), degreesToRadians(60.0)));
     */
  }

  public static final class LightstripConstants {
    public static int pwmPort = 9;
    public static int ledCount = 150;

    public static final class Ranges {
      public static Range intake = new Range(0, 19);
      public static Range drivetrain = new Range(20, 99);
      public static Range superstructure = new Range(100, 149);
      public static Range shooter = new Range(150, 199);
      public static Range full = new Range(0, ledCount);
    }

    public static LedState defaultState = new LedState(50, 0, 0, "Fade");
    public static LedState holdingState = new LedState(0, 0, 55, "Fade");
    public static TempLedState successSignal = new TempLedState(0, 55, 0, "Fast Blink", 1);
    public static TempLedState intake = new TempLedState(255, 255, 0, "Solid", 10);
  }
}
