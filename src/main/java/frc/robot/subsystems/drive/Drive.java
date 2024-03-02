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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.FieldRelativeAccel;
import frc.robot.util.FieldRelativeSpeed;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(29.0);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(29.0);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  private static final double ALIGN_MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();

  private static final Vector<N3> stateStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private static final Vector<N3> visionMeasurementStdDevs =
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
  public final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();

  public boolean ampAligned = false;
  public boolean subwooferAligned = false;

  ProfiledPIDController headingController =
      new ProfiledPIDController(
          3.0, 0.0, 0.05, new Constraints(ALIGN_MAX_ANGULAR_SPEED, ALIGN_MAX_ANGULAR_SPEED / 2));

  private Pose2d virtGoal = new Pose2d();
  private boolean shootOnMove = false;
  private double alignTolerance = 0.05;
  private double transTolerance = 0.15;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () -> Constants.isRed, // Alliance.Red
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    poseEstimator =
        new SwerveDrivePoseEstimator(
            this.kinematics,
            this.getRotation(),
            this.getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs);
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    int deltaCount =
        gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE;
    for (int i = 0; i < 4; i++) {
      deltaCount = Math.min(deltaCount, modules[i].getPositionDeltas().length);
    }
    for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
      // Read wheel deltas from each module
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelDeltas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
      }

      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      var twist = kinematics.toTwist2d(wheelDeltas);
      if (gyroInputs.connected) {
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        Rotation2d gyroRotation = gyroInputs.odometryYawPositions[deltaIndex];
        twist = new Twist2d(twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians());
        lastGyroRotation = gyroRotation;
      }
      // Apply the twist (change since last sample) to the current pose
      pose = pose.exp(twist);
    }
    m_fieldRelVel = new FieldRelativeSpeed(getChassisSpeed(), getRotation());
    m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, 0.020);
    m_lastFieldRelVel = m_fieldRelVel;
    // calcVirtualGoal();
    // Logger.recordOutput("Shooting/VirtualGoal", virtGoal);
    // Logger.recordOutput("Swerve/velx", m_fieldRelVel.vx);
    // Logger.recordOutput("Swerve/vely", m_fieldRelVel.vy);

    poseEstimator.update(this.getRotation(), this.getModulePositions());

    SmartDashboard.putNumber("estimatedX", getCurrentPose().getX());
    SmartDashboard.putNumber("estimatedY", getCurrentPose().getY());
    SmartDashboard.putNumber("estimatedR", getCurrentPose().getRotation().getDegrees());

    field2d.setRobotPose(getCurrentPose());
    Logger.recordOutput("Odometry/Estimated Pose", getCurrentPose());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d pose) {
    poseEstimator.resetPosition(this.getRotation(), this.getModulePositions(), pose);
  }

  private ChassisSpeeds getChassisSpeed() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void setAllianceColor() {}

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);

    // SmartDashboard.putNumber("odometryx", getPose().getX());
    // SmartDashboard.putNumber("odometryy", getPose().getY());
    // SmartDashboard.putNumber("odometryR", getPose().getRotation().getDegrees());
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  public void updateVirtualGoal(Translation2d virtualGoal) {
    virtGoal = new Pose2d(virtualGoal, new Rotation2d());
  }

  public static double calcShootTime(Translation2d curPose, Translation2d goalPose) {
    double shootDistance = curPose.getDistance(goalPose);
    double shotTime =
        ShooterConstants.timeEquation[0] * shootDistance + ShooterConstants.timeEquation[1];
    return shotTime;
  }

  public Translation2d calcVirtualGoal() {
    Pose2d curPose = getPose();
    Translation2d curTranslation = curPose.getTranslation();

    double a = curTranslation.getX();
    double d = curTranslation.getY();

    double b = FieldConstants.getSpeaker().getX();
    double e = FieldConstants.getSpeaker().getY();

    double c = m_fieldRelVel.vx + m_fieldRelAccel.ax * ShooterConstants.kAccelCompFactor;
    double f = m_fieldRelVel.vy + m_fieldRelAccel.ay * ShooterConstants.kAccelCompFactor;

    double l = 0.000, r = 2.000;
    double shotTime = 1.000;
    for (int iter = 0; iter < 40; iter++) {
      double mid = (l + r) / 2.0000;
      shotTime = mid;
      Translation2d virtGoal = new Translation2d(b - c * shotTime, e - f * shotTime);
      double actShotTime = calcShootTime(curTranslation, virtGoal);
      if (shotTime > actShotTime) r = mid;
      else l = mid;
    }

    updateVirtualGoal(new Translation2d(b - c * shotTime, e - f * shotTime));
    return virtGoal.getTranslation();
  }
  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns module positions for all modules */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public FieldRelativeSpeed getFieldRelativeSpeed() {
    return m_fieldRelVel;
  }

  public FieldRelativeAccel getFieldRelativeAccel() {
    return m_fieldRelAccel;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose; // wo vision
    // return getCurrentPose(); // vision measurement
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public boolean getShootMode() {
    return shootOnMove;
  }

  public void setShootMode(boolean desiredShootMode) {
    shootOnMove = desiredShootMode;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  public void resetHeadingController() {
    headingController.reset(getRotation().getRadians(), getChassisSpeed().omegaRadiansPerSecond);
  }

  private double getAngleDif(Translation2d goalPos) {
    double goalAngle =
        Math.atan2(goalPos.getY() - getPose().getY(), goalPos.getX() - getPose().getX());
    return goalAngle;
  }

  public double getAlignOutput() {
    Translation2d goalPos;
    if (getShootMode()) {
      goalPos = virtGoal.getTranslation();
    } else goalPos = FieldConstants.getSpeaker();
    Pose2d curPose = getPose();
    double goalAngle = getAngleDif(goalPos);
    double omega =
        headingController.calculate(curPose.getRotation().getRadians(), goalAngle)
            + headingController.getSetpoint().velocity;

    Logger.recordOutput("Swerve/goalAngle", goalAngle);
    Logger.recordOutput("Swerve/actAngle", getPose().getRotation().getRadians());

    return omega;
  }

  public boolean isAligned(Translation2d goalPos) {
    double angDif = getAngleDif(goalPos);
    // Rotation2d aDif;
    // return Math.abs(angDif - getRotation().getRadians()) < alignTolerance;
    return atAngle(angDif);
  }

  // figure out better way to do this, but this works for now
  public boolean atAngle(double rad) {
    double dif = rad - getPose().getRotation().getRadians();
    if (Math.abs(dif + 2 * Math.PI) < alignTolerance) return true;
    if (Math.abs(dif) < alignTolerance) return true;
    if (Math.abs(dif - 2 * Math.PI) < alignTolerance) return true;
    return false;
  }

  public boolean atGoal(Pose2d goalPos) {
    // boolean aligned = isAligned(goalPos);
    // boolean aligned = false;
    // double head = getPose().getRotation().getRadians();
    double xPos = getPose().getX();
    double yPos = getPose().getY();
    return atAngle(goalPos.getRotation().getRadians())
        && (Math.abs(yPos - goalPos.getY()) < transTolerance)
        && (Math.abs(xPos - goalPos.getX()) < transTolerance);
  }
}
