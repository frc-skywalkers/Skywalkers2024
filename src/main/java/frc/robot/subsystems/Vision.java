// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  private final PhotonCamera photonCamera1;
  private final PhotonCamera photonCamera2;
  private final Drive swerveSubsystem;
  private final AprilTagFieldLayout aprilTagFieldLayout;

  // Standard Deviations for (Module) States and Visioon measurements' trust
  // [x, y, theta], with units in meters and radians

  /*
  private static final Vector<N3> stateStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0.1)); // 5
  private static final Vector<N3> visionMeasurementStdDevs =
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(1000000)); // 10
  public final SwerveDrivePoseEstimator poseEstimator;
  */
  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  /** Creates a new Vision. */
  public Vision(PhotonCamera photonCamera1, PhotonCamera photonCamera2, Drive swerveSubsystem) {

    this.photonCamera1 = photonCamera1;
    this.photonCamera2 = photonCamera2;
    this.swerveSubsystem = swerveSubsystem;

    AprilTagFieldLayout layout;

    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      Logger.recordOutput("Odometry/Alliance", alliance.isPresent());
      if (alliance.isPresent()) {
        layout.setOrigin(
            alliance.get() == Alliance.Blue
                ? OriginPosition.kBlueAllianceWallRightSide
                : OriginPosition.kRedAllianceWallRightSide);
      } else {
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        Logger.recordOutput("Odometry/bad things have happened", true);
      }
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;

    /*
    poseEstimator =
        new SwerveDrivePoseEstimator(
            swerveSubsystem.kinematics,
            swerveSubsystem.getRotation(),
            swerveSubsystem.getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs);
    */
  }

  @Override
  public void periodic() {
    if (Constants.currentMode == Mode.SIM) {
      return;
    }
    if (Constants.currentMode == Mode.REAL) {
      return;
    }
    // return;
    // This method will be called once per scheduler run

    photonCamera1.getPipelineIndex();

    // Update pose estimator with the best visible target
    var pipelineResult1 = photonCamera1.getLatestResult();
    var pipelineResult2 = photonCamera2.getLatestResult();

    var resultTimestamp1 = pipelineResult1.getTimestampSeconds();
    var resultTimestamp2 = pipelineResult2.getTimestampSeconds();

    Logger.recordOutput("Odometry/resultTimestamp1", resultTimestamp1);
    Logger.recordOutput("Odometry/hasTargets", pipelineResult2.hasTargets());

    /*
    if (resultTimestamp1 != previousPipelineTimestamp && pipelineResult1.hasTargets()) {

      previousPipelineTimestamp = resultTimestamp1;
      var target1 = pipelineResult1.getBestTarget();
      var fiducialId1 = target1.getFiducialId();

      // Get the tag pose from field layout - consider that the layout will be null if it failed to
      // load
      Optional<Pose3d> tagPose1 =
          aprilTagFieldLayout == null
              ? Optional.empty()
              : aprilTagFieldLayout.getTagPose(fiducialId1);

      if (target1.getPoseAmbiguity() <= .2 && fiducialId1 >= 0 && tagPose1.isPresent()) {
        var targetPose1 = tagPose1.get();
        Transform3d camToTarget1 = target1.getBestCameraToTarget();
        Pose3d camPose1 = targetPose1.transformBy(camToTarget1.inverse());

        var visionMeasurement1 = camPose1.transformBy(VisionConstants.CAMERA_TO_ROBOT1);
        poseEstimator.addVisionMeasurement(visionMeasurement1.toPose2d(), resultTimestamp1);
        SmartDashboard.putNumber("Vision1X", visionMeasurement1.toPose2d().getX());
        SmartDashboard.putNumber("Vision1Y", visionMeasurement1.toPose2d().getY());
        SmartDashboard.putNumber(
            "Vision1Rotation", visionMeasurement1.toPose2d().getRotation().getDegrees());
      }
    }
    */

    if (resultTimestamp1 != previousPipelineTimestamp && pipelineResult1.hasTargets()) {

      previousPipelineTimestamp = resultTimestamp1;
      List<PhotonTrackedTarget> targets1 = pipelineResult1.getTargets();
      SmartDashboard.putNumber("targetnum1", targets1.size());

      for (int i = 0; i < targets1.size(); i++) {
        var currenttarget1 = targets1.get(i);
        var fiducialId1 = currenttarget1.getFiducialId();

        Optional<Pose3d> tagPose1 =
            aprilTagFieldLayout == null
                ? Optional.empty()
                : aprilTagFieldLayout.getTagPose(fiducialId1);

        if (currenttarget1.getPoseAmbiguity() <= .2 && fiducialId1 >= 0 && tagPose1.isPresent()) {
          var targetPose1 = tagPose1.get();
          Transform3d camToTarget1 = currenttarget1.getBestCameraToTarget();
          Pose3d camPose1 = targetPose1.transformBy(camToTarget1.inverse());

          var visionMeasurement1 =
              camPose1.transformBy(VisionConstants.CAMERA_TO_ROBOT1).toPose2d();
          Pose2d measurement1 =
              new Pose2d(
                  new Translation2d(visionMeasurement1.getX(), visionMeasurement1.getY()),
                  swerveSubsystem.getRotation());

          swerveSubsystem.poseEstimator.addVisionMeasurement(measurement1, resultTimestamp1);
        }
      }
    }
    if (resultTimestamp2 != previousPipelineTimestamp && pipelineResult2.hasTargets()) {

      previousPipelineTimestamp = resultTimestamp2;
      List<PhotonTrackedTarget> targets2 = pipelineResult2.getTargets();
      SmartDashboard.putNumber("targetnum2", targets2.size());

      for (int i = 0; i < targets2.size(); i++) {
        var currenttarget2 = targets2.get(i);
        var fiducialId2 = currenttarget2.getFiducialId();

        Optional<Pose3d> tagPose2 =
            aprilTagFieldLayout == null
                ? Optional.empty()
                : aprilTagFieldLayout.getTagPose(fiducialId2);

        if (currenttarget2.getPoseAmbiguity() <= .2 && fiducialId2 >= 0 && tagPose2.isPresent()) {
          var targetPose2 = tagPose2.get();
          Transform3d camToTarget2 = currenttarget2.getBestCameraToTarget();
          Pose3d camPose2 = targetPose2.transformBy(camToTarget2.inverse());

          var visionMeasurement2 =
              camPose2.transformBy(VisionConstants.CAMERA_TO_ROBOT1).toPose2d();
          Pose2d measurement2 =
              new Pose2d(
                  new Translation2d(visionMeasurement2.getX(), visionMeasurement2.getY()),
                  swerveSubsystem.getRotation());

          swerveSubsystem.poseEstimator.addVisionMeasurement(measurement2, resultTimestamp2);
        }
      }

      /*
      if (resultTimestamp2 != previousPipelineTimestamp && pipelineResult2.hasTargets()) {
        var target2 = pipelineResult2.getBestTarget();
        var fiducialId2 = target2.getFiducialId();

        // Get the tag pose from field layout - consider that the layout will be null if it failed to
        // load
        Optional<Pose3d> tagPose2 =
            aprilTagFieldLayout == null
                ? Optional.empty()
                : aprilTagFieldLayout.getTagPose(fiducialId2);
        Logger.recordOutput("Odometry/pose ambiguity", target2.getPoseAmbiguity());
        Logger.recordOutput("Odometry/tag present", tagPose2.isPresent());
        Logger.recordOutput("Odometry/id", fiducialId2);

        if (target2.getPoseAmbiguity() <= .2 && fiducialId2 >= 0 && tagPose2.isPresent()) {
          var targetPose2 = tagPose2.get();
          Transform3d camToTarget2 = target2.getBestCameraToTarget();
          Pose3d camPose2 = targetPose2.transformBy(camToTarget2.inverse());

          var visionMeasurement2 = camPose2.transformBy(VisionConstants.CAMERA_TO_ROBOT2);
          poseEstimator.addVisionMeasurement(visionMeasurement2.toPose2d(), resultTimestamp2);
          SmartDashboard.putNumber("Vision2X", visionMeasurement2.toPose2d().getX());
          SmartDashboard.putNumber("Vision2Y", visionMeasurement2.toPose2d().getY());
          SmartDashboard.putNumber(
              "Vision2Rotation", visionMeasurement2.toPose2d().getRotation().getDegrees());
        }
        */
    }

    // Update pose estimator with drivetrain sensors
    /*
    poseEstimator.update(swerveSubsystem.getRotation(), swerveSubsystem.getModulePositions());

    SmartDashboard.putNumber("estimatedX", getCurrentPose().getX());
    SmartDashboard.putNumber("estimatedY", getCurrentPose().getY());
    SmartDashboard.putNumber("estimatedR", getCurrentPose().getRotation().getDegrees());

    field2d.setRobotPose(getCurrentPose());
    Logger.recordOutput("Odometry/Estimated Pose", getCurrentPose());
    */
  }

  /*
  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d pose) {
    poseEstimator.resetPosition(
        swerveSubsystem.getRotation(), swerveSubsystem.getModulePositions(), pose);
  }
  */
}
