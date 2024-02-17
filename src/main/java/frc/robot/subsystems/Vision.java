// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {

  private final PhotonCamera photonCamera1;
  private final PhotonCamera photonCamera2;
  private final Drive swerveSubsystem;
  private final AprilTagFieldLayout aprilTagFieldLayout;

  // Standard Deviations for (Module) States and Visioon measurements' trust
  // [x, y, theta], with units in meters and radians
  private static final Vector<N3> stateStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private static final Vector<N3> visionMeasurementStdDevs =
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
  private final SwerveDrivePoseEstimator poseEstimator;
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
      layout.setOrigin(
          alliance.get() == Alliance.Blue
              ? OriginPosition.kBlueAllianceWallRightSide
              : OriginPosition.kRedAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            swerveSubsystem.kinematics,
            swerveSubsystem.getRotation(),
            swerveSubsystem.getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update pose estimator with the best visible target
    var pipelineResult1 = photonCamera1.getLatestResult();
    var pipelineResult2 = photonCamera2.getLatestResult();

    var resultTimestamp1 = pipelineResult1.getTimestampSeconds();
    var resultTimestamp2 = pipelineResult2.getTimestampSeconds();

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

        var visionMeasurement1 = camPose1.transformBy(VisionConstants.CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement1.toPose2d(), resultTimestamp1);
        SmartDashboard.putNumber("Vision1X", visionMeasurement1.toPose2d().getX());
        SmartDashboard.putNumber("Vision1Y", visionMeasurement1.toPose2d().getY());
        SmartDashboard.putNumber(
            "Vision1Rotation", visionMeasurement1.toPose2d().getRotation().getDegrees());
      }
    }

    if (resultTimestamp2 != previousPipelineTimestamp && pipelineResult2.hasTargets()) {

      previousPipelineTimestamp = resultTimestamp2;
      var target2 = pipelineResult2.getBestTarget();
      var fiducialId2 = target2.getFiducialId();

      // Get the tag pose from field layout - consider that the layout will be null if it failed to
      // load
      Optional<Pose3d> tagPose2 =
          aprilTagFieldLayout == null
              ? Optional.empty()
              : aprilTagFieldLayout.getTagPose(fiducialId2);

      if (target2.getPoseAmbiguity() <= .2 && fiducialId2 >= 0 && tagPose2.isPresent()) {
        var targetPose2 = tagPose2.get();
        Transform3d camToTarget2 = target2.getBestCameraToTarget();
        Pose3d camPose2 = targetPose2.transformBy(camToTarget2.inverse());

        var visionMeasurement2 = camPose2.transformBy(VisionConstants.CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement2.toPose2d(), resultTimestamp2);
        SmartDashboard.putNumber("Vision2X", visionMeasurement2.toPose2d().getX());
        SmartDashboard.putNumber("Vision2Y", visionMeasurement2.toPose2d().getY());
        SmartDashboard.putNumber(
            "Vision2Rotation", visionMeasurement2.toPose2d().getRotation().getDegrees());
      }
    }

    // Update pose estimator with drivetrain sensors
    poseEstimator.update(swerveSubsystem.getRotation(), swerveSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }
}
