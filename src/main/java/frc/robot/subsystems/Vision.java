// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.VisionConstants.CameraInformation;
import frc.robot.Constants.VisionConstants.CameraResult;
import java.io.IOException;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Vision {

  private Camera arducam1;
  // private Camera arducam2;

  // private VisionSystemSim visionSim;

  public Vision(CameraInformation arducam1info) {

    arducam1 = new Camera(arducam1info.name, arducam1info.cameraPose);
    // arducam2 = new Camera(arducam2info.name, arducam2info.cameraPose);

    /*
    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      try {
        visionSim.addAprilTags(
            new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/fields/BlueTags.json"));
      } catch (IOException e) {
      }

      visionSim.addCamera(
          arducam1.getCameraSim(), VisionConstants.CAMERA_TO_ROBOT1.inverse()); // camera pose
      visionSim.addCamera(arducam2.getCameraSim(), VisionConstants.CAMERA_TO_ROBOT2.inverse());
    }
    */
  }

  public CameraResult getCam1Result() {
    return arducam1.getCameraResult();
  }

  /*public CameraResult getCam2Result() {
    return arducam2.getCameraResult();
  }*/

  /*
  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }
  */

  public void setFieldTags(Alliance alliance) throws IOException {

    String resource =
        (alliance == Alliance.Blue) ? "/fields/BlueTags.json" : "/fields/RedTags.json";

    AprilTagFieldLayout field = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + resource);

    /*
    if (RobotBase.isSimulation()) {
      visionSim.clearAprilTags();
      visionSim.addAprilTags(field);
    }
    */

    arducam1.setAprilTagField(field);
    // arducam2.setAprilTagField(field);

    ArrayList<Pose3d> tagPoses = new ArrayList<Pose3d>();

    for (int i = 0; i < field.getTags().size(); i++) {
      tagPoses.add(field.getTags().get(i).pose);
    }

    Logger.recordOutput("Vision/Tag Poses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
  }

  /*
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }


  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
  */
}
