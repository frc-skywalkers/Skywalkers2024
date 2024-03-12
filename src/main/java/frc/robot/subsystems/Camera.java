package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraResult;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Camera {

  private PhotonCamera camera;
  // private PhotonCameraSim cameraSim;
  private PhotonPoseEstimator poseEstimator;

  private double lastEstTimestamp = 0;

  public Camera(String name, Transform3d cameraTransform) {
    camera = new PhotonCamera(name);

    /*
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
    cameraProp.setCalibError(0.1, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);

    cameraSim = new PhotonCameraSim(camera, cameraProp);
    */

    // try {
    poseEstimator =
        new PhotonPoseEstimator(
            // new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/fields/BlueTags.json"),
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            cameraTransform);
    // } catch (IOException e) {
    //   e.printStackTrace();
    // }
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    /*
    if (RobotBase.isSimulation()) {
      cameraSim.enableDrawWireframe(true);
    } else {
      cameraSim.enableDrawWireframe(false);
    }
    */
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = poseEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = camera.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public void setAprilTagField(AprilTagFieldLayout field) {
    poseEstimator.setFieldTags(field);
  }

  public CameraResult getCameraResult() {
    try {
      var estimate = getEstimatedGlobalPose().get();
      var estPose = estimate.estimatedPose.toPose2d();
      Logger.recordOutput("camresult", estPose.getX());
      Logger.recordOutput("camresultSTDDEV", getEstimationStdDevs(estPose).max());
      return new CameraResult(estPose, getEstimationStdDevs(estPose), estimate.timestampSeconds);
    } catch (Exception e) {
      return new CameraResult(new Pose2d(), VecBuilder.fill(0, 0, 0), 0);
    }
  }

  /*
  public PhotonCameraSim getCameraSim() {
    return cameraSim;
  }
  */
}
