package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.team6328.Alert;
import frc.robot.subsystems.vision.VisionIO.Inputs;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionModule {
  final VisionIO visionIO;
  final Inputs visionIOInputs;
  final PhotonPoseEstimator photonPoseEstimator;
  final Transform3d RobotToCamera;
  final String name;

  public VisionResultLoggedFields loggedFields;
  public double lastSuccessfullyProcessedResultTimeStampCTRETime;
  public VisionResultStatus lastStatus;

  public final Alert missingCameraAlert;

  public VisionModule(VisionModuleConfiguration config, AprilTagFieldLayout aprilTagFieldLayout) {
    this.visionIO = config.visionIO;
    this.visionIOInputs = new Inputs();
    this.RobotToCamera = config.robotToCamera;
    this.name = config.logName;

    this.loggedFields = VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.INIT);
    this.lastSuccessfullyProcessedResultTimeStampCTRETime = -1.0;

    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            visionIO.getCamera(),
            RobotToCamera);
    // might as well do lowest ambiguity since we this will likely only happen when only 1 target is
    // seen
    // no point filtering for the best target when there is only 1.
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);

    this.missingCameraAlert = new Alert("Missing Camera: " + this.name, Alert.AlertType.ERROR);
  }
}
