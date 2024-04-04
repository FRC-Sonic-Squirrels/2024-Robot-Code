package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team6328.Alert;
import frc.robot.subsystems.vision.VisionIO.Inputs;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

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

  public final LoggerEntry.ByteArray log_photonPacketBytes;
  public final LoggerEntry.Decimal log_lastTimestampCTRETime;
  public final LoggerEntry.Bool log_connected;
  public final LoggerEntry.Decimal log_medianLatency;
  public final LoggerEntry.Decimal log_medianUpdateTime;

  public final LoggerEntry.Decimal log_lastTimestampCTRETimeSuccessful;
  public final LoggerEntry.Text log_status;
  public final LoggerEntry.Struct<Pose3d> log_cameraPose;
  public final LoggerEntry.Struct<Pose3d> log_calculatedRobotPose3d;
  public final LoggerEntry.Struct<Pose2d> log_calculatedRobotPose2d;
  public final LoggerEntry.Integer log_numSeenTargets;
  public final LoggerEntry.Decimal log_distanceFromExistingPoseEstimate;
  public final LoggerEntry.Decimal log_averageDistanceFromTags;
  public final LoggerEntry.Decimal log_tagAmbiguity;
  public final LoggerEntry.Decimal log_xyStandardDeviation;
  public final LoggerEntry.Decimal log_thetaStandardDeviation;
  public final LoggerEntry.Bool log_successfulResult;
  public final LoggerEntry.BoolArray log_seenTargetsIDs;

  public VisionModule(
      LoggerGroup logGroup,
      VisionModuleConfiguration config,
      AprilTagFieldLayout aprilTagFieldLayout) {
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

    logGroup = logGroup.subgroup(name);

    log_photonPacketBytes = logGroup.buildBytes("photonPacketBytes");
    log_lastTimestampCTRETime = logGroup.buildDecimal("lastTimestampCTRETime");
    log_connected = logGroup.buildBoolean("connected");
    log_medianLatency = logGroup.buildDecimal("medianLatency");
    log_medianUpdateTime = logGroup.buildDecimal("medianUpdateTime");

    log_lastTimestampCTRETimeSuccessful = logGroup.buildDecimal("lastTimestampCTRETimeSuccessful");
    log_status = logGroup.buildString("*STATUS");
    log_cameraPose = logGroup.buildStruct(Pose3d.class, "CameraPose");
    log_calculatedRobotPose3d = logGroup.buildStruct(Pose3d.class, "calculatedRobotPose3d");
    log_calculatedRobotPose2d = logGroup.buildStruct(Pose2d.class, "calculatedRobotPose2d");
    log_numSeenTargets = logGroup.buildInteger("numSeenTargets");
    log_distanceFromExistingPoseEstimate =
        logGroup.buildDecimal("distanceFromExistingPoseEstimate");
    log_averageDistanceFromTags = logGroup.buildDecimal("averageDistanceFromTags");
    log_tagAmbiguity = logGroup.buildDecimal("tagAmbiguity");
    log_xyStandardDeviation = logGroup.buildDecimal("xyStandardDeviation");
    log_thetaStandardDeviation = logGroup.buildDecimal("thetaStandardDeviation");
    log_successfulResult = logGroup.buildBoolean("SUCCESSFUL_RESULT?");
    log_seenTargetsIDs = logGroup.buildBooleanArray("seenTargetsIDs");
  }

  public void log(Pose3d robotPose) {
    byte[] photonPacketBytes = new byte[visionIOInputs.lastResult.getPacketSize()];
    PhotonPipelineResult.serde.pack(new Packet(photonPacketBytes), visionIOInputs.lastResult);
    log_photonPacketBytes.info(photonPacketBytes);
    log_lastTimestampCTRETime.info(visionIOInputs.lastTimestampCTRETime);
    log_connected.info(visionIOInputs.connected);
    log_medianLatency.info(visionIOInputs.medianLatency);
    log_medianUpdateTime.info(visionIOInputs.medianUpdateTime);
    var fieldsToLog = loggedFields;

    log_lastTimestampCTRETimeSuccessful.info(lastSuccessfullyProcessedResultTimeStampCTRETime);

    var status = fieldsToLog.status();
    if (status == lastStatus && !status.success) {
      // skip logging details of unsuccessful results that repeat
      return;
    }
    lastStatus = status;

    log_status.info(status.name());
    log_cameraPose.info(robotPose.transformBy(RobotToCamera));
    log_calculatedRobotPose3d.info(fieldsToLog.robotPose3d());
    log_calculatedRobotPose2d.info(fieldsToLog.robotPose3d().toPose2d());
    log_numSeenTargets.info(fieldsToLog.numSeenTargets());
    log_distanceFromExistingPoseEstimate.info(fieldsToLog.distanceFromExistingPoseEstimate());
    log_averageDistanceFromTags.info(fieldsToLog.averageDistanceFromTags());
    log_tagAmbiguity.info(fieldsToLog.tagAmbiguity());
    log_xyStandardDeviation.info(fieldsToLog.xyStandardDeviation());
    log_thetaStandardDeviation.info(fieldsToLog.thetaStandardDeviation());
    log_successfulResult.info(status.success);
  }
}
