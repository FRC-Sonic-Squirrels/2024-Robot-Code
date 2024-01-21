package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.team6328.PoseEstimator;
import frc.lib.team6328.PoseEstimator.TimestampedVisionUpdate;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private static LoggedTunableNumber thetaStdDevCoefficient =
      new LoggedTunableNumber("vision/thetaStdDevCoefficient", 0.075);
  private static LoggedTunableNumber xyStdDevCoefficient =
      new LoggedTunableNumber("vision/xyStdDevCoefficient", 0.075);

  private static LoggedTunableNumber maxSingleTargetAmbiguity =
      new LoggedTunableNumber("Vision/MaxSingleTargetAmbiguity", 0.08);

  private static LoggedTunableNumber maxValidDistanceAwayFromCurrentEstimateMeters =
      new LoggedTunableNumber("Vision/MaxValidDistanceFromCurrentEstimateMeters", 3.0);

  private ArrayList<VisionModule> visionModules = new ArrayList<VisionModule>();

  private AprilTagFieldLayout aprilTagLayout;

  private boolean useVisionForPoseEstimation = true;
  private boolean useMaxDistanceAwayFromExistingEstimate = true;

  private HashMap<Integer, Double> lastTagDetectionTimes = new HashMap<Integer, Double>();

  private List<Pose3d> posesFedToPoseEstimator3D = new ArrayList<>();
  private List<Pose2d> posesFedToPoseEstimator2D = new ArrayList<>();

  private List<TimestampedVisionUpdate> allTimestampedVisionUpdates =
      new ArrayList<TimestampedVisionUpdate>();

  private final Consumer<List<TimestampedVisionUpdate>> visionEstimatesConsumer;
  private final Supplier<Pose2d> poseEstimatorPoseSupplier;

  public Vision(
      AprilTagFieldLayout aprilTagLayout,
      Supplier<Pose2d> poseEstimatorPoseSupplier,
      Consumer<List<TimestampedVisionUpdate>> visionEstimatesConsumer,
      VisionModuleConfiguration... visionModuleConfigs) {

    this.visionEstimatesConsumer = visionEstimatesConsumer;
    this.poseEstimatorPoseSupplier = poseEstimatorPoseSupplier;
    this.aprilTagLayout = aprilTagLayout;

    for (VisionModuleConfiguration config : visionModuleConfigs) {
      visionModules.add(new VisionModule(config, aprilTagLayout));
    }

    for (AprilTag tag : aprilTagLayout.getTags()) {
      Logger.recordOutput("Vision/AllAprilTags3D/" + tag.ID, tag.pose);
    }

    aprilTagLayout.getTags().forEach((AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, -1.0));
  }

  @Override
  public void periodic() {
    // update all inputs
    for (VisionModule module : visionModules) {
      module.visionIO.updateInputs(module.visionIOInputs);
      Logger.processInputs("Vision/" + module.name, module.visionIOInputs);
    }

    boolean processVision = true;
    if (!useVisionForPoseEstimation) {
      processVision = false;
      setAllVisionModuleUnsuccessfulStatus(VisionResultStatus.VISION_DISABLED);
    }

    // process vision
    if (processVision) {
      for (VisionModule visionModule : visionModules) {
        var fieldsToLog = processVision(visionModule);
        visionModule.loggedFields = fieldsToLog;
      }
    }

    // send successful results to pose estimator
    if (allTimestampedVisionUpdates.size() > 0) {
      visionEstimatesConsumer.accept(allTimestampedVisionUpdates);
      allTimestampedVisionUpdates.clear();
    }

    // log all vision module's logged fields
    for (VisionModule visionModule : visionModules) {
      logVisionModule(visionModule);
    }

    List<Pose3d> allSeenTags = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (detectionEntry.getValue() == Timer.getFPGATimestamp()) {
        var tagPose = aprilTagLayout.getTagPose(detectionEntry.getKey());
        allSeenTags.add(tagPose.get());
      }
    }

    // activate alerts if camera is not connected
    for (VisionModule module : visionModules) {
      module.missingCameraAlert.set(module.visionIOInputs.connected);
    }

    // log all camera poses
    for (VisionModule visionModule : visionModules) {
      var robotPose = new Pose3d(poseEstimatorPoseSupplier.get());
      var camPose = robotPose.transformBy(visionModule.RobotToCamera);

      Logger.recordOutput("Vision/" + visionModule.name + "/CameraPose", camPose);
    }

    Logger.recordOutput(
        "Vision/currentVisibleTags", allSeenTags.toArray(new Pose3d[allSeenTags.size()]));
    Logger.recordOutput(
        "Vision/posesFedToPoseEstimator3D",
        posesFedToPoseEstimator3D.toArray(new Pose3d[posesFedToPoseEstimator3D.size()]));
    Logger.recordOutput(
        "Vision/posesFedToPoseEstimator2D",
        posesFedToPoseEstimator2D.toArray(new Pose2d[posesFedToPoseEstimator2D.size()]));

    posesFedToPoseEstimator3D.clear();
    posesFedToPoseEstimator2D.clear();

    Logger.recordOutput("Vision/useVision", useVisionForPoseEstimation);
    Logger.recordOutput(
        "Vision/useMaxDistanceAwayFromExistingEstimate", useMaxDistanceAwayFromExistingEstimate);
  }

  public VisionResultLoggedFields processVision(VisionModule visionModule) {
    Pose2d prevEstimatedRobotPose = poseEstimatorPoseSupplier.get();

    PhotonPipelineResult cameraResult;
    double currentResultTimeStampCTRETime;

    Pose3d newCalculatedRobotPose;

    double xyStandardDeviation;
    double thetaStandardDeviation;
    double tagAmbiguity;
    double averageDistanceFromTags;

    boolean multiTagFailed = false;

    synchronized (visionModule.visionIOInputs) {
      cameraResult = visionModule.visionIOInputs.lastResult;
      currentResultTimeStampCTRETime = visionModule.visionIOInputs.lastTimestampCTRETime;
    }

    if (visionModule.lastSuccessfullyProcessedResultTimeStampCTRETime
        >= currentResultTimeStampCTRETime) {
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.NOT_A_NEW_RESULT);
    }

    visionModule.lastSuccessfullyProcessedResultTimeStampCTRETime = currentResultTimeStampCTRETime;

    // remove any tags that are not part of the field layout
    ArrayList<PhotonTrackedTarget> cleanTargets = new ArrayList<PhotonTrackedTarget>();
    for (PhotonTrackedTarget target : cameraResult.getTargets()) {
      if (aprilTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
        cleanTargets.add(target);
      }
    }

    cleanTargets.forEach(
        (PhotonTrackedTarget tag) ->
            lastTagDetectionTimes.put(tag.getFiducialId(), Timer.getFPGATimestamp()));

    var numTargetsSeen = cleanTargets.size();

    if (numTargetsSeen == 0) {
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.NO_TARGETS_VISIBLE);
    }

    if (numTargetsSeen == 1) {
      PhotonTrackedTarget singularTag = cameraResult.getTargets().get(0);

      if (!isValidTarget(singularTag)) {
        return (singularTag.getPoseAmbiguity() > maxSingleTargetAmbiguity.get())
            ? VisionResultLoggedFields.unsuccessfulResult(
                VisionResultStatus.INVALID_TAG_AMBIGUITY_TOO_HIGH)
            : VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.INVALID_TAG);
      }
    }

    Optional<EstimatedRobotPose> photonPoseEstimatorOptionalResult =
        visionModule.photonPoseEstimator.update(cameraResult);

    if (photonPoseEstimatorOptionalResult.isEmpty()) {
      return VisionResultLoggedFields.unsuccessfulResult(
          VisionResultStatus.PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY);
    }

    // used to log if the multi tag failed
    if (photonPoseEstimatorOptionalResult.get().strategy
        != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
      multiTagFailed = true;
    }

    newCalculatedRobotPose = photonPoseEstimatorOptionalResult.get().estimatedPose;

    tagAmbiguity = 0.0;
    double totalDistance = 0.0;
    for (PhotonTrackedTarget tag : cleanTargets) {
      totalDistance +=
          aprilTagLayout
              .getTagPose(tag.getFiducialId())
              .get()
              .getTranslation()
              .getDistance(newCalculatedRobotPose.getTranslation());
    }

    averageDistanceFromTags = totalDistance / (double) cleanTargets.size();

    var distanceFromExistingPoseEstimate =
        prevEstimatedRobotPose
            .getTranslation()
            .getDistance(
                new Translation2d(newCalculatedRobotPose.getX(), newCalculatedRobotPose.getY()));

    if (useMaxDistanceAwayFromExistingEstimate
        && (distanceFromExistingPoseEstimate
            > (maxValidDistanceAwayFromCurrentEstimateMeters.get() * numTargetsSeen))) {
      return VisionResultLoggedFields.unsuccessfulResult(
          VisionResultStatus.TOO_FAR_FROM_EXISTING_ESTIMATE);
    }

    xyStandardDeviation =
        (xyStdDevCoefficient.get() * Math.pow(averageDistanceFromTags, 2))
            / ((double) numTargetsSeen);
    thetaStandardDeviation =
        (thetaStdDevCoefficient.get() * Math.pow(averageDistanceFromTags, 2))
            / ((double) numTargetsSeen);

    var timestampedVisionUpdate =
        new PoseEstimator.TimestampedVisionUpdate(
            currentResultTimeStampCTRETime,
            newCalculatedRobotPose.toPose2d(),
            VecBuilder.fill(xyStandardDeviation, xyStandardDeviation, thetaStandardDeviation));

    allTimestampedVisionUpdates.add(timestampedVisionUpdate);

    posesFedToPoseEstimator3D.add(newCalculatedRobotPose);
    posesFedToPoseEstimator2D.add(newCalculatedRobotPose.toPose2d());

    var status =
        multiTagFailed
            ? VisionResultStatus.SUCCESSFUL_MULTI_TAG_FAILED
            : VisionResultStatus.SUCCESSFUL;

    return new VisionResultLoggedFields(
        status,
        numTargetsSeen,
        tagAmbiguity,
        averageDistanceFromTags,
        distanceFromExistingPoseEstimate,
        xyStandardDeviation,
        thetaStandardDeviation,
        newCalculatedRobotPose);
  }

  public boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() <= maxSingleTargetAmbiguity.get()
        && aprilTagLayout.getTagPose(target.getFiducialId()).isPresent();
  }

  public void useMaxDistanceAwayFromExistingEstimate(boolean value) {
    useMaxDistanceAwayFromExistingEstimate = value;
  }

  public void setAllVisionModuleUnsuccessfulStatus(VisionResultStatus status) {
    for (VisionModule cameraPackage : visionModules) {
      cameraPackage.loggedFields = VisionResultLoggedFields.unsuccessfulResult(status);
    }
  }

  private void logVisionModule(VisionModule visionModule) {

    String ROOT_TABLE_PATH = "Vision/" + visionModule.name + "/";
    var fieldsToLog = visionModule.loggedFields;

    Logger.recordOutput(
        ROOT_TABLE_PATH + "LastSuccessfullyProcessedResultTimeStampCTRETime",
        visionModule.lastSuccessfullyProcessedResultTimeStampCTRETime);

    Logger.recordOutput(
        ROOT_TABLE_PATH + "*STATUS",
        fieldsToLog.status().name() + ": " + fieldsToLog.status().additionalInfo);
    Logger.recordOutput(ROOT_TABLE_PATH + "calculatedRobotPose3d", fieldsToLog.robotPose3d());
    Logger.recordOutput(
        ROOT_TABLE_PATH + "calculatedRobotPose2d", fieldsToLog.robotPose3d().toPose2d());

    Logger.recordOutput(ROOT_TABLE_PATH + "numSeenTargets", fieldsToLog.numSeenTargets());
    Logger.recordOutput(
        ROOT_TABLE_PATH + "distanceFromExistingPoseEstimate",
        fieldsToLog.distanceFromExistingPoseEstimate());
    Logger.recordOutput(
        ROOT_TABLE_PATH + "averageDistanceFromTags", fieldsToLog.averageDistanceFromTags());
    Logger.recordOutput(ROOT_TABLE_PATH + "tagAmbiguity", fieldsToLog.tagAmbiguity());
    Logger.recordOutput(ROOT_TABLE_PATH + "xyStandardDeviation", fieldsToLog.xyStandardDeviation());
    Logger.recordOutput(
        ROOT_TABLE_PATH + "thetaStandardDeviation", fieldsToLog.thetaStandardDeviation());

    boolean addedVisionEstimateToPoseEstimator =
        (fieldsToLog.status() == VisionResultStatus.SUCCESSFUL) ? true : false;
    Logger.recordOutput(
        ROOT_TABLE_PATH + "addedVisionEstimateToPoseEstimator(AKA SUCCESSFUL?)",
        addedVisionEstimateToPoseEstimator);
  }
}
