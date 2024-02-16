package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
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
      new LoggedTunableNumber("Vision/thetaStdDevCoefficient", 0.075);
  private static LoggedTunableNumber xyStdDevCoefficient =
      new LoggedTunableNumber("Vision/xyStdDevCoefficient", 0.075);

  private static LoggedTunableNumber maxSingleTargetAmbiguity =
      new LoggedTunableNumber("Vision/MaxSingleTargetAmbiguity", 0.08);

  private static LoggedTunableNumber maxValidDistanceAwayFromCurrentEstimateMeters =
      new LoggedTunableNumber("Vision/MaxValidDistanceFromCurrentEstimateMeters", 30.0);

  private ArrayList<VisionModule> visionModules = new ArrayList<VisionModule>();

  private AprilTagFieldLayout aprilTagLayout;

  private boolean useVisionForPoseEstimation = true;
  private boolean useMaxDistanceAwayFromExistingEstimate = true;

  private HashMap<Integer, Double> lastTagDetectionTimes = new HashMap<Integer, Double>();
  private ArrayList<Integer> tagsUsedInPoseEstimation = new ArrayList<Integer>();

  private List<Pose3d> posesFedToPoseEstimator3D = new ArrayList<>();
  private List<Pose2d> posesFedToPoseEstimator2D = new ArrayList<>();

  private List<TimestampedVisionUpdate> allTimestampedVisionUpdates =
      new ArrayList<TimestampedVisionUpdate>();

  private final Consumer<List<TimestampedVisionUpdate>> visionEstimatesConsumer;
  private final Supplier<Pose2d> poseEstimatorPoseSupplier;

  private MedianFilter distance = new MedianFilter(10);
  private MedianFilter ambiguity = new MedianFilter(20);

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

    aprilTagLayout.getTags().forEach((AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, -1.0));

    Pose3d[] allTagsArray = new Pose3d[aprilTagLayout.getTags().size()];
    var allTagsList = aprilTagLayout.getTags();
    for (int i = 0; i < aprilTagLayout.getTags().size(); i++) {
      allTagsArray[i] = allTagsList.get(i).pose;
    }
    Logger.recordOutput("Vision/AllAprilTags3D", allTagsArray);
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming("Vision")) {
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

      // activate alerts if camera is not connected
      for (VisionModule module : visionModules) {
        module.missingCameraAlert.set(module.visionIOInputs.connected);
      }

      // log all camera poses
      for (VisionModule visionModule : visionModules) {
        var robotPose = new Pose3d(poseEstimatorPoseSupplier.get());
        var camPose = robotPose.transformBy(visionModule.RobotToCamera);

        Logger.recordOutput("Vision/VisionModules/" + visionModule.name + "/CameraPose", camPose);
      }

      // logging all visible tags
      List<Pose3d> allSeenTags = new ArrayList<>();
      for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
        if (detectionEntry.getValue() == Timer.getFPGATimestamp()) {
          var tagPose = aprilTagLayout.getTagPose(detectionEntry.getKey());
          allSeenTags.add(tagPose.get());
        }
      }
      Logger.recordOutput(
          "Vision/visibleTags/rawAllVisibleTags",
          allSeenTags.toArray(new Pose3d[allSeenTags.size()]));

      // logging tags actually used in pose estimation
      Pose3d[] tagsUsedInPoseEstimationPoses = new Pose3d[tagsUsedInPoseEstimation.size()];
      for (int i = 0; i < tagsUsedInPoseEstimation.size(); i++) {
        tagsUsedInPoseEstimationPoses[i] =
            aprilTagLayout.getTagPose(tagsUsedInPoseEstimation.get(i)).get();
      }
      Logger.recordOutput(
          "Vision/visibleTags/tagsActuallyUsedInPoseEstimation", tagsUsedInPoseEstimationPoses);

      Logger.recordOutput(
          "Vision/posesFedToPoseEstimator3D",
          posesFedToPoseEstimator3D.toArray(new Pose3d[posesFedToPoseEstimator3D.size()]));
      Logger.recordOutput(
          "Vision/posesFedToPoseEstimator2D",
          posesFedToPoseEstimator2D.toArray(new Pose2d[posesFedToPoseEstimator2D.size()]));

      tagsUsedInPoseEstimation.clear();
      posesFedToPoseEstimator3D.clear();
      posesFedToPoseEstimator2D.clear();

      Logger.recordOutput("Vision/useVision", useVisionForPoseEstimation);
      Logger.recordOutput(
          "Vision/useMaxDistanceAwayFromExistingEstimate", useMaxDistanceAwayFromExistingEstimate);
    }
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

    averageDistanceFromTags = totalDistance / (double) numTargetsSeen;

    // Calibration DEBUG
    if (true) {

      // CAVEAT: only works if you do one camera at a time

      PhotonTrackedTarget target = cameraResult.getTargets().get(0);
      double x =
          (target.getDetectedCorners().get(0).x + target.getDetectedCorners().get(2).x) / 2.0;
      double y =
          (target.getDetectedCorners().get(0).y + target.getDetectedCorners().get(2).y) / 2.0;

      x = x - (1280.0 / 2.0);
      y = y - (720.0 / 2.0);

      Logger.recordOutput("Vision/Calibrate/" + visionModule.name + "/pixel_x", x);
      Logger.recordOutput("Vision/Calibrate/" + visionModule.name + "/pixel_y", y);
      Logger.recordOutput(
          "Vision/Calibrate/" + visionModule.name + "/ambiguity",
          ambiguity.calculate(target.getPoseAmbiguity()));
      Logger.recordOutput(
          "Vision/Calibrate/" + visionModule.name + "/dist",
          distance.calculate(averageDistanceFromTags));
    }

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
    cleanTargets.forEach(
        (PhotonTrackedTarget tag) -> tagsUsedInPoseEstimation.add(tag.getFiducialId()));

    VisionResultStatus status;
    if (numTargetsSeen == 1) {
      status = VisionResultStatus.SUCCESSFUL_SINGLE_TAG;
    } else {
      status =
          multiTagFailed
              ? VisionResultStatus.SUCCESSFUL_SINGLE_TAG_BECAUSE_MULTI_TAG_FALLBACK
              : VisionResultStatus.SUCCESSFUL_MULTI_TAG;
    }

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

    String ROOT_TABLE_PATH = "Vision/VisionModules/" + visionModule.name + "/";
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

    var currentStatus = fieldsToLog.status();
    boolean addedVisionEstimateToPoseEstimator =
        (currentStatus == VisionResultStatus.SUCCESSFUL_MULTI_TAG
                || currentStatus == VisionResultStatus.SUCCESSFUL_SINGLE_TAG
                || currentStatus
                    == VisionResultStatus.SUCCESSFUL_SINGLE_TAG_BECAUSE_MULTI_TAG_FALLBACK)
            ? true
            : false;
    Logger.recordOutput(ROOT_TABLE_PATH + "SUCCESSFUL_RESULT?", addedVisionEstimateToPoseEstimator);
  }
}
