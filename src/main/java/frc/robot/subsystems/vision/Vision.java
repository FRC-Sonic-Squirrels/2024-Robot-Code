package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.team6328.PoseEstimator;
import frc.lib.team6328.PoseEstimator.TimestampedVisionUpdate;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  public static final String ROOT_TABLE = "Vision";

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  public static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.StructArray<Pose3d> logAllAprilTags3D =
      logGroup.buildStructArray(Pose3d.class, "AllAprilTags3D");
  private static final LoggerEntry.Bool logUseVision = logGroup.buildBoolean("useVision");
  private static final LoggerEntry.Bool logUseMaxDistanceAwayFromExistingEstimate =
      logGroup.buildBoolean("useMaxDistanceAwayFromExistingEstimate");

  private static final LoggerEntry.Integer logRejectByGyro = logGroup.buildInteger("RejectByGyro");
  private static final LoggerEntry.Decimal logRejectByGyroError =
      logGroup.buildDecimal("RejectByGyroError");
  private static final LoggerEntry.Integer logRejectByDistance =
      logGroup.buildInteger("RejectByDistance");
  private static final LoggerEntry.Decimal logRejectByDistanceError =
      logGroup.buildDecimal("RejectByDistanceError");
  public static final LoggerEntry.Integer logConfigTagAmount =
      logGroup.buildInteger("configTagAmount");

  private static final LoggerGroup logGroupVisibleTags = logGroup.subgroup("visibleTags");
  private static final LoggerEntry.StructArray<Pose3d> log_rawAllVisibleTags =
      logGroupVisibleTags.buildStructArray(Pose3d.class, "rawAllVisibleTags");
  private static final LoggerEntry.StructArray<Pose3d> log_tagsActuallyUsedInPoseEstimation =
      logGroupVisibleTags.buildStructArray(Pose3d.class, "tagsActuallyUsedInPoseEstimation");
  private static final LoggerEntry.StructArray<Pose3d> log_posesFedToPoseEstimator3D =
      logGroupVisibleTags.buildStructArray(Pose3d.class, "posesFedToPoseEstimator3D");
  private static final LoggerEntry.StructArray<Pose2d> log_posesFedToPoseEstimator2D =
      logGroupVisibleTags.buildStructArray(Pose2d.class, "posesFedToPoseEstimator2D");

  protected static final TunableNumberGroup group = new TunableNumberGroup("Vision");

  private static final LoggedTunableNumber thetaStdDevCoefficient =
      group.build("thetaStdDevCoefficient", 0.075);
  private static final LoggedTunableNumber xyStdDevCoefficient =
      group.build("xyStdDevCoefficient", 0.075);

  private static final LoggedTunableNumber maxSingleTargetAmbiguity =
      group.build("MaxSingleTargetAmbiguity", 0.08);

  private static final LoggedTunableNumber maxValidDistanceAwayFromCurrentEstimateMeters =
      group.build("MaxValidDistanceFromCurrentEstimateMeters", 30.0);

  private static final LoggedTunableNumber gyroFilteringToleranceDegrees =
      group.build("GyroFilteringToleranceDegrees", 20.0);

  private static final LoggedTunableNumber zHeightToleranceMeters =
      group.build("zHeightToleranceMeters", 0.5);

  private static final LoggedTunableNumber pitchAndRollToleranceDegrees =
      group.build("pitchToleranceDegrees", 10.0);

  private final List<VisionModule> visionModules = new ArrayList<>();

  private final AprilTagFieldLayout aprilTagLayout;

  private boolean useVisionForPoseEstimation = true;
  private boolean useMaxDistanceAwayFromExistingEstimate = true;
  private boolean useGyroBasedFilteringForVision = true;

  private final HashMap<Integer, Double> lastTagDetectionTimes = new HashMap<>();
  private final List<Integer> tagsUsedInPoseEstimation = new ArrayList<>();

  private final List<Pose3d> posesFedToPoseEstimator3D = new ArrayList<>();
  private final List<Pose2d> posesFedToPoseEstimator2D = new ArrayList<>();

  private final List<TimestampedVisionUpdate> allTimestampedVisionUpdates = new ArrayList<>();

  private final Consumer<List<TimestampedVisionUpdate>> visionEstimatesConsumer;
  private final Supplier<Pose2d> poseEstimatorPoseSupplier;
  private final Supplier<Rotation2d> currentGyroBasedRobotRotationSupplier;
  private int useMaxDistanceAwayFromExistingEstimateCount;
  private int useGyroBasedFilteringForVisionCount;

  public Vision(
      AprilTagFieldLayout aprilTagLayout,
      Supplier<Pose2d> poseEstimatorPoseSupplier,
      Supplier<Rotation2d> currentGyroBasedRobotRotationSupplier,
      Consumer<List<TimestampedVisionUpdate>> visionEstimatesConsumer,
      VisionModuleConfiguration... visionModuleConfigs) {

    this.visionEstimatesConsumer = visionEstimatesConsumer;
    this.poseEstimatorPoseSupplier = poseEstimatorPoseSupplier;
    this.currentGyroBasedRobotRotationSupplier = currentGyroBasedRobotRotationSupplier;
    this.aprilTagLayout = aprilTagLayout;

    for (VisionModuleConfiguration config : visionModuleConfigs) {
      visionModules.add(new VisionModule(logGroup, config, aprilTagLayout));
    }

    aprilTagLayout.getTags().forEach((AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, -1.0));

    Pose3d[] allTagsArray = new Pose3d[aprilTagLayout.getTags().size()];
    var allTagsList = aprilTagLayout.getTags();
    for (int i = 0; i < aprilTagLayout.getTags().size(); i++) {
      allTagsArray[i] = allTagsList.get(i).pose;
    }
    logAllAprilTags3D.info(allTagsArray);
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // update all inputs
      for (VisionModule module : visionModules) {
        module.visionIO.updateInputs(module.visionIOInputs);
      }

      boolean processVision = true;
      if (!useVisionForPoseEstimation) {
        processVision = false;
        setAllVisionModuleUnsuccessfulStatus(VisionResultStatus.VISION_DISABLED);
      }

      // process vision
      if (processVision) {
        for (VisionModule visionModule : visionModules) {
          visionModule.loggedFields = processVision(visionModule);
        }
      }

      // send successful results to pose estimator
      if (!allTimestampedVisionUpdates.isEmpty()) {
        visionEstimatesConsumer.accept(allTimestampedVisionUpdates);
        allTimestampedVisionUpdates.clear();
      }

      // log all vision module's logged fields
      var robotPose = new Pose3d(poseEstimatorPoseSupplier.get());
      for (VisionModule visionModule : visionModules) {
        visionModule.log(robotPose);
      }

      // activate alerts if camera is not connected
      for (VisionModule module : visionModules) {
        module.missingCameraAlert.set(module.visionIOInputs.connected);
      }

      // logging all visible tags
      List<Pose3d> allSeenTags = new ArrayList<>();
      for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
        if (detectionEntry.getValue() == Timer.getFPGATimestamp()) {
          var tagPose = aprilTagLayout.getTagPose(detectionEntry.getKey());
          if (tagPose.isPresent()) {
            allSeenTags.add(tagPose.get());
          }
        }
      }
      log_rawAllVisibleTags.info(allSeenTags.toArray(new Pose3d[0]));

      // logging tags actually used in pose estimation
      Pose3d[] tagsUsedInPoseEstimationPoses = new Pose3d[tagsUsedInPoseEstimation.size()];
      for (int i = 0; i < tagsUsedInPoseEstimation.size(); i++) {
        tagsUsedInPoseEstimationPoses[i] =
            aprilTagLayout.getTagPose(tagsUsedInPoseEstimation.get(i)).get();
      }

      log_tagsActuallyUsedInPoseEstimation.info(tagsUsedInPoseEstimationPoses);
      log_posesFedToPoseEstimator3D.info(posesFedToPoseEstimator3D.toArray(new Pose3d[0]));
      log_posesFedToPoseEstimator2D.info(posesFedToPoseEstimator2D.toArray(new Pose2d[0]));

      tagsUsedInPoseEstimation.clear();
      posesFedToPoseEstimator3D.clear();
      posesFedToPoseEstimator2D.clear();

      logUseVision.info(useVisionForPoseEstimation);
      logUseMaxDistanceAwayFromExistingEstimate.info(useMaxDistanceAwayFromExistingEstimate);
    }
  }

  public VisionResultLoggedFields processVision(VisionModule visionModule) {
    Pose2d prevEstimatedRobotPose = poseEstimatorPoseSupplier.get();

    PhotonPipelineResult cameraResult;
    double currentResultTimeStampCTRETime;

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
    var timeStampCameraResult = cameraResult.getTimestampSeconds();
    List<Integer> cleanTargets = new ArrayList<>();

    for (PhotonTrackedTarget target : cameraResult.getTargets()) {
      int fiducialId = target.getFiducialId();
      Optional<Pose3d> optTagPose = aprilTagLayout.getTagPose(fiducialId);
      if (optTagPose.isEmpty()) continue;

      cleanTargets.add(target.getFiducialId());
      lastTagDetectionTimes.put(fiducialId, timeStampCameraResult);
    }

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

    var photonPoseEstimatorOptionalResult = visionModule.photonPoseEstimator.update(cameraResult);
    if (photonPoseEstimatorOptionalResult.isEmpty()) {
      return VisionResultLoggedFields.unsuccessfulResult(
          VisionResultStatus.PHOTON_POSE_ESTIMATOR_OPTIONAL_RESULT_EMPTY);
    }

    // used to log if the multi tag failed
    EstimatedRobotPose photonPoseEstimatorResult = photonPoseEstimatorOptionalResult.get();
    var multiTagSuccess =
        photonPoseEstimatorResult.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    if (multiTagSuccess) {
      // only include tags used by PNP
      cleanTargets = cameraResult.getMultiTagResult().fiducialIDsUsed;
    }

    var newCalculatedRobotPose = photonPoseEstimatorResult.estimatedPose;
    Pose2d newCalculatedRobotPose2d = newCalculatedRobotPose.toPose2d();
    if (GeometryUtil.isPoseOutsideField(newCalculatedRobotPose2d)) {
      return VisionResultLoggedFields.unsuccessfulResult(
          VisionResultStatus.INVALID_POSE_OUTSIDE_FIELD);
    }

    Rotation3d newCalculatedRobotPoseRotation = newCalculatedRobotPose.getRotation();
    double posePitch = newCalculatedRobotPoseRotation.getX();
    double poseRoll = newCalculatedRobotPoseRotation.getY();
    double poseYaw = newCalculatedRobotPoseRotation.getZ();

    if (useGyroBasedFilteringForVision) {
      var gyroYaw = currentGyroBasedRobotRotationSupplier.get();
      var yawDelta = GeometryUtil.optimizeRotation(poseYaw - gyroYaw.getRadians());

      var absError = Math.toDegrees(Math.abs(yawDelta));
      if (absError > gyroFilteringToleranceDegrees.get()) {
        logRejectByGyro.info(++useGyroBasedFilteringForVisionCount);
        logRejectByGyroError.info(absError);
        return VisionResultLoggedFields.unsuccessfulResult(
            VisionResultStatus.NOT_CLOSE_ENOUGH_TO_GYRO_ROTATION);
      }
    }

    if (Math.abs(newCalculatedRobotPose.getZ()) >= zHeightToleranceMeters.get()) {
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.Z_HEIGHT_BAD);
    }

    var pitchAndRollToleranceValue = pitchAndRollToleranceDegrees.get();
    if (Math.abs(poseRoll) >= pitchAndRollToleranceValue
        || Math.abs(posePitch) >= pitchAndRollToleranceValue) {
      return VisionResultLoggedFields.unsuccessfulResult(VisionResultStatus.PITCH_OR_ROLL_BAD);
    }

    double tagAmbiguity = 0.0;
    double totalDistance = 0.0;
    double averageDistanceFromTags;

    if (multiTagSuccess) {

      // use average distance for PNP with multiple tags
      for (int tag : cleanTargets) {
        totalDistance +=
            aprilTagLayout
                .getTagPose(tag)
                .get()
                .getTranslation()
                .getDistance(newCalculatedRobotPose.getTranslation());
      }

      averageDistanceFromTags = totalDistance / (double) cleanTargets.size();
    } else {
      // Use single tag distance for all other methods
      totalDistance =
          aprilTagLayout
              .getTagPose(cameraResult.getBestTarget().getFiducialId())
              .get()
              .getTranslation()
              .getDistance(newCalculatedRobotPose.getTranslation());

      averageDistanceFromTags = totalDistance;
    }

    var distanceFromExistingPoseEstimate =
        GeometryUtil.getDist(prevEstimatedRobotPose, newCalculatedRobotPose2d);

    if (useMaxDistanceAwayFromExistingEstimate
        && (distanceFromExistingPoseEstimate
            > (maxValidDistanceAwayFromCurrentEstimateMeters.get() * numTargetsSeen))) {
      logRejectByDistance.info(++useMaxDistanceAwayFromExistingEstimateCount);
      logRejectByDistanceError.info(distanceFromExistingPoseEstimate);
      return VisionResultLoggedFields.unsuccessfulResult(
          VisionResultStatus.TOO_FAR_FROM_EXISTING_ESTIMATE);
    }

    var averageDistanceFromTagsSquared = averageDistanceFromTags * averageDistanceFromTags;
    var xyStandardDeviation =
        xyStdDevCoefficient.get() * averageDistanceFromTagsSquared / numTargetsSeen;
    var thetaStandardDeviation =
        thetaStdDevCoefficient.get() * averageDistanceFromTagsSquared / numTargetsSeen;

    var timestampedVisionUpdate =
        new PoseEstimator.TimestampedVisionUpdate(
            cleanTargets,
            currentResultTimeStampCTRETime,
            newCalculatedRobotPose2d,
            VecBuilder.fill(xyStandardDeviation, xyStandardDeviation, thetaStandardDeviation));

    allTimestampedVisionUpdates.add(timestampedVisionUpdate);

    posesFedToPoseEstimator3D.add(newCalculatedRobotPose);
    posesFedToPoseEstimator2D.add(newCalculatedRobotPose2d);
    tagsUsedInPoseEstimation.addAll(cleanTargets);

    VisionResultStatus status;
    if (numTargetsSeen == 1 || !multiTagSuccess) {
      status = VisionResultStatus.SUCCESSFUL_SINGLE_TAG;
    } else {
      status = VisionResultStatus.SUCCESSFUL_MULTI_TAG;
    }

    return new VisionResultLoggedFields(
        status,
        numTargetsSeen,
        tagAmbiguity,
        averageDistanceFromTags,
        distanceFromExistingPoseEstimate,
        xyStandardDeviation,
        thetaStandardDeviation,
        newCalculatedRobotPose,
        timestampedVisionUpdate.tags().toArray(new Integer[timestampedVisionUpdate.tags().size()]));
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

  public void setUsingVision(boolean value) {
    useVisionForPoseEstimation = value;
  }

  public void useGyroBasedFilteringForVision(boolean value) {
    useGyroBasedFilteringForVision = value;
  }

  public boolean isUsingVision() {
    return useVisionForPoseEstimation;
  }

  public static void restartPhotonVision(String ipString) {
    sendPhotonVisionCommand(ipString, "restartProgram");
  }

  public static void rebootPhotonVision(String ipString) {
    sendPhotonVisionCommand(ipString, "restartDevice");
  }

  public static void sendPhotonVisionCommand(String ipString, String command) {
    try {
      HttpClient httpClient = HttpClient.newHttpClient();
      HttpRequest request =
          HttpRequest.newBuilder()
              .uri(new URI("http://" + ipString + ":5800/api/utils/" + command))
              .POST(HttpRequest.BodyPublishers.noBody())
              .build();
      httpClient.sendAsync(request, HttpResponse.BodyHandlers.ofString());
    } catch (Exception ignored) {
    }
  }
}
