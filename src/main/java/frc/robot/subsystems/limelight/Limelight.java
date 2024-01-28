// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.robot.subsystems.limelight.LimelightIO.LimelightIOInputs;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  private final LimelightIO io;
  private final LimelightIOInputs inputs = new LimelightIOInputs();

  private Supplier<Pose2d> robotPoseSupplier;

  private RawGamepieceData[] rawGamepieceData;
  private ProcessedGamepieceData[] processedGamepieceData;
  private ProcessedGamepieceData closestGamepiece =
      new ProcessedGamepieceData(
          new Rotation2d(), new Rotation2d(), 0, new Pose2d(), new Pose2d(), 0, 0);

  private static ObjectMapper mapper;
  private LimelightResults results;

  /** Creates a new Limelight. */
  public Limelight(LimelightIO io, Supplier<Pose2d> robotPose) {
    this.io = io;
    this.robotPoseSupplier = robotPose;
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming("Limelight")) {
      io.updateInputs(inputs);
      Logger.processInputs("Limelight", inputs);
      if (!inputs.json.isEmpty()) results = getLatestResults(inputs.json);

      // uncomment if simming for target gamepiece
      // center field:
      // Translation2d simPose = new Translation2d(8.288247108459473, 2.3854970932006836);
      // blue source:
      // Translation2d simPose = new Translation2d(15.472177505493164, 0.7243906259536743);
      // red source:
      // Translation2d simPose = new Translation2d(1.1043164730072021, 0.7243906259536743);

      // closestGamepiece.pose =
      //     new Pose2d(
      //         simPose.getX() - robotPoseSupplier.get().getX(),
      //         simPose.getY() - robotPoseSupplier.get().getY(),
      //         new Rotation2d(0));

      // closestGamepiece.globalPose = new Pose2d(simPose.getX(), simPose.getY(), new
      // Rotation2d(0));

      if (inputs.validTarget) {
        rawGamepieceData = new RawGamepieceData[results.targetingResults.targets_Detector.length];
        processedGamepieceData =
            new ProcessedGamepieceData[results.targetingResults.targets_Detector.length];
        for (int index = 0; index < results.targetingResults.targets_Detector.length; index++) {
          rawGamepieceData[index] =
              resultsToRawGamepiecData(
                  results.targetingResults.targets_Detector[index],
                  results.targetingResults.timestamp_RIOFPGA_capture);
          processedGamepieceData[index] = processGamepieceData(rawGamepieceData[index]);
          if (index == 0) {
            closestGamepiece = processedGamepieceData[0];
          } else {
            if (processedGamepieceData[index].distance < closestGamepiece.distance) {
              closestGamepiece = processedGamepieceData[index];
            }
          }
          logGamepieceData(rawGamepieceData[index], processedGamepieceData[index], index);
        }
      } else {
        closestGamepiece.targetYaw =
            new Rotation2d(closestGamepiece.pose.getX(), closestGamepiece.pose.getY());
        closestGamepiece.distance =
            Math.hypot(closestGamepiece.pose.getX(), closestGamepiece.pose.getY());
      }

      Logger.recordOutput(
          "Limelight/totalLatencyMs", inputs.pipelineLatencyMs + inputs.captureLatencyMs);

      Logger.recordOutput("Limelight/ClosestGamepiece/distance", closestGamepiece.distance);
      Logger.recordOutput(
          "Limelight/ClosestGamepiece/targetYawDegrees", closestGamepiece.targetYaw.getDegrees());
      Logger.recordOutput(
          "Limelight/ClosestGamepiece/targetPitchDegrees",
          closestGamepiece.targetPitch.getDegrees());
      Logger.recordOutput("Limelight/ClosestGamepiece/poseRobotCentric", closestGamepiece.pose);
      Logger.recordOutput("Limelight/ClosestGamepiece/pose", closestGamepiece.globalPose);
      Logger.recordOutput("Limelight/ClosestGamepiece/confidence", closestGamepiece.confidence);
      Logger.recordOutput(
          "Limelight/ClosestGamepiece/timestamp", closestGamepiece.timestamp_RIOFPGA_capture);
    }
  }

  /**
   * Changes the mode of the leds
   *
   * <ul>
   *   <ln>
   * </ul>
   *
   * <p>0: use the LED Mode set in the current pipeline
   *
   * <ul>
   *   <ln>
   * </ul>
   *
   * <p>1: force off
   *
   * <ul>
   *   <ln>
   * </ul>
   *
   * <p>2: force blink
   *
   * <ul>
   *   <ln>
   * </ul>
   *
   * <p>3: force on
   */
  public void ledMode(double mode) {
    io.ledMode(mode);
  }

  public boolean isValidTarget() {
    return inputs.validTarget;
  }

  public LimelightResults getLimelightResults() {
    return results;
  }

  public ProcessedGamepieceData getClosestGamepiece() {
    return closestGamepiece;
  }

  private static Rotation2d targetYaw(double xOffset) {
    return new Rotation2d(
        Math.toRadians(
            -xOffset
                + Math.toDegrees(
                    LimelightConstants.limelight.LIMELIGHT_POSE.getRotation().getZ())));
  }

  private static Rotation2d targetPitch(double yOffset) {
    return new Rotation2d(
        Math.toRadians(
            yOffset
                + 90.0
                + Math.toDegrees(
                    LimelightConstants.limelight.LIMELIGHT_POSE.getRotation().getY())));
  }

  private double groundGamepieceDistance(Rotation2d targetPitch) {
    return (LimelightConstants.limelight.LIMELIGHT_POSE.getZ()
            - (LimelightConstants.gamepieces.NOTE_OUTER_RADIUS_METERS
                    - LimelightConstants.gamepieces.NOTE_INNER_RADIUS_METERS)
                / 2)
        * Math.tan(targetPitch.getRadians());
  }

  private Pose2d groundGamepiecePose(double distanceMeters, Rotation2d targetYaw) {
    return new Pose2d(new Translation2d(distanceMeters, targetYaw), new Rotation2d());
  }

  private RawGamepieceData resultsToRawGamepiecData(
      LimelightTarget_Detector result, double timestamp_RIOFPGA_capture) {
    return new RawGamepieceData(
        result.className,
        result.classID,
        result.confidence,
        0,
        result.tx,
        result.tx_pixels,
        result.ty,
        result.ty_pixels,
        timestamp_RIOFPGA_capture);
  }

  private ProcessedGamepieceData processGamepieceData(RawGamepieceData rawGamepieceData) {
    Rotation2d targetYaw = targetYaw(rawGamepieceData.tx);
    Rotation2d targetPitch = targetPitch(rawGamepieceData.ty);
    double distance = groundGamepieceDistance(targetPitch);
    Pose2d pose = groundGamepiecePose(distance, targetYaw);
    return new ProcessedGamepieceData(
        targetYaw,
        targetPitch,
        distance,
        pose,
        robotPoseSupplier
            .get()
            .transformBy(new Transform2d(pose.getX(), pose.getY(), pose.getRotation())),
        rawGamepieceData.timestamp_RIOFPGA_capture,
        rawGamepieceData.confidence);
  }

  private void logGamepieceData(
      RawGamepieceData rawGamepieceData, ProcessedGamepieceData processedGamepieceData, int index) {
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/classID", rawGamepieceData.classID);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/className", rawGamepieceData.className);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/confidence",
        rawGamepieceData.confidence);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/ta", rawGamepieceData.ta);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/timestamp_RIOFPGA_capture",
        rawGamepieceData.timestamp_RIOFPGA_capture);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/tx", rawGamepieceData.tx);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/tx_pixels", rawGamepieceData.tx_pixels);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/ty", rawGamepieceData.ty);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/ty_pixels", rawGamepieceData.ty_pixels);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Processed/distance",
        processedGamepieceData.distance);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Processed/distanceInches",
        Units.metersToInches(processedGamepieceData.distance));
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Processed/targetYaw",
        processedGamepieceData.targetYaw);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Processed/targetPitch",
        processedGamepieceData.targetPitch);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Processed/pose",
        processedGamepieceData.pose);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Processed/globalPose",
        processedGamepieceData.globalPose);
  }

  // ----------------------- JSON DUMP PROCESSING BELOW -----------------------

  private static Pose3d toPose3D(double[] inData) {
    if (inData.length < 6) {
      System.err.println("Bad LL 3D Pose Data!");
      return new Pose3d();
    }
    return new Pose3d(
        new Translation3d(inData[0], inData[1], inData[2]),
        new Rotation3d(
            Units.degreesToRadians(inData[3]),
            Units.degreesToRadians(inData[4]),
            Units.degreesToRadians(inData[5])));
  }

  private static Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      System.err.println("Bad LL 2D Pose Data!");
      return new Pose2d();
    }
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
    return new Pose2d(tran2d, r2d);
  }

  public static class LimelightTarget_Retro {

    @JsonProperty("t6c_ts")
    private double[] cameraPose_TargetSpace;

    @JsonProperty("t6r_fs")
    private double[] robotPose_FieldSpace;

    @JsonProperty("t6r_ts")
    private double[] robotPose_TargetSpace;

    @JsonProperty("t6t_cs")
    private double[] targetPose_CameraSpace;

    @JsonProperty("t6t_rs")
    private double[] targetPose_RobotSpace;

    public Pose3d getCameraPose_TargetSpace() {
      return toPose3D(cameraPose_TargetSpace);
    }

    public Pose3d getRobotPose_FieldSpace() {
      return toPose3D(robotPose_FieldSpace);
    }

    public Pose3d getRobotPose_TargetSpace() {
      return toPose3D(robotPose_TargetSpace);
    }

    public Pose3d getTargetPose_CameraSpace() {
      return toPose3D(targetPose_CameraSpace);
    }

    public Pose3d getTargetPose_RobotSpace() {
      return toPose3D(targetPose_RobotSpace);
    }

    public Pose2d getCameraPose_TargetSpace2D() {
      return toPose2D(cameraPose_TargetSpace);
    }

    public Pose2d getRobotPose_FieldSpace2D() {
      return toPose2D(robotPose_FieldSpace);
    }

    public Pose2d getRobotPose_TargetSpace2D() {
      return toPose2D(robotPose_TargetSpace);
    }

    public Pose2d getTargetPose_CameraSpace2D() {
      return toPose2D(targetPose_CameraSpace);
    }

    public Pose2d getTargetPose_RobotSpace2D() {
      return toPose2D(targetPose_RobotSpace);
    }

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double ty_pixels;

    @JsonProperty("ts")
    public double ts;

    public LimelightTarget_Retro() {
      cameraPose_TargetSpace = new double[6];
      robotPose_FieldSpace = new double[6];
      robotPose_TargetSpace = new double[6];
      targetPose_CameraSpace = new double[6];
      targetPose_RobotSpace = new double[6];
    }
  }

  public static class LimelightTarget_Fiducial {
    @JsonProperty("fID")
    public double fiducialID;

    @JsonProperty("fam")
    public String fiducialFamily;

    @JsonProperty("t6c_ts")
    private double[] cameraPose_TargetSpace;

    @JsonProperty("t6r_fs")
    private double[] robotPose_FieldSpace;

    @JsonProperty("t6r_ts")
    private double[] robotPose_TargetSpace;

    @JsonProperty("t6t_cs")
    private double[] targetPose_CameraSpace;

    @JsonProperty("t6t_rs")
    private double[] targetPose_RobotSpace;

    public Pose3d getCameraPose_TargetSpace() {
      return toPose3D(cameraPose_TargetSpace);
    }

    public Pose3d getRobotPose_FieldSpace() {
      return toPose3D(robotPose_FieldSpace);
    }

    public Pose3d getRobotPose_TargetSpace() {
      return toPose3D(robotPose_TargetSpace);
    }

    public Pose3d getTargetPose_CameraSpace() {
      return toPose3D(targetPose_CameraSpace);
    }

    public Pose3d getTargetPose_RobotSpace() {
      return toPose3D(targetPose_RobotSpace);
    }

    public Pose2d getCameraPose_TargetSpace2D() {
      return toPose2D(cameraPose_TargetSpace);
    }

    public Pose2d getRobotPose_FieldSpace2D() {
      return toPose2D(robotPose_FieldSpace);
    }

    public Pose2d getRobotPose_TargetSpace2D() {
      return toPose2D(robotPose_TargetSpace);
    }

    public Pose2d getTargetPose_CameraSpace2D() {
      return toPose2D(targetPose_CameraSpace);
    }

    public Pose2d getTargetPose_RobotSpace2D() {
      return toPose2D(targetPose_RobotSpace);
    }

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double ty_pixels;

    @JsonProperty("ts")
    public double ts;

    public LimelightTarget_Fiducial() {
      cameraPose_TargetSpace = new double[6];
      robotPose_FieldSpace = new double[6];
      robotPose_TargetSpace = new double[6];
      targetPose_CameraSpace = new double[6];
      targetPose_RobotSpace = new double[6];
    }
  }

  public static class LimelightTarget_Barcode {}

  public static class LimelightTarget_Classifier {
    @JsonProperty("class")
    public String className;

    @JsonProperty("classID")
    public double classID;

    @JsonProperty("conf")
    public double confidence;

    @JsonProperty("zone")
    public double zone;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double ty_pixels;

    public LimelightTarget_Classifier() {}
  }

  public static class LimelightTarget_Detector {
    @JsonProperty("class")
    public String className;

    @JsonProperty("classID")
    public double classID;

    @JsonProperty("conf")
    public double confidence;

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double ty_pixels;

    public LimelightTarget_Detector() {}
  }

  public static class Results {
    @JsonProperty("pID")
    public double pipelineID;

    @JsonProperty("tl")
    public double latency_pipeline;

    @JsonProperty("cl")
    public double latency_capture;

    public double latency_jsonParse;

    @JsonProperty("ts")
    public double timestamp_LIMELIGHT_publish;

    @JsonProperty("ts_rio")
    public double timestamp_RIOFPGA_capture;

    @JsonProperty("v")
    @JsonFormat(shape = Shape.NUMBER)
    public boolean valid;

    @JsonProperty("botpose")
    public double[] botpose;

    @JsonProperty("botpose_wpired")
    public double[] botpose_wpired;

    @JsonProperty("botpose_wpiblue")
    public double[] botpose_wpiblue;

    @JsonProperty("t6c_rs")
    public double[] camerapose_robotspace;

    public Pose3d getBotPose3d() {
      return toPose3D(botpose);
    }

    public Pose3d getBotPose3d_wpiRed() {
      return toPose3D(botpose_wpired);
    }

    public Pose3d getBotPose3d_wpiBlue() {
      return toPose3D(botpose_wpiblue);
    }

    public Pose2d getBotPose2d() {
      return toPose2D(botpose);
    }

    public Pose2d getBotPose2d_wpiRed() {
      return toPose2D(botpose_wpired);
    }

    public Pose2d getBotPose2d_wpiBlue() {
      return toPose2D(botpose_wpiblue);
    }

    @JsonProperty("Retro")
    public LimelightTarget_Retro[] targets_Retro;

    @JsonProperty("Fiducial")
    public LimelightTarget_Fiducial[] targets_Fiducials;

    @JsonProperty("Classifier")
    public LimelightTarget_Classifier[] targets_Classifier;

    @JsonProperty("Detector")
    public LimelightTarget_Detector[] targets_Detector;

    @JsonProperty("Barcode")
    public LimelightTarget_Barcode[] targets_Barcode;

    public Results() {
      botpose = new double[6];
      botpose_wpired = new double[6];
      botpose_wpiblue = new double[6];
      camerapose_robotspace = new double[6];
      targets_Retro = new LimelightTarget_Retro[0];
      targets_Fiducials = new LimelightTarget_Fiducial[0];
      targets_Classifier = new LimelightTarget_Classifier[0];
      targets_Detector = new LimelightTarget_Detector[0];
      targets_Barcode = new LimelightTarget_Barcode[0];
    }
  }

  public static class LimelightResults {
    @JsonProperty("Results")
    public Results targetingResults;

    public LimelightResults() {
      targetingResults = new Results();
    }
  }

  /** Parses Limelight's JSON results dump into a LimelightResults Object */
  public static LimelightResults getLatestResults(String json) {

    long start = System.nanoTime();
    LimelightResults results = new LimelightResults();
    if (mapper == null) {
      mapper =
          new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    }

    try {
      results = mapper.readValue(json, LimelightResults.class);
    } catch (JsonProcessingException e) {
      System.err.println("lljson error: " + e.getMessage());
    }

    long end = System.nanoTime();
    double millis = (end - start) * .000001;
    results.targetingResults.latency_jsonParse = millis;

    return results;
  }
}
