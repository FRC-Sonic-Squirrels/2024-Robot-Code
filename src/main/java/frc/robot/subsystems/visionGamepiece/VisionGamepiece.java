// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.visionGamepiece;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.function.Function;

public class VisionGamepiece extends SubsystemBase {
  private static final String ROOT_TABLE = "VisionGamepiece";

  private static final LoggerGroup logInputs = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Bool logInputs_isConnected =
      logInputs.buildBoolean("IsConnected");
  private static final LoggerEntry.Bool logInputs_validTarget =
      logInputs.buildBoolean("ValidTarget");
  private static final LoggerEntry.Decimal logInputs_totalLatencyMs =
      logInputs.buildDecimal("TotalLatencyMs");
  private static final LoggerEntry.Decimal logInputs_timestamp =
      logInputs.buildDecimal("Timestamp");
  private static final LoggerEntry.DecimalArray logInputs_pitch =
      logInputs.buildDecimalArray("Pitch");
  private static final LoggerEntry.DecimalArray logInputs_yaw = logInputs.buildDecimalArray("Yaw");
  private static final LoggerEntry.DecimalArray logInputs_area =
      logInputs.buildDecimalArray("Area");
  private static final LoggerEntry.Integer logInputs_targetCount =
      logInputs.buildInteger("TargetCount");

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Integer logGamepieceCount =
      logGroup.buildInteger("gamepieceCount");
  private static final LoggerEntry.Decimal logTotalLatencyMs =
      logGroup.buildDecimal("totalLatencyMs");
  private static final LoggerEntry.StructArray<Pose3d> logGamepiecePoseArray =
      logGroup.buildStructArray(Pose3d.class, "GamepiecePoseArray");
  private static final LoggerEntry.Decimal logFudgeFromYaw = logGroup.buildDecimal("fudgeFromYaw");
  private static final LoggerEntry.Decimal logFudgeFromPitch =
      logGroup.buildDecimal("fudgeFromPitch");

  private static final LoggerGroup logGroupClosestGamepiece = logGroup.subgroup("ClosestGamepiece");
  private static final LoggerEntry.Decimal log_distance =
      logGroupClosestGamepiece.buildDecimal("distance");
  private static final LoggerEntry.Decimal log_targetYawDegrees =
      logGroupClosestGamepiece.buildDecimal("targetYawDegrees");
  private static final LoggerEntry.Decimal log_targetPitchDegrees =
      logGroupClosestGamepiece.buildDecimal("targetPitchDegrees");
  private static final LoggerEntry.Struct<Pose3d> log_poseRobotCentric =
      logGroupClosestGamepiece.buildStruct(Pose3d.class, "poseRobotCentric");
  private static final LoggerEntry.Struct<Pose3d> log_pose =
      logGroupClosestGamepiece.buildStruct(Pose3d.class, "pose");
  private static final LoggerEntry.Decimal log_timestamp =
      logGroupClosestGamepiece.buildDecimal("timestamp");

  private static final TunableNumberGroup groupTunable = new TunableNumberGroup(ROOT_TABLE);
  private static final LoggedTunableNumber pitchFudgeFactor =
      groupTunable.build("pitchFudgeFactor", 21.85);
  private static final LoggedTunableNumber yawFudgeFactor =
      groupTunable.build("yawFudgeFactor", 56.16);

  private final VisionGamepieceIO io;
  private final VisionGamepieceIO.Inputs inputs = new VisionGamepieceIO.Inputs();

  private final Function<Double, Pose2d> robotPoseSupplier;
  private final ArrayList<ProcessedGamepieceData> seenGamePieces = new ArrayList<>();

  private boolean usingGamepieceDetection = true;

  /** Creates a new VisionGamepiece. */
  public VisionGamepiece(VisionGamepieceIO io, Function<Double, Pose2d> robotPose) {
    this.io = io;
    this.robotPoseSupplier = robotPose;
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming(ROOT_TABLE)) {
      logTotalLatencyMs.info(inputs.totalLatencyMs);
      logInputs_isConnected.info(inputs.isConnected);
      logInputs_validTarget.info(inputs.validTarget);
      logInputs_timestamp.info(inputs.timestamp);
      logInputs_targetCount.info(inputs.targetCount);

      // GAMEPIECE DETECTION --------------------------------------------:
      if (usingGamepieceDetection) {
        io.updateInputs(inputs);
        logInputs_pitch.info(inputs.pitch);
        logInputs_yaw.info(inputs.yaw);
        logInputs_area.info(inputs.area);

        // uncomment if simming for target gamepieces ---------------------------------------

        //     Gamepieces (8.273, yMeters, 0.0):
        //  G1: yMeters = 7.474
        //  G2: yMeters = 5.792
        //  G3: yMeters = 4.11
        //  G4: yMeters = 2.428
        //  G5: yMeters = 0.742

        // Pose2d g1 = new Pose2d(8.273, 7.474, Constants.zeroRotation2d);
        // Pose2d g2 = new Pose2d(8.273, 5.792, Constants.zeroRotation2d);
        // Pose2d g3 = new Pose2d(8.273, 4.11, Constants.zeroRotation2d);
        // Pose2d g4 = new Pose2d(8.273, 2.428, Constants.zeroRotation2d);
        // Pose2d g5 = new Pose2d(8.273, 2.428, Constants.zeroRotation2d);

        // processedGamepieceData =
        //     new ProcessedGamepieceData[] {
        //       new ProcessedGamepieceData(
        //           Constants.zeroRotation2d,
        //           Constants.zeroRotation2d,
        //           GeometryUtil.getDist(robotPoseSupplier.get(), g1),
        //           robotPoseSupplier.get().relativeTo(g1),
        //           g1,
        //           Timer.getFPGATimestamp()),
        //       new ProcessedGamepieceData(
        //           Constants.zeroRotation2d,
        //           Constants.zeroRotation2d,
        //           GeometryUtil.getDist(robotPoseSupplier.get(), g2),
        //           robotPoseSupplier.get().relativeTo(g2),
        //           g2,
        //           Timer.getFPGATimestamp()),
        //       new ProcessedGamepieceData(
        //           Constants.zeroRotation2d,
        //           Constants.zeroRotation2d,
        //           GeometryUtil.getDist(robotPoseSupplier.get(), g3),
        //           robotPoseSupplier.get().relativeTo(g3),
        //           g3,
        //           Timer.getFPGATimestamp()),
        //       new ProcessedGamepieceData(
        //           Constants.zeroRotation2d,
        //           Constants.zeroRotation2d,
        //           GeometryUtil.getDist(robotPoseSupplier.get(), g4),
        //           robotPoseSupplier.get().relativeTo(g4),
        //           g4,
        //           Timer.getFPGATimestamp()),
        //       new ProcessedGamepieceData(
        //           Constants.zeroRotation2d,
        //           Constants.zeroRotation2d,
        //           GeometryUtil.getDist(robotPoseSupplier.get(), g5),
        //           robotPoseSupplier.get().relativeTo(g5),
        //           g5,
        //           Timer.getFPGATimestamp())
        //     };

        // for (int i = 0; i < processedGamepieceData.length; i++) {
        //   if (i == 0) {
        //     closestGamepiece = processedGamepieceData[0];
        //   } else {
        //     if (processedGamepieceData[i].distance < closestGamepiece.distance) {
        //       closestGamepiece = processedGamepieceData[i];
        //     }
        //   }
        //   logGamepieceData(new RawGamepieceData(0, 0, 0), processedGamepieceData[i], i);
        // }

        // --------------------------------------------------------------------------------------

        // closestGamepiece.globalPose = new Pose2d(simPose.getX(), simPose.getY(), new
        // Rotation2d(0));

        if (inputs.validTarget) {
          for (int index = 0; index < inputs.targetCount; index++) {
            double pitch = inputs.pitch[index];
            double yaw = inputs.yaw[index];

            var processedGamepieceData = processGamepieceData(yaw, pitch, inputs.timestamp);

            boolean alreadySeen = false;

            for (int i = 0; i < seenGamePieces.size(); i++) {
              if (seenGamePieces.get(i).sameGamepiece(processedGamepieceData)) {
                seenGamePieces.set(i, processedGamepieceData);
                alreadySeen = true;
              }
            }

            if (!alreadySeen) seenGamePieces.add(processedGamepieceData);

            logGamepieceData(yaw, pitch, processedGamepieceData, index);
          }
        }

        var timestamp = Utils.getCurrentTimeSeconds();

        seenGamePieces.removeIf(previousSeenGamePiece -> previousSeenGamePiece.isStale(timestamp));

        var closestGamepiece = getClosestGamepiece();
        if (closestGamepiece != null) {
          Pose2d robotPose = robotPoseSupplier.apply(closestGamepiece.timestamp_RIOFPGA_capture);
          Pose3d pose =
              new Pose3d(
                  closestGamepiece.getRobotCentricPose(robotPose).getX(),
                  closestGamepiece.getRobotCentricPose(robotPose).getY(),
                  0.0254,
                  new Rotation3d());

          Pose3d globalPose =
              new Pose3d(
                  closestGamepiece.globalPose.getX(),
                  closestGamepiece.globalPose.getY(),
                  0.0254,
                  new Rotation3d());

          log_distance.info(closestGamepiece.getDistance(robotPose).in(Units.Meters));
          log_targetYawDegrees.info(closestGamepiece.getYaw(robotPose));
          log_targetPitchDegrees.info(closestGamepiece.getPitch(robotPose));
          log_poseRobotCentric.info(pose);
          log_pose.info(globalPose);
          log_timestamp.info(closestGamepiece.timestamp_RIOFPGA_capture);
        }

        var gamepiecePoses = new Pose3d[seenGamePieces.size()];
        for (int i = 0; i < seenGamePieces.size(); i++) {
          Pose2d pose = seenGamePieces.get(i).globalPose;
          gamepiecePoses[i] = new Pose3d(pose.getX(), pose.getY(), 0.0254, new Rotation3d());
        }
        logGamepieceCount.info(seenGamePieces.size());
        logGamepiecePoseArray.info(gamepiecePoses);
      }
      // APRIL TAG DETECTION -----------------------------------------:
      else {

      }
    }
  }

  public boolean isValidTarget() {
    return inputs.validTarget;
  }

  public ProcessedGamepieceData getClosestGamepiece() {
    ProcessedGamepieceData closestGamepiece = null;
    double closestDistance = Double.MAX_VALUE;
    var robotPose = robotPoseSupplier.apply(Double.NaN);

    for (var gamepiece : seenGamePieces) {
      var distance = gamepiece.getDistance(robotPose).magnitude();
      if (distance < closestDistance) {
        closestDistance = distance;
        closestGamepiece = gamepiece;
      }
    }

    return closestGamepiece;
  }

  private static Rotation2d targetYaw(double xOffset) {
    return new Rotation2d(
        Math.toRadians(
            -xOffset
                + Math.toDegrees(
                    Constants.VisionGamepieceConstants.GAMEPIECE_CAMERA_POSE
                        .getRotation()
                        .getZ())));
  }

  private static Rotation2d targetPitch(double yOffset) {
    return new Rotation2d(
        Math.toRadians(
            yOffset
                + 90.0
                + Math.toDegrees(
                    Constants.VisionGamepieceConstants.GAMEPIECE_CAMERA_POSE
                        .getRotation()
                        .getY())));
  }

  private double groundGamepieceDistance(
      Rotation2d targetPitch, Rotation2d cameraYaw, Rotation2d cameraPitch) {
    double fudgeFromYaw = yawFudgeFactor.get() * Math.sin(Math.abs(cameraYaw.getRadians()));
    double fudgeFromPitch = pitchFudgeFactor.get() * Math.sin(-cameraPitch.getRadians());

    logFudgeFromYaw.info(fudgeFromYaw);
    logFudgeFromPitch.info(fudgeFromPitch);

    return (Constants.VisionGamepieceConstants.GAMEPIECE_CAMERA_POSE.getZ()
                - (Constants.FieldConstants.Gamepieces.NOTE_OUTER_RADIUS
                        .minus(Constants.FieldConstants.Gamepieces.NOTE_INNER_RADIUS)
                        .in(Units.Meters))
                    / 2)
            * Math.tan(targetPitch.getRadians())
        + Units.Inches.of(fudgeFromYaw).in(Units.Meter)
        + Units.Inches.of(fudgeFromPitch).in(Units.Meter);
  }

  private Pose2d groundGamepiecePose(double distanceMeters, Rotation2d targetYaw) {
    return new Pose2d(new Translation2d(distanceMeters, targetYaw), Constants.zeroRotation2d);
  }

  private ProcessedGamepieceData processGamepieceData(double yaw, double pitch, double timestamp) {
    Rotation2d targetYaw = targetYaw(yaw);
    Rotation2d targetPitch = targetPitch(pitch);
    double distance =
        groundGamepieceDistance(
            targetPitch, Rotation2d.fromDegrees(yaw), Rotation2d.fromDegrees(pitch));
    Pose2d pose = groundGamepiecePose(distance, targetYaw);
    Pose2d robotPose = robotPoseSupplier.apply(timestamp);

    return new ProcessedGamepieceData(
        targetYaw,
        targetPitch,
        distance,
        pose,
        robotPose.transformBy(new Transform2d(pose.getX(), pose.getY(), pose.getRotation())),
        timestamp);
  }

  private void logGamepieceData(
      double yam, double pitch, ProcessedGamepieceData processedGamepieceData, int index) {
    Pose2d robotPose = robotPoseSupplier.apply(Double.NaN);
    var baseGroup = logGroup.subgroup("Gamepiece" + index);
    double distance = processedGamepieceData.getDistance(robotPose).in(Units.Inches);

    var rawGroup = baseGroup.subgroup("Raw");
    rawGroup.buildDecimal("yaw").info(Math.toDegrees(yam));
    rawGroup.buildDecimal("pitch").info(Math.toDegrees(pitch));

    var processedGroup = baseGroup.subgroup("Processed");
    processedGroup.buildDecimal("distanceInches").info(distance);
    processedGroup.buildDecimal("targetYaw").info(processedGamepieceData.getYaw(robotPose));
    processedGroup.buildDecimal("targetPitch").info(processedGamepieceData.getPitch(robotPose));
    processedGroup
        .buildStruct(Pose2d.class, "pose")
        .info(processedGamepieceData.getRobotCentricPose(robotPose));
    processedGroup.buildStruct(Pose2d.class, "globalPose").info(processedGamepieceData.globalPose);
    processedGroup.buildDecimal("timestamp").info(processedGamepieceData.timestamp_RIOFPGA_capture);
  }

  public void setPipelineIndex(int index) {
    usingGamepieceDetection = false;
    io.setPipelineIndex(index);
  }
}
