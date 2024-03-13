// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.visionGamepiece;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

public class VisionGamepiece extends SubsystemBase {
  private static final String ROOT_TABLE = "VisionGamepiece";

  private static final LoggerEntry logInputs = new LoggerEntry(ROOT_TABLE);
  private static final LoggerGroup logGroup = new LoggerGroup(ROOT_TABLE);
  private static final LoggerEntry logTotalLatencyMs = logGroup.build("totalLatencyMs");
  private static final LoggerEntry logGamepiecePoseArray = logGroup.build("GamepiecePoseArray");

  private static final LoggerGroup logGroupClosestGamepiece = logGroup.subgroup("ClosestGamepiece");
  private static final LoggerEntry log_distance = logGroupClosestGamepiece.build("distance");
  private static final LoggerEntry log_targetYawDegrees =
      logGroupClosestGamepiece.build("targetYawDegrees");
  private static final LoggerEntry log_targetPitchDegrees =
      logGroupClosestGamepiece.build("targetPitchDegrees");
  private static final LoggerEntry log_poseRobotCentric =
      logGroupClosestGamepiece.build("poseRobotCentric");
  private static final LoggerEntry log_pose = logGroupClosestGamepiece.build("pose");
  private static final LoggerEntry log_timestamp = logGroupClosestGamepiece.build("timestamp");

  private static final LoggedTunableNumber pitchFudgeFactor = new LoggedTunableNumber(ROOT_TABLE + "/pitchFudgeFactor", 0.0);
  private static final LoggedTunableNumber yawFudgeFactor = new LoggedTunableNumber(ROOT_TABLE + "/yawFudgeFactor", 0.0);

  private final VisionGamepieceIO io;
  private final VisionGamepieceIOInputsAutoLogged inputs = new VisionGamepieceIOInputsAutoLogged();

  private final Supplier<Pose2d> robotPoseSupplier;
  private final ArrayList<ProcessedGamepieceData> seenGamePieces = new ArrayList<>();

  /** Creates a new VisionGamepiece. */
  public VisionGamepiece(VisionGamepieceIO io, Supplier<Pose2d> robotPose) {
    this.io = io;
    this.robotPoseSupplier = robotPose;
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming(ROOT_TABLE)) {
      io.updateInputs(inputs);
      logInputs.info(inputs);

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
            if(seenGamePieces.get(i).sameGamepiece(processedGamepieceData)){
              seenGamePieces.set(i, processedGamepieceData);
              alreadySeen = true;
            }
          }

          if(!alreadySeen)
          seenGamePieces.add(processedGamepieceData);

          logGamepieceData(yaw, pitch, processedGamepieceData, index);
        }
      }

      var timestamp = Utils.getCurrentTimeSeconds();

      logTotalLatencyMs.info(inputs.totalLatencyMs);

      seenGamePieces.removeIf(previousSeenGamePiece -> previousSeenGamePiece.isStale(timestamp));

      var closestGamepiece = getClosestGamepiece();
      if (closestGamepiece != null) {
        Pose2d robotPose = robotPoseSupplier.get();
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
      Logger.recordOutput(ROOT_TABLE + "/gamepieceCount", seenGamePieces.size());
      logGamepiecePoseArray.info(gamepiecePoses);
    }
  }

  public boolean isValidTarget() {
    return inputs.validTarget;
  }

  public ProcessedGamepieceData getClosestGamepiece() {
    ProcessedGamepieceData closestGamepiece = null;
    double closestDistance = Double.MAX_VALUE;
    var robotPose = robotPoseSupplier.get();

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

  private double groundGamepieceDistance(Rotation2d targetPitch, Rotation2d cameraYaw, Rotation2d cameraPitch) {
    double fudgeFromYaw = yawFudgeFactor.get() * Math.sin(Math.abs(cameraYaw.getRadians()));
    double fudgeFromPitch = pitchFudgeFactor.get() * Math.sin(-cameraPitch.getRadians());

    Logger.recordOutput(ROOT_TABLE + "/fudgeFromYaw", fudgeFromYaw);
    Logger.recordOutput(ROOT_TABLE + "/fudgeFromPitch", fudgeFromPitch);

    return (Constants.VisionGamepieceConstants.GAMEPIECE_CAMERA_POSE.getZ()
                - (Constants.FieldConstants.Gamepieces.NOTE_OUTER_RADIUS
                        .minus(Constants.FieldConstants.Gamepieces.NOTE_INNER_RADIUS)
                        .in(Units.Meters))
                    / 2)
            * Math.tan(targetPitch.getRadians()) + Units.Inches.of(fudgeFromYaw).in(Units.Meter) + Units.Inches.of(fudgeFromPitch).in(Units.Meter);
  }

  private Pose2d groundGamepiecePose(double distanceMeters, Rotation2d targetYaw) {
    return new Pose2d(new Translation2d(distanceMeters, targetYaw), Constants.zeroRotation2d);
  }

  private ProcessedGamepieceData processGamepieceData(double yaw, double pitch, double timestamp) {
    Rotation2d targetYaw = targetYaw(yaw);
    Rotation2d targetPitch = targetPitch(pitch);
    double distance = groundGamepieceDistance(targetPitch, Rotation2d.fromDegrees(yaw), Rotation2d.fromDegrees(pitch));
    Pose2d pose = groundGamepiecePose(distance, targetYaw);
    Pose2d robotPose = robotPoseSupplier.get();

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
    Pose2d robotPose = robotPoseSupplier.get();
    var baseGroup = logGroup.subgroup("Gamepiece" + index);
    double distance = processedGamepieceData.getDistance(robotPose).in(Units.Inches);

    var rawGroup = baseGroup.subgroup("Raw");
    rawGroup.build("yaw").info(Math.toDegrees(yam));
    rawGroup.build("pitch").info(Math.toDegrees(pitch));

    var processedGroup = baseGroup.subgroup("Processed");
    processedGroup.build("distanceInches").info(distance);
    processedGroup.build("targetYaw").info(processedGamepieceData.getYaw(robotPose));
    processedGroup.build("targetPitch").info(processedGamepieceData.getPitch(robotPose));
    processedGroup.build("pose").info(processedGamepieceData.getRobotCentricPose(robotPose));
    processedGroup.build("globalPose").info(processedGamepieceData.globalPose);
    processedGroup.build("timestamp").info(processedGamepieceData.timestamp_RIOFPGA_capture);
  }
}
