// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.visionGamepiece;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionGamepiece extends SubsystemBase {
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
    try (var ignored = new ExecutionTiming("VisionGamepiece")) {
      io.updateInputs(inputs);

      Logger.processInputs("VisionGamepiece", inputs);

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

          seenGamePieces.removeIf(
              previousSeenGamePiece -> previousSeenGamePiece.sameGamepiece(processedGamepieceData));

          seenGamePieces.add(processedGamepieceData);

          logGamepieceData(yaw, pitch, processedGamepieceData, index);
        }
      }

      var timestamp = Timer.getFPGATimestamp();

      seenGamePieces.removeIf(previousSeenGamePiece -> previousSeenGamePiece.isStale(timestamp));

      log("totalLatencyMs", inputs.totalLatencyMs);

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

        log("ClosestGamepiece/distance", closestGamepiece.getDistance(robotPose).in(Units.Meters));
        log("ClosestGamepiece/targetYawDegrees", closestGamepiece.getYaw(robotPose));
        log("ClosestGamepiece/targetPitchDegrees", closestGamepiece.getPitch(robotPose));
        log("ClosestGamepiece/poseRobotCentric", pose);
        log("ClosestGamepiece/pose", globalPose);
        log("ClosestGamepiece/timestamp", closestGamepiece.timestamp_RIOFPGA_capture);
      }

      var gamepiecePoses = new Pose3d[seenGamePieces.size()];
      for (int i = 0; i < seenGamePieces.size(); i++) {
        Pose2d pose = seenGamePieces.get(i).globalPose;
        gamepiecePoses[i] = new Pose3d(pose.getX(), pose.getY(), 0.0254, new Rotation3d());
      }
      log("GamepiecePoseArray", gamepiecePoses);
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

  private double groundGamepieceDistance(Rotation2d targetPitch) {
    return (Constants.VisionGamepieceConstants.GAMEPIECE_CAMERA_POSE.getZ()
            - (Constants.FieldConstants.Gamepieces.NOTE_OUTER_RADIUS
                    .minus(Constants.FieldConstants.Gamepieces.NOTE_INNER_RADIUS)
                    .in(Units.Meters))
                / 2)
        * Math.tan(targetPitch.getRadians());
  }

  private Pose2d groundGamepiecePose(double distanceMeters, Rotation2d targetYaw) {
    return new Pose2d(new Translation2d(distanceMeters, targetYaw), Constants.zeroRotation2d);
  }

  private ProcessedGamepieceData processGamepieceData(double yaw, double pitch, double timestamp) {
    Rotation2d targetYaw = targetYaw(yaw);
    Rotation2d targetPitch = targetPitch(pitch);
    double distance = groundGamepieceDistance(targetPitch);
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
    String baseName = "Gamepiece" + index;
    double distance = processedGamepieceData.getDistance(robotPose).in(Units.Inches);

    log(baseName + "/Raw/yaw", Math.toDegrees(yam));
    log(baseName + "/Raw/pitch", Math.toDegrees(pitch));
    log(baseName + "/Processed/distanceInches", distance);
    log(baseName + "/Processed/targetYaw", processedGamepieceData.getYaw(robotPose));
    log(baseName + "/Processed/targetPitch", processedGamepieceData.getPitch(robotPose));
    log(baseName + "/Processed/pose", processedGamepieceData.getRobotCentricPose(robotPose));
    log(baseName + "/Processed/globalPose", processedGamepieceData.globalPose);
  }

  private static void log(String key, double value) {
    Logger.recordOutput("VisionGamepiece/" + key, value);
  }

  private static void log(String key, Pose2d value) {
    Logger.recordOutput("VisionGamepiece/" + key, value);
  }

  private static void log(String key, Pose3d value) {
    Logger.recordOutput("VisionGamepiece/" + key, value);
  }

  private static void log(String key, Pose3d[] value) {
    Logger.recordOutput("VisionGamepiece/" + key, value);
  }

  private static void log(String key, Rotation2d value) {
    log(key, value.getDegrees());
  }
}
