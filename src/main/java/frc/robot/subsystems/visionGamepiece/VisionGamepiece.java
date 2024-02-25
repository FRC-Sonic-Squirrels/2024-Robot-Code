// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.visionGamepiece;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionGamepiece extends SubsystemBase {
  private final VisionGamepieceIO io;
  private final VisionGamepieceIOInputsAutoLogged inputs = new VisionGamepieceIOInputsAutoLogged();

  private Supplier<Pose2d> robotPoseSupplier;

  private RawGamepieceData[] rawGamepieceData;
  private ProcessedGamepieceData[] processedGamepieceData;
  private ProcessedGamepieceData closestGamepiece =
      new ProcessedGamepieceData(
          new Rotation2d(), new Rotation2d(), 0, new Pose2d(), new Pose2d(), 0);

  private static ObjectMapper mapper;

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

      // Pose2d g1 = new Pose2d(8.273, 7.474, new Rotation2d());
      // Pose2d g2 = new Pose2d(8.273, 5.792, new Rotation2d());
      // Pose2d g3 = new Pose2d(8.273, 4.11, new Rotation2d());
      // Pose2d g4 = new Pose2d(8.273, 2.428, new Rotation2d());
      // Pose2d g5 = new Pose2d(8.273, 2.428, new Rotation2d());

      // processedGamepieceData =
      //     new ProcessedGamepieceData[] {
      //       new ProcessedGamepieceData(
      //           new Rotation2d(),
      //           new Rotation2d(),
      //           GeometryUtil.getDist(robotPoseSupplier.get(), g1),
      //           robotPoseSupplier.get().relativeTo(g1),
      //           g1,
      //           Timer.getFPGATimestamp()),
      //       new ProcessedGamepieceData(
      //           new Rotation2d(),
      //           new Rotation2d(),
      //           GeometryUtil.getDist(robotPoseSupplier.get(), g2),
      //           robotPoseSupplier.get().relativeTo(g2),
      //           g2,
      //           Timer.getFPGATimestamp()),
      //       new ProcessedGamepieceData(
      //           new Rotation2d(),
      //           new Rotation2d(),
      //           GeometryUtil.getDist(robotPoseSupplier.get(), g3),
      //           robotPoseSupplier.get().relativeTo(g3),
      //           g3,
      //           Timer.getFPGATimestamp()),
      //       new ProcessedGamepieceData(
      //           new Rotation2d(),
      //           new Rotation2d(),
      //           GeometryUtil.getDist(robotPoseSupplier.get(), g4),
      //           robotPoseSupplier.get().relativeTo(g4),
      //           g4,
      //           Timer.getFPGATimestamp()),
      //       new ProcessedGamepieceData(
      //           new Rotation2d(),
      //           new Rotation2d(),
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
        rawGamepieceData = new RawGamepieceData[inputs.targetCount];
        processedGamepieceData = new ProcessedGamepieceData[inputs.targetCount];
        for (int index = 0; index < inputs.targetCount; index++) {
          rawGamepieceData[index] =
              new RawGamepieceData(inputs.yaw[index], inputs.pitch[index], inputs.timestamp);
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

      Logger.recordOutput("VisionGamepiece/totalLatencyMs", inputs.totalLatencyMs);

      Logger.recordOutput("VisionGamepiece/ClosestGamepiece/distance", closestGamepiece.distance);
      Logger.recordOutput(
          "VisionGamepiece/ClosestGamepiece/targetYawDegrees",
          closestGamepiece.targetYaw.getDegrees());
      Logger.recordOutput(
          "VisionGamepiece/ClosestGamepiece/targetPitchDegrees",
          closestGamepiece.targetPitch.getDegrees());
      Logger.recordOutput(
          "VisionGamepiece/ClosestGamepiece/poseRobotCentric",
          new Pose3d(
              closestGamepiece.pose.getX(),
              closestGamepiece.pose.getY(),
              (double) 0.0254,
              new Rotation3d()));
      Logger.recordOutput(
          "VisionGamepiece/ClosestGamepiece/pose",
          new Pose3d(
              closestGamepiece.globalPose.getX(),
              closestGamepiece.globalPose.getY(),
              (double) 0.0254,
              new Rotation3d()));
      Logger.recordOutput(
          "VisionGamepiece/ClosestGamepiece/timestamp", closestGamepiece.timestamp_RIOFPGA_capture);
    }
  }

  public boolean isValidTarget() {
    return inputs.validTarget;
  }

  public ProcessedGamepieceData getClosestGamepiece() {
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
    return new Pose2d(new Translation2d(distanceMeters, targetYaw), new Rotation2d());
  }

  private ProcessedGamepieceData processGamepieceData(RawGamepieceData rawGamepieceData) {
    Rotation2d targetYaw = targetYaw(rawGamepieceData.yaw());
    Rotation2d targetPitch = targetPitch(rawGamepieceData.pitch());
    double distance = groundGamepieceDistance(targetPitch);
    Pose2d pose = groundGamepiecePose(distance, targetYaw);
    Pose2d robotPose = robotPoseSupplier.get();

    return new ProcessedGamepieceData(
        targetYaw,
        targetPitch,
        distance,
        pose,
        robotPose.transformBy(new Transform2d(pose.getX(), pose.getY(), pose.getRotation())),
        rawGamepieceData.timestamp());
  }

  private void logGamepieceData(
      RawGamepieceData rawGamepieceData, ProcessedGamepieceData processedGamepieceData, int index) {
    String baseName = "VisionGamepiece/Gamepieces/Gamepiece: " + index;

    Logger.recordOutput(baseName + "/Raw/yaw", Math.toDegrees(rawGamepieceData.yaw()));
    Logger.recordOutput(baseName + "/Raw/pitch", Math.toDegrees(rawGamepieceData.pitch()));
    Logger.recordOutput(baseName + "/Processed/distance", processedGamepieceData.distance);
    Logger.recordOutput(
        baseName + "/Processed/distanceInches",
        Units.Meters.of(processedGamepieceData.distance).in(Units.Inches));
    Logger.recordOutput(
        baseName + "/Processed/targetYaw", processedGamepieceData.targetYaw.getDegrees());
    Logger.recordOutput(
        baseName + "/Processed/targetPitch", processedGamepieceData.targetPitch.getDegrees());
    Logger.recordOutput(baseName + "/Processed/pose", processedGamepieceData.pose);
    // Logger.recordOutput(baseName + "/Processed/globalPose", new
    // Pose3d(processedGamepieceData.globalPose.getX(), processedGamepieceData.globalPose.getY(),
    // (Constants.FieldConstants.Gamepieces.NOTE_OUTER_RADIUS -
    // Constants.FieldConstants.Gamepieces.NOTE_OUTER_RADIUS) /2.0);
  }
}
