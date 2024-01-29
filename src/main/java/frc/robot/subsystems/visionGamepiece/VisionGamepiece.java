// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.visionGamepiece;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

  /** Creates a new Limelight. */
  public VisionGamepiece(VisionGamepieceIO io, Supplier<Pose2d> robotPose) {
    this.io = io;
    this.robotPoseSupplier = robotPose;
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming("VisionGamepiece")) {
      io.updateInputs(inputs);

      Logger.processInputs("VisionGamepiece", inputs);

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
        rawGamepieceData = new RawGamepieceData[inputs.targetCount];
        processedGamepieceData = new ProcessedGamepieceData[inputs.targetCount];
        for (int index = 0; index < inputs.targetCount; index++) {
          rawGamepieceData[index] =
              new RawGamepieceData(
                  inputs.id[index], inputs.yaw[index], inputs.pitch[index], inputs.timestamp);
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

      Logger.recordOutput("Limelight/totalLatencyMs", inputs.totalLatencyMs);

      Logger.recordOutput("Limelight/ClosestGamepiece/distance", closestGamepiece.distance);
      Logger.recordOutput(
          "Limelight/ClosestGamepiece/targetYawDegrees", closestGamepiece.targetYaw.getDegrees());
      Logger.recordOutput(
          "Limelight/ClosestGamepiece/targetPitchDegrees",
          closestGamepiece.targetPitch.getDegrees());
      Logger.recordOutput("Limelight/ClosestGamepiece/poseRobotCentric", closestGamepiece.pose);
      Logger.recordOutput("Limelight/ClosestGamepiece/pose", closestGamepiece.globalPose);
      Logger.recordOutput(
          "Limelight/ClosestGamepiece/timestamp", closestGamepiece.timestamp_RIOFPGA_capture);
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
            - (Constants.FieldConstants.Gamepieces.NOTE_OUTER_RADIUS_METERS
                    - Constants.FieldConstants.Gamepieces.NOTE_INNER_RADIUS_METERS)
                / 2)
        * Math.tan(targetPitch.getRadians());
  }

  private Pose2d groundGamepiecePose(double distanceMeters, Rotation2d targetYaw) {
    return new Pose2d(new Translation2d(distanceMeters, targetYaw), new Rotation2d());
  }

  private ProcessedGamepieceData processGamepieceData(RawGamepieceData rawGamepieceData) {
    Rotation2d targetYaw = targetYaw(rawGamepieceData.yaw);
    Rotation2d targetPitch = targetPitch(rawGamepieceData.pitch);
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
        rawGamepieceData.timestamp);
  }

  private void logGamepieceData(
      RawGamepieceData rawGamepieceData, ProcessedGamepieceData processedGamepieceData, int index) {
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/fiducialID",
        rawGamepieceData.fiducialID);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/yaw", rawGamepieceData.yaw);
    Logger.recordOutput(
        "Limelight/Gamepieces/Gamepiece: " + index + "/Raw/pitch", rawGamepieceData.pitch);
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
}
