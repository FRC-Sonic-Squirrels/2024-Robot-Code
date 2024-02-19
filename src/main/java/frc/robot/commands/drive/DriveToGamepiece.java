// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToGamepiece extends Command {
  /** Creates a new DriveToGamepiece. */
  private Supplier<ProcessedGamepieceData> targetGamepiece;

  private final DrivetrainWrapper drive;

  private Supplier<Boolean> gamepieceIntaked;

  private static final TunableNumberGroup group = new TunableNumberGroup("DriveToGamepiece");

  private LoggedTunableNumber rotationKp = group.build("rotationKp", 0.4);

  private LoggedTunableNumber Kp = group.build("Kp", 0.4);
  private LoggedTunableNumber xKpSourceArea = group.build("xKpSourceArea", 0.1);
  private LoggedTunableNumber yKpSourceArea = group.build("yKpSourceArea", 0.1);

  private LoggedTunableNumber Ki = group.build("Ki", 0.0);
  private LoggedTunableNumber Kd = group.build("Kd", 0.0);

  private PIDController rotationController = new PIDController(rotationKp.get(), 0, 0);

  private PIDController xController = new PIDController(Kp.get(), Ki.get(), Kd.get());
  private PIDController yController = new PIDController(Kp.get(), Ki.get(), Kd.get());

  private double rotationalErrorDegrees;
  private double xVel;
  private double yVel;
  private double rotVel;
  private LoggedTunableNumber allowedRotationalErrorDegrees =
      group.build("allowedRotationalErrorDegrees", 20);
  private LoggedTunableNumber advancedMode = group.build("advancedMode/doAdvancedMotion", 1);

  private double rotVelCorrection = 0;
  private Rotation2d gamepieceDirection = new Rotation2d();

  private Rotation2d sourceAngle = new Rotation2d();

  /** Drives robot to gamepiece, intended for ground gamepieces only */
  public DriveToGamepiece(
      Supplier<ProcessedGamepieceData> targetGamepiece,
      DrivetrainWrapper drive,
      Supplier<Boolean> gamepieceIntaked) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetGamepiece = targetGamepiece;
    this.drive = drive;
    this.gamepieceIntaked = gamepieceIntaked;

    setName("DriveToGamepiece");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(2);

    xController.reset();
    xController.setTolerance(0.01);

    yController.reset();
    yController.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var closestGamepiece = targetGamepiece.get();
    if (closestGamepiece == null) return;

    var poseEstimatorPose = drive.getPoseEstimatorPose();

    rotationalErrorDegrees =
        Math.abs(
            closestGamepiece.getYaw(drive.getPoseEstimatorPose()).getDegrees()
                // - 180.0
                - poseEstimatorPose.getRotation().getDegrees());
    Logger.recordOutput("rotationalErrorDegrees", rotationalErrorDegrees);

    if (isInSourceArea(closestGamepiece.getGlobalPose())) {
      sourceAngle =
          new Rotation2d(
              isInBlueSourceArea(closestGamepiece.getGlobalPose())
                  ? -1.0417663596685425
                  : -2.103952069121958);

      xVel =
          xKpSourceArea.get()
              * Math.cos(closestGamepiece.getYaw(drive.getPoseEstimatorPose()).getRadians());
      yVel =
          yKpSourceArea.get()
              * Math.sin(closestGamepiece.getYaw(drive.getPoseEstimatorPose()).getRadians());

      rotVel =
          rotationController.calculate(
              poseEstimatorPose.getRotation().getRadians(), sourceAngle.getRadians());
    } else {
      double allowedRotationalErrorDegreesValue = allowedRotationalErrorDegrees.get();
      if (advancedMode.get() == 0) {
        xVel =
            rotationalErrorDegrees < allowedRotationalErrorDegreesValue
                ? xController.calculate(
                    0.0, closestGamepiece.getRobotCentricPose(drive.getPoseEstimatorPose()).getX())
                : 0.0;
        yVel =
            rotationalErrorDegrees < allowedRotationalErrorDegreesValue
                ? yController.calculate(
                    0.0, closestGamepiece.getRobotCentricPose(drive.getPoseEstimatorPose()).getY())
                : 0.0;
      } else {
        xVel =
            xController.calculate(
                    0.0, closestGamepiece.getRobotCentricPose(drive.getPoseEstimatorPose()).getX())
                / Math.max(rotationalErrorDegrees / allowedRotationalErrorDegreesValue, 1.0);
        yVel =
            yController.calculate(
                    0.0, closestGamepiece.getRobotCentricPose(drive.getPoseEstimatorPose()).getY())
                / Math.max(rotationalErrorDegrees / allowedRotationalErrorDegreesValue, 1.0);
      }
      // rotVelCorrection =
      //     Math.hypot(xVel, yVel)
      //         * Math.cos(
      //             targetGamepiece.get().targetYaw.getRadians()
      //                 - new Rotation2d(xVel, yVel).getRadians()
      //                 - Math.PI / 2)
      //         / targetGamepiece.get().distance;

      rotVel =
          rotationController.calculate(
              poseEstimatorPose.getRotation().getRadians(),
              closestGamepiece.getYaw(drive.getPoseEstimatorPose()).getRadians());
    }

    // TODO: ---------------change to estimated pose if using this IRL------------------
    drive.setVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, poseEstimatorPose.getRotation()));

    Logger.recordOutput("DriveToGamepiece/rotationalErrorDegrees", rotationalErrorDegrees);
    Logger.recordOutput("DriveToGamepiece/xVel", xVel);
    Logger.recordOutput("DriveToGamepiece/yVel", yVel);
    Logger.recordOutput("DriveToGamepiece/rotVel", rotVel);

    Logger.recordOutput("ActiveCommands/DriveToGamepiece", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(new ChassisSpeeds(0, 0, 0));
    Logger.recordOutput("ActiveCommands/DriveToGamepiece", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gamepieceIntaked.get();
  }

  private boolean isInSourceArea(Pose2d pose) {
    return (pose.getX() >= 14.26539134979248 || pose.getX() <= 2.154930830001831)
        && pose.getY() <= 1.803399920463562;
  }

  private boolean isInBlueSourceArea(Pose2d pose) {
    return pose.getX() >= 14.26539134979248 && pose.getY() <= 1.803399920463562;
  }
}
