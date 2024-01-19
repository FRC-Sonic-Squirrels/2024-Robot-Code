// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.limelight.ProcessedGamepieceData;
import frc.robot.subsystems.swerve.Drivetrain;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToGamepiece extends Command {
  /** Creates a new DriveToGamepiece. */
  private Supplier<ProcessedGamepieceData> targetGamepiece;

  private final Drivetrain drive;

  private Supplier<Boolean> gamepieceIntaked;

  private LoggedTunableNumber rotationKp =
      new LoggedTunableNumber("DriveToGamepiece/rotationKp", 4.9);

  private LoggedTunableNumber Kp = new LoggedTunableNumber("DriveToGamepiece/Kp", 3.2);
  private LoggedTunableNumber xKpSourceArea =
      new LoggedTunableNumber("DriveToGamepiece/xKpSourceArea", 5.0);
  private LoggedTunableNumber yKpSourceArea =
      new LoggedTunableNumber("DriveToGamepiece/yKpSourceArea", 5.0);

  private LoggedTunableNumber Ki = new LoggedTunableNumber("DriveToGamepiece/Ki", 0.0);

  private LoggedTunableNumber Kd = new LoggedTunableNumber("DriveToGamepiece/Kd", 0.0);

  private PIDController rotationController = new PIDController(rotationKp.get(), 0, 0);

  private PIDController xController = new PIDController(Kp.get(), Ki.get(), Kd.get());
  private PIDController yController = new PIDController(Kp.get(), Ki.get(), Kd.get());

  private double rotationalErrorDegrees;
  private double xVel;
  private double yVel;
  private double rotVel;
  private LoggedTunableNumber allowedRotationalErrorDegrees =
      new LoggedTunableNumber("DriveToGamepiece/allowedRotationalErrorDegrees", 20);

  private LoggedTunableNumber advancedMode =
      new LoggedTunableNumber("DriveToGamepiece/advancedMode/doAdvancedMotion", 1);

  private double rotVelCorrection = 0;
  private Rotation2d gamepieceDirection = new Rotation2d();

  private Rotation2d sourceAngle = new Rotation2d();

  /** Drives robot to gamepiece, intended for ground gamepieces only */
  public DriveToGamepiece(
      Supplier<ProcessedGamepieceData> targetGamepiece,
      Drivetrain drive,
      Supplier<Boolean> gamepieceIntaked) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetGamepiece = targetGamepiece;
    this.drive = drive;
    this.gamepieceIntaked = gamepieceIntaked;

    addRequirements(drive);
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

    rotationalErrorDegrees =
        Math.abs(
            targetGamepiece.get().targetYaw.getDegrees()
                // - 180.0
                - drive.getRawOdometryPose().getRotation().getDegrees());
    Logger.recordOutput("rotationalErrorDegrees", rotationalErrorDegrees);

    if (isInSourceArea(targetGamepiece.get().globalPose)) {

      sourceAngle =
          new Rotation2d(
              isInBlueSourceArea(targetGamepiece.get().globalPose)
                  ? -1.0417663596685425
                  : -2.103952069121958);

      gamepieceDirection =
          new Rotation2d(targetGamepiece.get().pose.getX(), targetGamepiece.get().pose.getY());

      xVel = xKpSourceArea.get() * Math.cos(gamepieceDirection.getRadians());
      yVel = yKpSourceArea.get() * Math.sin(gamepieceDirection.getRadians());

      rotVel =
          rotationController.calculate(
              drive.getRawOdometryPose().getRotation().getRadians(), sourceAngle.getRadians());
    } else {
      if (advancedMode.get() == 0) {

        xVel =
            rotationalErrorDegrees < allowedRotationalErrorDegrees.get()
                ? xController.calculate(0.0, targetGamepiece.get().pose.getX())
                : 0.0;
        yVel =
            rotationalErrorDegrees < allowedRotationalErrorDegrees.get()
                ? yController.calculate(0.0, targetGamepiece.get().pose.getY())
                : 0.0;
      } else {
        xVel =
            xController.calculate(0.0, targetGamepiece.get().pose.getX())
                / Math.max(rotationalErrorDegrees / allowedRotationalErrorDegrees.get(), 1.0);
        yVel =
            yController.calculate(0.0, targetGamepiece.get().pose.getY())
                / Math.max(rotationalErrorDegrees / allowedRotationalErrorDegrees.get(), 1.0);
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
              drive.getRawOdometryPose().getRotation().getRadians(),
              targetGamepiece.get().targetYaw.getRadians());
    }

    // TODO: ---------------change to estimated pose if using this IRL------------------
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVel, yVel, rotVel, drive.getRawOdometryPose().getRotation()));

    Logger.recordOutput("DriveToGamepiece/rotationalErrorDegrees", rotationalErrorDegrees);
    Logger.recordOutput("DriveToGamepiece/xVel", xVel);
    Logger.recordOutput("DriveToGamepiece/yVel", yVel);
    Logger.recordOutput("DriveToGamepiece/rotVel", rotVel);

    Logger.recordOutput("ActiveCommands/DriveToGamepiece", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
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