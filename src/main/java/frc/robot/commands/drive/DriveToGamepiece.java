// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
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

  private LoggedTunableNumber rotationKp =
      new LoggedTunableNumber("DriveToGamepiece/rotationKp", 4.9);

  private LoggedTunableNumber xKp = new LoggedTunableNumber("DriveToGamepiece/xKp", 3.2);
  private LoggedTunableNumber yKp = new LoggedTunableNumber("DriveToGamepiece/yKp", 3.2);

  private LoggedTunableNumber xKi = new LoggedTunableNumber("DriveToGamepiece/xKi", 0.0);
  private LoggedTunableNumber yKi = new LoggedTunableNumber("DriveToGamepiece/yKi", 0.0);

  private LoggedTunableNumber xKd = new LoggedTunableNumber("DriveToGamepiece/xKd", 0);
  private LoggedTunableNumber yKd = new LoggedTunableNumber("DriveToGamepiece/yKd", 0);

  private PIDController rotationController = new PIDController(rotationKp.get(), 0, 0);

  private PIDController xController = new PIDController(xKp.get(), xKi.get(), xKd.get());
  private PIDController yController = new PIDController(yKp.get(), yKi.get(), yKd.get());

  private double rotationalErrorDegrees;
  private double xVel;
  private double yVel;
  private double rotVel;
  private LoggedTunableNumber allowedRotationalErrorDegrees =
      new LoggedTunableNumber("DriveToGamepiece/allowedRotationalErrorDegrees", 20);

  private LoggedTunableNumber advancedMode =
      new LoggedTunableNumber("DriveToGamepiece/advancedMode/doAdvancedMotion", 0);

  private double rotVelCorrection = 0;

  /** Drives robot to gamepiece, intended for ground gamepieces only */
  public DriveToGamepiece(Supplier<ProcessedGamepieceData> targetGamepiece, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetGamepiece = targetGamepiece;
    this.drive = drive;

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
                - 180.0
                - drive.getPose().getRotation().getDegrees());
    Logger.recordOutput("rotationalErrorDegrees", rotationalErrorDegrees);

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

    rotVel =
        rotationController.calculate(
            drive.getPose().getRotation().getRadians(),
            targetGamepiece.get().targetYaw.getRadians());

    rotVelCorrection =
        Math.hypot(xVel, yVel)
            * Math.cos(
                targetGamepiece.get().targetYaw.getRadians()
                    - new Rotation2d(xVel, yVel).getRadians()
                    - Math.PI / 2)
            / targetGamepiece.get().distance;

    drive.runVelocity(new ChassisSpeeds(xVel, yVel, rotVel));

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
    return false;
  }
}
