// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.DriveToGamepieceHelper;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public class DriveToGamepiece extends Command {
  /** Creates a new DriveToGamepiece. */
  private Supplier<ProcessedGamepieceData> targetGamepiece;

  private final DrivetrainWrapper wrapper;

  private Supplier<Boolean> gamepieceIntaked;

  private DriveToGamepieceHelper helper = new DriveToGamepieceHelper();

  private Supplier<Pose2d> pose;

  /** Drives robot to gamepiece, intended for ground gamepieces only */
  public DriveToGamepiece(
      Supplier<ProcessedGamepieceData> targetGamepiece,
      DrivetrainWrapper wrapper,
      Supplier<Boolean> gamepieceIntaked,
      Supplier<Pose2d> pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetGamepiece = targetGamepiece;
    this.wrapper = wrapper;
    this.gamepieceIntaked = gamepieceIntaked;
    this.pose = pose;

    setName("DriveToGamepiece");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    helper = new DriveToGamepieceHelper();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds =
        helper.calculateChassisSpeeds(
            targetGamepiece.get().globalPose.getTranslation(), pose.get());

    if (speeds != null) wrapper.setVelocityOverride(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrapper.resetVelocityOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gamepieceIntaked.get();
  }
}
