// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.subsystems.swerve.Drivetrain;

public class TestChoreoHelper extends Command {
  private Timer runTime = new Timer();
  private Drivetrain drive;
  private ChoreoHelper helper;

  /** Creates a new TestChoreoHelper. */
  public TestChoreoHelper(Drivetrain drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    helper =
        new ChoreoHelper(
            "TestFlipping",
            new PIDController(6.0, 0, 0),
            new PIDController(4.9, 0, 0),
            drive.getPoseEstimatorPose());
    runTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            helper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), runTime.get()),
            drive.getPoseEstimatorPose().getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runTime.stop();
    runTime.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
