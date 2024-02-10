// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DrivetrainWrapper;
import frc.robot.autonomous.ChoreoHelper;

public class TestChoreoHelper extends Command {
  private Timer runTime = new Timer();
  private DrivetrainWrapper drive;
  private ChoreoHelper helper;

  /** Creates a new TestChoreoHelper. */
  public TestChoreoHelper(DrivetrainWrapper drive) {
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    helper =
        new ChoreoHelper(
            Choreo.getTrajectory("TestFlipping"),
            new PIDController(6.0, 0, 0),
            new PIDController(4.9, 0, 0),
            drive.getPoseEstimatorPose());
    runTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setVelocity(helper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), runTime.get()));
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
