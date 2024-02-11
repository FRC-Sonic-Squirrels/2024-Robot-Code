// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DrivetrainWrapper;
import frc.robot.configs.RobotConfig;
import org.littletonrobotics.junction.Logger;

public class FollowPath extends Command {

  Timer runTime = new Timer();
  ChoreoHelper choreoHelper;
  DrivetrainWrapper drive;
  ChoreoTrajectory traj;

  /** Creates a new FollowPath. */
  public FollowPath(RobotConfig config, DrivetrainWrapper drive, ChoreoTrajectory traj) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    choreoHelper =
        new ChoreoHelper(
            traj,
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController(),
            drive.getPoseEstimatorPose());

    setName("FollowPath");
    Logger.recordOutput("Autonomous/constructed", true);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Autonomous/initialized", true);
    runTime.start();
  }

  @Override
  public void execute() {
    drive.setVelocityOverride(
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), runTime.get()));
    Logger.recordOutput("Autonomous/finished", runTime.get() >= traj.getTotalTime() + 0.5);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Autonomous/pathRunning", false);
  }
}
