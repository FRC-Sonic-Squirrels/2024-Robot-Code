// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.team2930.StateMachine;
import frc.robot.DrivetrainWrapper;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public class AutoSubstateMachine extends StateMachine {
  private DrivetrainWrapper drive;
  private Shooter shooter;
  private EndEffector endEffector;
  private Intake intake;
  private ChoreoHelper choreoHelper;
  private RobotConfig config;
  private ChoreoTrajectory trajToGP;
  private ChoreoTrajectory trajToShoot;
  private Timer runTime = new Timer();
  private double distToIntakeGP = 1.0;
  private Supplier<ProcessedGamepieceData> closestGamepiece;

  /** Creates a new AutoSubstateMachine. */
  public AutoSubstateMachine(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      String trajToGP,
      String trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.config = config;
    this.trajToGP = Choreo.getTrajectory(trajToGP);
    this.trajToShoot = Choreo.getTrajectory(trajToShoot);
    this.closestGamepiece = closestGamepiece;

    setInitialState(followGPpathInit());
  }

  private StateHandler followGPpathInit() {
    choreoHelper =
        new ChoreoHelper(
            trajToGP,
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController(),
            drive.getPoseEstimatorPose());
    runTime.start();
    return followGPpath();
  }

  private StateHandler followGPpath() {
    drive.setVelocity(
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), runTime.get()));

    if (runTime.get() >= trajToGP.getTotalTime() + 1.0) return setStopped();

    if (closestGamepiece.get().distance <= distToIntakeGP)
      CommandScheduler.getInstance().schedule(new IntakeGamepiece(intake));

    return intake.getBeamBreak() ? followShootPathInit() : null;
  }

  private StateHandler followShootPathInit() {
    runTime.reset();
    choreoHelper =
        new ChoreoHelper(
            trajToShoot,
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController(),
            drive.getPoseEstimatorPose());
    return followShootPath();
  }

  private StateHandler followShootPath() {
    drive.setVelocity(
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), runTime.get()));
    ScoreSpeaker scoreSpeaker = new ScoreSpeaker(drive, shooter, () -> true);
    CommandScheduler.getInstance().schedule(scoreSpeaker);
    if (scoreSpeaker.isFinished()) return setDone();
    return null;
  }
}
