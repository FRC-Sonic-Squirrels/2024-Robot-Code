// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.team2930.StateMachine;
import frc.robot.DrivetrainWrapper;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoSubstateMachine extends StateMachine {
  private DrivetrainWrapper drive;
  private Shooter shooter;
  private EndEffector endEffector;
  private Intake intake;
  private RobotConfig config;
  private ChoreoTrajectory trajToGP;
  private ChoreoTrajectory trajToShoot;
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

    setInitialState(followGPpath());
    Logger.recordOutput("Autonomous/SubstateConstructed", true);
  }

  private StateHandler followGPpath() {
    Logger.recordOutput("Autonomous/followGPpathStarted", true);
    return suspendForCommand(
        new FollowPath(config, drive, trajToGP),
        new ResumeStateHandlerFromCommand() {
          @Override
          public StateHandler advance(Command command) {
            if (!intake.getBeamBreak() && command.isFinished()) {
              Logger.recordOutput("Autonomous/stopped", true);
              return setStopped();
            }
            Logger.recordOutput("Autonomous/next", true);
            return followShootPath();
          }
        });
  }

  private StateHandler followShootPath() {
    Logger.recordOutput("Autonomous/followShootPathStarted", true);
    ScoreSpeaker scoreSpeaker = new ScoreSpeaker(drive, shooter, () -> true);
    CommandScheduler.getInstance().schedule(scoreSpeaker);
    if (scoreSpeaker.isFinished()) return setDone();
    return suspendForCommand(
        new FollowPath(config, drive, trajToShoot).alongWith(scoreSpeaker),
        new ResumeStateHandlerFromCommand() {
          @Override
          public StateHandler advance(Command command) {
            return setDone();
          }
        });
  }
}
