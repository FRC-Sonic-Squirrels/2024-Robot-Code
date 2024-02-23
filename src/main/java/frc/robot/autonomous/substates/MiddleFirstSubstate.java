package frc.robot.autonomous.substates;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public class MiddleFirstSubstate extends StateMachine {

  private DrivetrainWrapper drive;
  private Shooter shooter;
  private EndEffector endEffector;
  private Intake intake;
  private RobotConfig config;
  private ChoreoTrajectory traj;
  private Supplier<ProcessedGamepieceData> closestGamepiece;
  private ChoreoHelper choreoHelper;
  private ScoreSpeaker scoreSpeaker;
  private IntakeGamepiece intakeGamepiece;
  private boolean prevEndEffectorBeamBreak = true;
  private int gamepieceCounter = 0;
  private boolean reachedCenter = false;

  public MiddleFirstSubstate(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.config = config;
    this.traj = Choreo.getTrajectory("middleAuto.1");
    this.closestGamepiece = closestGamepiece;

    setInitialState(this::initFollowPath);
  }

  private StateHandler initFollowPath() {
    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            drive.getPoseEstimatorPose(),
            this.traj,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());

    return this::followPath;
  }

  private StateHandler followPath() {
    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());

    if (!endEffector.gamepieceInEndEffector() && prevEndEffectorBeamBreak) {
      intakeGamepiece = new IntakeGamepiece(intake, endEffector, shooter);
      intakeGamepiece.schedule();
    }

    if (endEffector.gamepieceInEndEffector() && !prevEndEffectorBeamBreak) {
      gamepieceCounter++;
      scoreSpeaker =
          new ScoreSpeaker(
              drive, shooter, endEffector, () -> true, gamepieceCounter == 3 ? 0.71 : 0.88);
      scoreSpeaker.schedule();
    }

    prevEndEffectorBeamBreak = endEffector.gamepieceInEndEffector();

    if (drive.getPoseEstimatorPose().getX() >= 8.1) {
      reachedCenter = true;
    }

    // if (reachedCenter
    //     && drive.getPoseEstimatorPose().getX() <= 8.05
    //     && !endEffector.gamepieceInEndEffector()) {
    //   return setStopped();
    // }

    if (chassisSpeeds != null) {
      // TODO: Check for note in intake.
      drive.setVelocityOverride(chassisSpeeds);
      return null;
    }

    if (scoreSpeaker.isFinished()) {
      return setDone();
    }

    return null;
  }
}
