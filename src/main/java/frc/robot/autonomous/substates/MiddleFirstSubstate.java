package frc.robot.autonomous.substates;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.commands.ScoreSpeaker;
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
  private Trigger endEffectorTrigger;
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

    endEffectorTrigger = new Trigger(endEffector::intakeSideTOFDetectGamepiece).debounce(0.1);

    return this::followPath;
  }

  private StateHandler followPath() {
    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());

    if (!endEffectorTrigger.getAsBoolean()) {
      endEffector.setPercentOut(0.7);
    }

    if (endEffectorTrigger.getAsBoolean() && !prevEndEffectorBeamBreak) {
      endEffector.setPercentOut(0.0);
      gamepieceCounter++;
      scoreSpeaker =
          new ScoreSpeaker(drive, shooter, () -> true, gamepieceCounter == 3 ? 0.71 : 0.88);
      scoreSpeaker.schedule();
    }

    intake.setPercentOut(0.8);

    prevEndEffectorBeamBreak = endEffectorTrigger.getAsBoolean();

    if (drive.getPoseEstimatorPose().getX() >= 8.1) {
      reachedCenter = true;
    }

    if (reachedCenter
        && drive.getPoseEstimatorPose().getX() <= 8.05
        && !endEffectorTrigger.getAsBoolean()) {
      return setStopped();
    }

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
