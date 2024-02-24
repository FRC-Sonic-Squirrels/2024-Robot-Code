package frc.robot.autonomous.substates;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.StateMachine;
import frc.robot.Constants;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.commands.shooter.ShooterScoreSpeakerStateMachine;
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
  private Command scoreSpeaker;
  private IntakeGamepiece intakeGamepiece;
  private boolean prevEndEffectorBeamBreak = true;
  private int gamepieceCounter = 0;
  private boolean reachedCenter = false;
  private boolean hasShotGP = false;

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

    if (!endEffector.noteInEndEffector() && prevEndEffectorBeamBreak) {
      intakeGamepiece = new IntakeGamepiece(intake, endEffector, shooter);
      intakeGamepiece.schedule();
    }

    if (endEffector.noteInEndEffector() && !prevEndEffectorBeamBreak) {
      gamepieceCounter++;
      scoreSpeaker = ShooterScoreSpeakerStateMachine.getAsCommand(drive, shooter, endEffector, 5);

      spawnCommand(
          scoreSpeaker,
          (command) -> {
            hasShotGP = true;
            return null;
          });
    }

    prevEndEffectorBeamBreak = endEffector.noteInEndEffector();

    if (drive.getPoseEstimatorPose().getX() >= 8.1) {
      reachedCenter = true;
    }
    if (Constants.unusedCode) {
      if (reachedCenter
          && drive.getPoseEstimatorPose().getX() <= 8.05
          && !endEffector.noteInEndEffector()) {
        return setStopped();
      }
    }

    if (chassisSpeeds != null) {
      // TODO: Check for note in intake.
      drive.setVelocityOverride(chassisSpeeds);
      return null;
    }

    if (hasShotGP) {
      return setDone();
    }

    return null;
  }
}
