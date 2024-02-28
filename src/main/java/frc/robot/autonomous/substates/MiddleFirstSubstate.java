package frc.robot.autonomous.substates;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.StateMachine;
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
import org.littletonrobotics.junction.Logger;

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
  private boolean prevEndEffectorBeamBreak = false;
  private boolean prevNoteInRobot = true;
  private int gamepieceCounter = 0;
  private boolean reachedCenter = false;
  private boolean hasShotGP[] = new boolean[] {false, false, false, false, false};

  public MiddleFirstSubstate(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    super("MiddleFirstSub");

    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.config = config;
    this.traj = Choreo.getTrajectory("middleAuto.1");
    this.closestGamepiece = closestGamepiece;

    setInitialState(stateWithName("init", this::init));
  }

  private StateHandler init() {
    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            drive.getPoseEstimatorPose(),
            this.traj,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());

    return stateWithName("initIntake", this::initIntake);
  }

  private StateHandler initIntake() {
    spawnCommand(
        new IntakeGamepiece(intake, endEffector, shooter).until(endEffector::noteInEndEffector),
        (c) -> {
          return this::initShoot;
        });

    return this::driveWhileOtherCommandRuns;
  }

  private StateHandler driveWhileOtherCommandRuns() {
    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());

    if (chassisSpeeds != null) {
      // TODO: Check for note in intake.
      drive.setVelocityOverride(chassisSpeeds);
      return null;
    } else {
      drive.resetVelocityOverride();
    }
    Logger.recordOutput("Autonomous/gamepieceCount", gamepieceCounter);
    if (gamepieceCounter == 3) {
      return setDone();
    }

    return null;
  }

  private StateHandler initShoot() {
    spawnCommand(
        ShooterScoreSpeakerStateMachine.getAsCommand(drive, shooter, endEffector, intake, 0.5),
        (c) -> {
          gamepieceCounter++;
          return this::initIntake;
        });

    return this::driveWhileOtherCommandRuns;
  }
}
