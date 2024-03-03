package frc.robot.autonomous.substates;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.commands.shooter.ShooterScoreSpeakerStateMachine;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MiddleFirstSubstate extends StateMachine {

  private final DrivetrainWrapper drive;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final Intake intake;
  private final Elevator elevator;
  private final Arm arm;
  private final RobotConfig config;
  private final ChoreoTrajectory traj;
  private final Supplier<ProcessedGamepieceData> closestGamepiece;
  private ChoreoHelper choreoHelper;
  private int gamepieceCounter = 0;
  private boolean shooting;

  public MiddleFirstSubstate(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      Elevator elevator,
      Arm arm,
      RobotConfig config,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    super("MiddleFirstSub");

    this.drive = drive;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.elevator = elevator;
    this.arm =arm;
    this.config = config;
    this.traj = Choreo.getTrajectory("middleAuto");
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

    initIntake();

    return stateWithName("driveWhileOtherCommandRuns", this::driveWhileOtherCommandRuns);
  }

  private StateHandler driveWhileOtherCommandRuns() {
    Logger.recordOutput("Autonomous/driveWhileOtherCommandRuns", true);
    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());

    if (chassisSpeeds != null) {
      // TODO: Check for note in intake.
      drive.setVelocityOverride(chassisSpeeds);
      return null;
    }

    drive.resetVelocityOverride();

    if (shooting) {
      return null;
    }

    Logger.recordOutput("Autonomous/gamepieceCount", gamepieceCounter);
    if (gamepieceCounter == 3) {
      return setDone();
    }

    return null;
  }

  private void initIntake() {
    var cmd = new IntakeGamepiece(intake, endEffector, shooter, arm, elevator);

    spawnCommand(
        cmd,
        (c) -> {
          this.initShoot();
          return null;
        });
  }

  private void initShoot() {
    choreoHelper.pause(timeFromStart());
    shooting = true;

    var cmd = ShooterScoreSpeakerStateMachine.getAsCommand(drive, shooter, endEffector, intake, 15);

    spawnCommand(
        cmd,
        (c) -> {
          shooting = false;
          choreoHelper.resume(timeFromStart());
          gamepieceCounter++;
          if (gamepieceCounter < 3) {
            this.initIntake();
          }
          return null;
        });
  }
}
