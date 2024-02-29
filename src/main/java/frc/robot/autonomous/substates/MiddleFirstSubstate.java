package frc.robot.autonomous.substates;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj.Timer;
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

  private final DrivetrainWrapper drive;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final Intake intake;
  private final RobotConfig config;
  private final ChoreoTrajectory traj;
  private final Supplier<ProcessedGamepieceData> closestGamepiece;
  private ChoreoHelper choreoHelper;
  private int gamepieceCounter = 0;
  private Timer timer = new Timer();
  private boolean shooting;

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
    this.traj = Choreo.getTrajectory("middleAuto");
    this.closestGamepiece = closestGamepiece;

    setInitialState(stateWithName("init", this::init));
  }

  private StateHandler init() {
    timer.reset();
    timer.start();
    choreoHelper =
        new ChoreoHelper(
            timer.get(),
            drive.getPoseEstimatorPose(),
            this.traj,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());

            initIntake();

    return this::driveWhileOtherCommandRuns;
  }

  private StateHandler driveWhileOtherCommandRuns() {
    if (shooting) {
    drive.resetVelocityOverride();
    return null;

    }
    var chassisSpeeds =
        choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(),             timer.get());

    if (chassisSpeeds != null) {
      // TODO: Check for note in intake.
      drive.setVelocityOverride(chassisSpeeds);
      return null;
    }

    drive.resetVelocityOverride();
    Logger.recordOutput("Autonomous/gamepieceCount", gamepieceCounter);
    if (gamepieceCounter == 3) {
      return setDone();
    }

    return null;
  }

  private void initIntake() {
        var cmd = new IntakeGamepiece(intake, endEffector, shooter);

    spawnCommand(cmd, (c) -> { this.initShoot(); return null;});

  }
  private void initShoot() {
    timer.stop();
    shooting = true;
    drive.resetVelocityOverride();
    var cmd =
        ShooterScoreSpeakerStateMachine.getAsCommand(drive, shooter, endEffector, intake, 15);

    spawnCommand(
        cmd,
        (c) -> {
          timer.start();
    shooting = false;
          gamepieceCounter++;
          if (gamepieceCounter < 3)
          this.initIntake();
          return null;
        });

  }
}
