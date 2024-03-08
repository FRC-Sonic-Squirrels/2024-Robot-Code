package frc.robot.autonomous.substates;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.autonomous.DriveToGamepieceHelper;
import frc.robot.commands.intake.IntakeGamepiece;
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

public class AutoSubstateMachineChoreo extends AutoSubstateMachine {
  private ChoreoTrajectory trajToGamepiece;

  /** Creates a new AutoSubstateMachine. */
  public AutoSubstateMachineChoreo(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      Elevator elevator,
      Arm arm,
      ChoreoTrajectory trajToGamepiece,
      ChoreoTrajectory trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece,
      Translation2d gamepieceTranslation) {
    super(
        drive,
        shooter,
        endEffector,
        intake,
        config,
        elevator,
        arm,
        trajToShoot,
        closestGamepiece,
        gamepieceTranslation);

    this.trajToGamepiece = trajToGamepiece;

    setInitialState(stateWithName("initFollowPathToGamePiece", this::initFollowPathToGamePiece));
  }

  private StateHandler initFollowPathToGamePiece() {
    super.intakeCommand = new IntakeGamepiece(intake, endEffector, shooter, arm, elevator);
    super.intakeCommand.schedule();
    choreoHelper =
        new ChoreoHelper(
            timeFromStart(),
            drive.getPoseEstimatorPose(),
            trajToGamepiece,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());

    driveToGamepieceHelper = new DriveToGamepieceHelper();

    return stateWithName("followPathToGamePiece", this::pickupGamepiece);
  }

  private StateHandler pickupGamepiece() {
    ChassisSpeeds speeds;
    if (useVisionForGamepiece()) {
      speeds =
          driveToGamepieceHelper.calculateChassisSpeeds(
              closestGamepiece.get().globalPose.getTranslation(), drive.getPoseEstimatorPose());
    } else {
      speeds = choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(), timeFromStart());
    }

    if (!robotStopped.getAsBoolean()) {
      startedMoving = true;
    }

    Logger.recordOutput("Autonomous/robotStopped", robotStopped.getAsBoolean());
    Logger.recordOutput("Autonomous/noteInEndEffector", endEffector.noteInEndEffector());
    if (!endEffector.noteInEndEffector()) {
      drive.setVelocityOverride(speeds);
      if (robotStopped.getAsBoolean() && startedMoving) {
        drive.resetVelocityOverride();
        return setStopped();
      }
      return null;
    }
    return stateWithName("prepFollowPathToShooting", super::prepFollowPathToShooting);
  }
}
