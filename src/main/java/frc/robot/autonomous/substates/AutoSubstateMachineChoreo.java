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

public class AutoSubstateMachineChoreo extends AutoSubstateMachine {
  private final ChoreoTrajectory trajToGamepiece;

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

    if (trajToGamepiece != null) {
      choreoHelper =
          new ChoreoHelper(
              timeFromStart(),
              drive.getPoseEstimatorPose(true),
              trajToGamepiece,
              config.getAutoTranslationPidController(),
              config.getAutoTranslationPidController(),
              config.getAutoThetaPidController());
    }

    driveToGamepieceHelper = new DriveToGamepieceHelper();

    return stateWithName("followPathToGamePiece", this::pickupGamepiece);
  }

  private StateHandler pickupGamepiece() {
    if (useVisionForGamepiece()) {
      return stateWithName("visionPickupGamepiece", super::visionPickupGamepiece);
    }

    if (endEffector.noteInEndEffector()) {
      drive.resetVelocityOverride();
      return stateWithName("prepFollowPathToShooting", super::prepFollowPathToShooting);
    }

    ChassisSpeeds speeds =
        choreoHelper != null
            ? choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(true), timeFromStart())
            : null;
    if (speeds == null) {
      drive.resetVelocityOverride();
      return setStopped();
    }
    drive.setVelocityOverride(speeds);
    return null;
  }
}
