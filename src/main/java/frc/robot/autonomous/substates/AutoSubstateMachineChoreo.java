package frc.robot.autonomous.substates;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autonomous.AutosSubsystems;
import frc.robot.autonomous.ChoreoHelper;
import frc.robot.autonomous.ChoreoTrajectoryWithName;
import frc.robot.autonomous.DriveToGamepieceHelper;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public class AutoSubstateMachineChoreo extends AutoSubstateMachine {
  private final ChoreoTrajectoryWithName trajToGamepiece;

  /** Creates a new AutoSubstateMachine. */
  public AutoSubstateMachineChoreo(
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean useVision,
      ChoreoTrajectoryWithName trajToGamepiece,
      ChoreoTrajectoryWithName trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece,
      Translation2d gamepieceTranslation) {
    super(
        "AutoSub "
            + ChoreoTrajectoryWithName.getName(trajToGamepiece)
            + " # "
            + ChoreoTrajectoryWithName.getName(trajToShoot),
        subsystems,
        config,
        useVision,
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
      var traj = trajToGamepiece.rescale(slowDownFactor.get());
      choreoHelper =
          new ChoreoHelper(
              timeFromStart(),
              drive.getPoseEstimatorPose(true),
              traj,
              config.getDriveBaseRadius() / 2,
              minVelToPause.get(),
              config.getAutoTranslationPidController(),
              config.getAutoTranslationPidController(),
              config.getAutoThetaPidController());
    }

    driveToGamepieceHelper = new DriveToGamepieceHelper(led);

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
      return stateWithName("gamepieceConfirmation", super::gamepieceConfirmation);
    }
    drive.setVelocityOverride(speeds);
    return null;
  }
}
