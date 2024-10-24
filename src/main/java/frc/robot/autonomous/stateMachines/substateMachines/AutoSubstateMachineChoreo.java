package frc.robot.autonomous.stateMachines.substateMachines;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.helpers.DriveToGamepieceHelper;
import frc.robot.autonomous.records.AutosSubsystems;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
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
      boolean doDistanceCheck,
      boolean waitForGamepieceVision,
      boolean nextSubstatePlop,
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
        doDistanceCheck,
        waitForGamepieceVision,
        nextSubstatePlop,
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

    driveToGamepieceHelper =
        new DriveToGamepieceHelper(
            drive.getPoseEstimatorPose(true), drive.getFieldRelativeVelocities());

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

    if (choreoHelper != null) {
      var result =
          choreoHelper.calculateChassisSpeeds(drive.getPoseEstimatorPose(true), timeFromStart());
      if (!result.atEndOfPath()) {
        drive.setVelocityOverride(result.chassisSpeeds());
        return null;
      }
    }

    if (waitForGamepieceVision) {
      return null;
    }

    drive.resetVelocityOverride();
    return stateWithName("gamepieceConfirmation", super::gamepieceConfirmation);
  }
}
