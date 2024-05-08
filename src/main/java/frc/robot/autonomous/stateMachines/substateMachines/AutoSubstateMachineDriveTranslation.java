package frc.robot.autonomous.stateMachines.substateMachines;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autonomous.helpers.DriveToGamepieceHelper;
import frc.robot.autonomous.records.AutosSubsystems;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public class AutoSubstateMachineDriveTranslation extends AutoSubstateMachine {
  private final Translation2d gamepieceTranslation;

  // private final RotateToAngle rotateToAngle;

  /** Creates a new AutoSubstateMachineDriveTranslation. */
  public AutoSubstateMachineDriveTranslation(
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean useVision,
      boolean doDistanceCheck,
      boolean waitForGamepieceVision,
      boolean nextSubstatePlop,
      Translation2d gamepieceTranslation,
      ChoreoTrajectoryWithName trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    super(
        "AutoSub " + ChoreoTrajectoryWithName.getName(trajToShoot),
        subsystems,
        config,
        useVision,
        doDistanceCheck,
        nextSubstatePlop,
        waitForGamepieceVision,
        trajToShoot,
        closestGamepiece,
        gamepieceTranslation);

    this.gamepieceTranslation = gamepieceTranslation;
    // this.rotateToAngle = new RotateToAngle(drive,
    // DriverStation.getAlliance().get().equals(Alliance.Blue), drive.getPoseEstimatorPose(true))

    setInitialState(stateWithName("initDriveToGamepiece", this::initDriveToGamepiece));
  }

  private StateHandler initDriveToGamepiece() {
    intakeCommand = new IntakeGamepiece(intake, endEffector, shooter, arm, elevator);
    intakeCommand.schedule();

    driveToGamepieceHelper =
        new DriveToGamepieceHelper(
            drive.getPoseEstimatorPose(true), drive.getFieldRelativeVelocities());

    return stateWithName("pickupGamepiece", this::pickupGamepiece);
  }

  private StateHandler pickupGamepiece() {
    led.setBaseRobotState(BaseRobotState.AUTO_DRIVE_TO_POSE);
    ChassisSpeeds speeds =
        driveToGamepieceHelper.calculateChassisSpeeds(
            gamepieceTranslation, drive.getPoseEstimatorPose(true));

    if (useVisionForGamepiece()) {
      led.setBaseRobotState(BaseRobotState.NOTE_STATUS);
      return stateWithName("visionPickupGamepiece", super::visionPickupGamepiece);
    }

    if (!endEffector.noteInEndEffector()) {
      drive.setVelocityOverride(speeds);
      if (driveToGamepieceHelper.isAtTarget() && !super.waitForGamepieceVision) {
        drive.resetVelocityOverride();
        led.setBaseRobotState(BaseRobotState.NOTE_STATUS);
        return super::gamepieceConfirmation;
      }
      return null;
    }
    drive.resetVelocityOverride();
    led.setBaseRobotState(BaseRobotState.NOTE_STATUS);
    return stateWithName("prepFollowPathToShooting", super::prepFollowPathToShooting);
  }

  // private StateHandler rotateTowardOpponentWall(){

  // }
}
