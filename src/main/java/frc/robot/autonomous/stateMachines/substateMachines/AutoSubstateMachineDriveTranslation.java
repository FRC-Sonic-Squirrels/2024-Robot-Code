package frc.robot.autonomous.stateMachines.substateMachines;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.team2930.AllianceFlipUtil;
import frc.robot.autonomous.helpers.DriveToGamepieceHelper;
import frc.robot.autonomous.records.AutosSubsystems;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.autonomous.records.TargetGP;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public class AutoSubstateMachineDriveTranslation extends AutoSubstateMachine {
  private final TargetGP targetGPInfo;

  // private final RotateToAngle rotateToAngle;

  /** Creates a new AutoSubstateMachineDriveTranslation. */
  public AutoSubstateMachineDriveTranslation(
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean useVision,
      boolean ploppedGamepeice,
      TargetGP targetGPInfo,
      ChoreoTrajectoryWithName trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    super(
        "AutoSub " + ChoreoTrajectoryWithName.getName(trajToShoot),
        subsystems,
        config,
        useVision,
        ploppedGamepeice,
        trajToShoot,
        closestGamepiece,
        AllianceFlipUtil.flipTranslationForAlliance(targetGPInfo.targetGP()));

    this.targetGPInfo = targetGPInfo;

    setInitialState(stateWithName("initDriveToGamepiece", this::initDriveToGamepiece));
  }

  private StateHandler initDriveToGamepiece() {
    intakeCommand = new IntakeGamepiece(intake, endEffector, shooter, arm, elevator);
    intakeCommand.schedule();

    driveToGamepieceHelper =
        new DriveToGamepieceHelper(
            drive.getPoseEstimatorPose(true), drive.getFieldRelativeVelocities());

    return targetGPInfo.prepForTargetGP() == null
        ? stateWithName("pickupGamepiece", this::pickupGamepiece)
        : stateWithName("goToPrepPose", this::goToPrepPose);
  }

  private StateHandler goToPrepPose() {
    led.setBaseRobotState(BaseRobotState.AUTO_DRIVE_TO_PREP_POSE);

    AllianceFlipUtil.flipPoseForAlliance(targetGPInfo.prepForTargetGP());

    spawnCommand(
        new DriveToPose(
                drive,
                () -> AllianceFlipUtil.flipPoseForAlliance(targetGPInfo.prepForTargetGP()),
                () -> drive.getPoseEstimatorPose(true),
                0.1)
            .until(() -> useVisionForGamepiece() || endEffector.noteInEndEffector()),
        (c) -> {
          drive.resetVelocityOverride();
          driveToGamepieceHelper =
              new DriveToGamepieceHelper(
                  drive.getPoseEstimatorPose(true), drive.getFieldRelativeVelocities());

          return stateWithName("pickupGamepiece", this::pickupGamepiece);
        });

    return stateWithName("waitWhileDrivingToPrepPose", this::waitWhileDrivingToPrepPose);
  }

  private StateHandler waitWhileDrivingToPrepPose() {
    return null;
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
      if (driveToGamepieceHelper.isAtTarget()) {
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
