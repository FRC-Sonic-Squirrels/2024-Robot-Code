package frc.robot.autonomous.substates;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

public class AutoSubstateMachineDriveTranslation extends AutoSubstateMachine {
  private Translation2d gamepieceTranslation;

  /** Creates a new AutoSubstateMachineDriveTranslation. */
  public AutoSubstateMachineDriveTranslation(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      Elevator elevator,
      Arm arm,
      boolean useVision,
      Translation2d gamepieceTranslation,
      ChoreoTrajectory trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    super(
        drive,
        shooter,
        endEffector,
        intake,
        config,
        elevator,
        arm,
        useVision,
        trajToShoot,
        closestGamepiece,
        gamepieceTranslation);

    this.gamepieceTranslation = gamepieceTranslation;

    setInitialState(stateWithName("initFollowPathToGamePiece", this::initFollowPathToGamePiece));
  }

  private StateHandler initFollowPathToGamePiece() {
    intakeCommand = new IntakeGamepiece(intake, endEffector, shooter, arm, elevator);
    intakeCommand.schedule();

    driveToGamepieceHelper = new DriveToGamepieceHelper();

    return stateWithName("followPathToGamePiece", this::pickupGamepiece);
  }

  private StateHandler pickupGamepiece() {
    ChassisSpeeds speeds =
        driveToGamepieceHelper.calculateChassisSpeeds(
            gamepieceTranslation, drive.getPoseEstimatorPose(true));

    if (useVisionForGamepiece()) {
      return stateWithName("visionPickupGamepiece", super::visionPickupGamepiece);
    }

    if (!endEffector.noteInEndEffector()) {
      drive.setVelocityOverride(speeds);
      if (driveToGamepieceHelper.isAtTarget()) {
        drive.resetVelocityOverride();
        return setStopped();
      }
      return null;
    }
    drive.resetVelocityOverride();
    return stateWithName("prepFollowPathToShooting", super::prepFollowPathToShooting);
  }
}
