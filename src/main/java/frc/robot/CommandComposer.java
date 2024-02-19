package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.GeometryUtil;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.endEffector.EndEffectorPercentOut;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CommandComposer {
  public static Command autoClimb(
      DrivetrainWrapper drivetrainWrapper, Elevator elevator, Arm arm, EndEffector endEffector) {
    AutoClimb autoClimb = new AutoClimb(drivetrainWrapper, elevator, arm, endEffector);

    DriveToPose driveToClimbPos =
        new DriveToPose(
            drivetrainWrapper,
            () -> autoClimb.getTargetPose(drivetrainWrapper.getPoseEstimatorPose()));

    BooleanSupplier withinRangeOfStage =
        () ->
            GeometryUtil.getDist(
                    Constants.FieldConstants.getStageCenter(),
                    drivetrainWrapper.getPoseEstimatorPose().getTranslation())
                <= 2.0;

    BooleanSupplier underStage =
        () -> autoClimb.underStage(drivetrainWrapper.getPoseEstimatorPose());

    Supplier<Command> prepForClimb =
        () ->
            new ConditionalCommand(
                MechanismActions.climbPrepUnderStagePosition(elevator, arm)
                    .until(() -> !underStage.getAsBoolean())
                    .andThen(MechanismActions.climbPrepPosition(elevator, arm)),
                MechanismActions.climbPrepPosition(elevator, arm),
                underStage);

    Command climbCommand =
        (driveToClimbPos.alongWith(
                new ConditionalCommand(
                    prepForClimb.get(),
                    MechanismActions.loadingPosition(elevator, arm)
                        .until(withinRangeOfStage)
                        .andThen(prepForClimb.get()),
                    withinRangeOfStage)))
            .until(driveToClimbPos::atGoal)
            .andThen(autoClimb);

    climbCommand.setName("AutoClimb");
    return climbCommand;
  }

  public static Command scoreAmp(
      EndEffector endEffector, DrivetrainWrapper drivetrainWrapper, Elevator elevator, Arm arm) {
    Trigger gamepieceInEE =
        new Trigger(
                () ->
                    !endEffector.intakeSideTOFDetectGamepiece()
                        && !endEffector.shooterSideTOFDetectGamepiece())
            .debounce(0.4);

    DriveToPose driveToAmp =
        new DriveToPose(
            drivetrainWrapper,
            () ->
                new Pose2d(
                    Constants.isRedAlliance() ? 14.714638710021973 : 1.8273155689239502,
                    7.65,
                    Rotation2d.fromDegrees(90.0)));

    Command scoreAmp =
        driveToAmp
            .until(driveToAmp::atGoal)
            .alongWith(MechanismActions.ampPosition(elevator, arm))
            .andThen(new EndEffectorPercentOut(endEffector, 0.8).until(gamepieceInEE));

    scoreAmp.setName("ScoreAmp");

    return scoreAmp;
  }

  public static Command cancelScoreAmp(
      DrivetrainWrapper drivetrainWrapper, Elevator elevator, Arm arm) {
    Command cancelScoreAmp =
        Commands.waitUntil(() -> drivetrainWrapper.getPoseEstimatorPose().getY() <= 7.4)
            .andThen(MechanismActions.loadingPosition(elevator, arm));

    cancelScoreAmp.setName("LoadPosition");
    return cancelScoreAmp;
  }
}
