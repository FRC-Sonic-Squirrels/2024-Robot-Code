package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team6328.GeomUtil;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.endEffector.EndEffectorCenterNoteBetweenToFs;
import frc.robot.commands.endEffector.EndEffectorPercentOut;
import frc.robot.commands.led.LedSetBaseState;
import frc.robot.commands.led.LedSetStateForSeconds;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.commands.mechanism.elevator.ReactionArmsSetAngle;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.LED.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class CommandComposer {
  public static Command autoClimb(
      DrivetrainWrapper drivetrainWrapper,
      Elevator elevator,
      Arm arm,
      EndEffector endEffector,
      Shooter shooter,
      Intake intake) {
    /*
     * Step 1: drive to in front of the chain,
     * at the same time, if the robot is within 2 meters of the stage:
     * if the robot is under the stage
     * bring mech into under stage prep,
     * else
     * bring the mech into climb prep positon,
     * if we are farther than 2 meters,
     * stay in stow/loading position
     * Step 2: run climb command
     */

    DriveToPose driveToClimbPos =
        new DriveToPose(
            drivetrainWrapper,
            () -> AutoClimb.getTargetPose(drivetrainWrapper.getPoseEstimatorPose(true)),
            () -> drivetrainWrapper.getPoseEstimatorPose(true));

    BooleanSupplier withinRangeOfStage =
        () ->
            GeometryUtil.getDist(
                    Constants.FieldConstants.getStageCenter(),
                    drivetrainWrapper.getPoseEstimatorPose(false).getTranslation())
                <= 2.0;

    BooleanSupplier underStage =
        () -> AutoClimb.underStage(drivetrainWrapper.getPoseEstimatorPose(false));

    Supplier<Command> prepForClimb =
        () ->
            new ConditionalCommand(
                MechanismActions.climbPrepUnderStagePosition(elevator, arm)
                    .until(() -> !underStage.getAsBoolean())
                    .andThen(
                        MechanismActions.climbPrepPosition(
                            elevator, arm, endEffector, shooter, intake)),
                MechanismActions.climbPrepPosition(elevator, arm, endEffector, shooter, intake),
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
            .andThen(new AutoClimb(drivetrainWrapper, elevator, arm, endEffector));

    climbCommand.setName("AutoClimb");
    return climbCommand;
  }

  public static Command scoreAmp(
      EndEffector endEffector,
      DrivetrainWrapper drivetrainWrapper,
      Elevator elevator,
      Arm arm,
      Intake intake,
      Shooter shooter,
      LED led,
      boolean doDrive,
      Trigger confirmation,
      Consumer<Double> rumbleCommand) {
    /*
     * Step 1: drive to amp
     * at the same time, if we are within a distance move mech into position
     * Step 2: run end effector
     */
    Trigger noGamepieceInEE =
        new Trigger(
                () ->
                    !endEffector.intakeSideTOFDetectGamepiece()
                        && !endEffector.shooterSideTOFDetectGamepiece())
            .debounce(0.4);

    DriveToPose driveToAmp =
        new DriveToPose(
            drivetrainWrapper,
            Constants.FieldConstants::getAmpScoringPose,
            () -> drivetrainWrapper.getPoseEstimatorPose(true));

    DriveToPose driveToPrep =
        new DriveToPose(
            drivetrainWrapper,
            () ->
                GeomUtil.transformToPose(
                    Constants.FieldConstants.getAmpScoringPose()
                        .minus(new Pose2d(0.0, 0.5, new Rotation2d()))),
            () -> drivetrainWrapper.getPoseEstimatorPose(true));

    Measure<Distance> distToElevateMech = Units.Meters.of(3.5);

    BooleanSupplier withinRangeOfAmp =
        () ->
            GeometryUtil.getDist(
                    drivetrainWrapper.getPoseEstimatorPose(false),
                    Constants.FieldConstants.getAmpScoringPose())
                <= distToElevateMech.in(Units.Meters);

    Command goToAmpPosition =
        new ReactionArmsSetAngle(
                elevator,
                Constants.ElevatorConstants.ReactionArmConstants.REACTION_ARM_AMP_ROTATIONS)
            .andThen(MechanismActions.ampPrepPosition(elevator, arm))
            .andThen(
                new ReactionArmsSetAngle(
                    elevator,
                    Constants.ElevatorConstants.ReactionArmConstants.REACTION_ARM_HOME_ROTATIONS));

    Command scoreAmp =
        goToAmpPosition
            .deadlineWith(driveToPrep)
            .andThen(
                driveToAmp
                    .alongWith(Commands.runOnce(() -> rumbleCommand.accept(0.5)))
                    .alongWith(new LedSetStateForSeconds(led, RobotState.AMP_READY_TO_SCORE, 1))
                    .until(() -> driveToAmp.withinTolerance(0.1, Rotation2d.fromDegrees(10.0)))
                    .finallyDo(() -> rumbleCommand.accept(0.0))
                    .andThen(Commands.waitUntil(confirmation::getAsBoolean))
                    .asProxy()
                    .deadlineWith(
                        new EndEffectorCenterNoteBetweenToFs(endEffector, intake, shooter))
                    .andThen(
                        MechanismActions.ampPosition(elevator, arm)
                            .andThen(Commands.run(() -> endEffector.setVelocity(2500), endEffector))
                            .until(noGamepieceInEE))
                    .alongWith(new LedSetBaseState(led, BaseRobotState.AMP_LINE_UP))
                    .finallyDo(() -> rumbleCommand.accept(0.0)));
    // .andThen(cancelScoreAmp(drivetrainWrapper, endEffector, elevator, arm));

    scoreAmp.setName("ScoreAmp");

    return scoreAmp;
  }

  public static Command cancelScoreAmp(
      DrivetrainWrapper drivetrainWrapper,
      EndEffector endEffector,
      Elevator elevator,
      Arm arm,
      LED led) {
    Command cancelScoreAmp =
        Commands.waitUntil(
                () ->
                    GeometryUtil.getDist(
                            drivetrainWrapper.getPoseEstimatorPose(true),
                            Constants.FieldConstants.getAmpScoringPose())
                        >= 0.5)
            .andThen(
                new ReactionArmsSetAngle(
                        elevator,
                        Constants.ElevatorConstants.ReactionArmConstants.REACTION_ARM_AMP_ROTATIONS)
                    .andThen(MechanismActions.loadingPosition(elevator, arm))
                    .andThen(
                        new ReactionArmsSetAngle(
                            elevator,
                            Constants.ElevatorConstants.ReactionArmConstants
                                .REACTION_ARM_HOME_ROTATIONS))
                    .alongWith(new EndEffectorPercentOut(endEffector, 0.0))
                    .alongWith(new LedSetBaseState(led, BaseRobotState.NOTE_STATUS)));

    cancelScoreAmp.setName("CancelScoreAmp");
    return cancelScoreAmp;
  }

  public static Command stageAlign(
      DrivetrainWrapper wrapper, LED led, Consumer<Double> rumbleMagnitude) {
    Supplier<Pose2d> robotLocalization =
        () ->
            Constants.isRedAlliance()
                ? wrapper.getPoseEstimatorPoseStageRed(false)
                : wrapper.getPoseEstimatorPoseStageBlue(false);
    Supplier<Pose2d> targetPose = () -> AutoClimb.getTargetPose(robotLocalization.get());
    DriveToPose driveCommand = new DriveToPose(wrapper, targetPose, robotLocalization, true);
    Command stageAlign =
        driveCommand
            .alongWith(
                Commands.run(
                    () ->
                        rumbleMagnitude.accept(
                            driveCommand.withinTolerance(0.05, new Rotation2d(0.05)) ? 0.5 : 0.0)))
            .finallyDo(
                () -> {
                  rumbleMagnitude.accept(0.0);
                  led.setBaseRobotState(BaseRobotState.NOTE_STATUS);
                })
            .alongWith(new LedSetBaseState(led, BaseRobotState.CLIMB_LINE_UP));
    stageAlign.setName("stageAlign");
    return stageAlign;
  }

  private static LoggedTunableNumber stageApproachSpeed =
      new LoggedTunableNumber("driveToChain/stageApproachSpeed", 0.5);

  public static Command driveToChain(DrivetrainWrapper wrapper, LED led) {
    Supplier<Pose2d> robotLocalization =
        () ->
            Constants.isRedAlliance()
                ? wrapper.getPoseEstimatorPoseStageRed(false)
                : wrapper.getPoseEstimatorPoseStageBlue(false);
    RotateToAngle rotateToAngle =
        new RotateToAngle(
            wrapper,
            () ->
                AutoClimb.getTargetPose(robotLocalization.get())
                    .getRotation()
                    .plus(Rotation2d.fromDegrees(180)),
            robotLocalization);

    Command stageApproach =
        Commands.run(
                () ->
                    wrapper.setVelocityOverride(
                        new ChassisSpeeds(
                            rotateToAngle.withinTolerance() ? stageApproachSpeed.get() : 0.0,
                            0.0,
                            0.0)))
            .alongWith(rotateToAngle)
            .finallyDo(
                () -> {
                  wrapper.resetVelocityOverride();
                  wrapper.resetRotationOverride();
                  led.setBaseRobotState(BaseRobotState.NOTE_STATUS);
                })
            .alongWith(new LedSetBaseState(led, BaseRobotState.CLIMB_LINE_UP));

    stageApproach.setName("stageApproach");

    return stageApproach;
  }
}
