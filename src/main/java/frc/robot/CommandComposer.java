package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team6328.GeomUtil;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.drive.AlignWithChain;
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
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class CommandComposer {
  //   public static Command autoClimb(
  //       AprilTagFieldLayout aprilTagFieldLayout,
  //       DrivetrainWrapper drivetrainWrapper,
  //       Elevator elevator,
  //       Arm arm,
  //       EndEffector endEffector,
  //       Shooter shooter,
  //       Intake intake) {
  //     /*
  //      * Step 1: drive to in front of the chain,
  //      * at the same time, if the robot is within 2 meters of the stage:
  //      * if the robot is under the stage
  //      * bring mech into under stage prep,
  //      * else
  //      * bring the mech into climb prep positon,
  //      * if we are farther than 2 meters,
  //      * stay in stow/loading position
  //      * Step 2: run climb command
  //      */

  //     DriveToPose driveToClimbPos =
  //         new DriveToPose(
  //             drivetrainWrapper,
  //             () ->
  //                 AutoClimb.getTargetPose(
  //                     aprilTagFieldLayout, drivetrainWrapper.getPoseEstimatorPose(true)),
  //             () -> drivetrainWrapper.getPoseEstimatorPose(true));

  //     BooleanSupplier withinRangeOfStage =
  //         () ->
  //             GeometryUtil.getDist(
  //                     Constants.FieldConstants.getStageCenter(),
  //                     drivetrainWrapper.getPoseEstimatorPose(false).getTranslation())
  //                 <= 2.0;

  //     BooleanSupplier underStage =
  //         () -> AutoClimb.underStage(drivetrainWrapper.getPoseEstimatorPose(false));

  //     Supplier<Command> prepForClimb =
  //         () ->
  //             new ConditionalCommand(
  //                 MechanismActions.climbPrepUnderStagePosition(elevator, arm)
  //                     .until(() -> !underStage.getAsBoolean())
  //                     .andThen(
  //                         MechanismActions.climbPrepPosition(
  //                             elevator, arm, endEffector, shooter, intake)),
  //                 MechanismActions.climbPrepPosition(elevator, arm, endEffector, shooter,
  // intake),
  //                 underStage);

  //     Command climbCommand =
  //         (driveToClimbPos.alongWith(
  //                 new ConditionalCommand(
  //                     prepForClimb.get(),
  //                     MechanismActions.loadingPosition(elevator, arm)
  //                         .until(withinRangeOfStage)
  //                         .andThen(prepForClimb.get()),
  //                     withinRangeOfStage)))
  //             .until(driveToClimbPos::atGoal)
  //             .andThen(new AutoClimb(drivetrainWrapper, elevator, arm, endEffector));

  //     climbCommand.setName("AutoClimb");
  //     return climbCommand;
  //   }

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
        new ConditionalCommand(
                driveToAmp
                    .alongWith(Commands.runOnce(() -> rumbleCommand.accept(0.5)))
                    .alongWith(new LedSetStateForSeconds(led, RobotState.AMP_READY_TO_SCORE, 1))
                    .until(() -> driveToAmp.withinTolerance(0.1, Rotation2d.fromDegrees(10.0)))
                    .finallyDo(() -> rumbleCommand.accept(0.0))
                    .andThen(Commands.waitUntil(() -> false))
                    .until(() -> confirmation.getAsBoolean()),
                Commands.waitUntil(() -> confirmation.getAsBoolean()),
                () -> doDrive)
            .asProxy()
            .alongWith(
                new ReactionArmsSetAngle(
                        elevator,
                        Constants.ElevatorConstants.ReactionArmConstants.REACTION_ARM_AMP_ROTATIONS)
                    .andThen(MechanismActions.ampPrepPosition(elevator, arm))
                    .andThen(
                        new ReactionArmsSetAngle(
                            elevator,
                            Constants.ElevatorConstants.ReactionArmConstants
                                .REACTION_ARM_HOME_ROTATIONS)))
            .deadlineWith(new EndEffectorCenterNoteBetweenToFs(endEffector, intake, shooter))
            .andThen(
                MechanismActions.ampPosition(elevator, arm)
                    .andThen(Commands.run(() -> endEffector.setVelocity(2500), endEffector))
                    .until(noGamepieceInEE))
            .alongWith(new LedSetBaseState(led, BaseRobotState.AMP_LINE_UP))
            .finallyDo(() -> rumbleCommand.accept(0.0));

    // Command scoreAmp =
    // goToAmpPosition
    //     .deadlineWith(driveToPrep)
    //     .andThen(
    //         (driveToAmp
    //             .alongWith(Commands.runOnce(() -> rumbleCommand.accept(0.5)))
    //             .alongWith(new LedSetStateForSeconds(led, RobotState.AMP_READY_TO_SCORE, 1))
    //             .until(() -> driveToAmp.withinTolerance(0.1, Rotation2d.fromDegrees(10.0)))
    //             .finallyDo(() ->
    // rumbleCommand.accept(0.0))).until(confirmation::getAsBoolean)
    //             .asProxy()
    //             .deadlineWith(
    //                 new EndEffectorCenterNoteBetweenToFs(endEffector, intake, shooter))
    //             .andThen(
    //                 MechanismActions.ampPosition(elevator, arm)
    //                     .andThen(Commands.run(() -> endEffector.setVelocity(2500),
    // endEffector))
    //                     .until(noGamepieceInEE))
    //             .alongWith(new LedSetBaseState(led, BaseRobotState.AMP_LINE_UP))
    //             .finallyDo(() -> rumbleCommand.accept(0.0)));
    // goToAmpPosition
    //     .andThen(new WaitUntilCommand(confirmation))
    //     .andThen(
    //         MechanismActions.ampPosition(elevator, arm)
    //             .andThen(Commands.run(() -> endEffector.setVelocity(2500), endEffector))
    //             .until(noGamepieceInEE)
    //             .andThen(new LedSetStateForSeconds(led, RobotState.AMP_READY_TO_SCORE, 1)))
    //     .alongWith(new LedSetBaseState(led, BaseRobotState.AMP_LINE_UP));
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

    Trigger noGamepieceInEE =
        new Trigger(
                () ->
                    !endEffector.intakeSideTOFDetectGamepiece()
                        && !endEffector.shooterSideTOFDetectGamepiece())
            .debounce(0.4);

    Command cancelScoreAmp =
        Commands.waitUntil(noGamepieceInEE::getAsBoolean)
            .andThen(
                Commands.waitUntil(
                        () ->
                            GeometryUtil.getDist(
                                    drivetrainWrapper.getPoseEstimatorPose(true),
                                    Constants.FieldConstants.getAmpScoringPose())
                                >= 0.2)
                    .andThen(
                        new ReactionArmsSetAngle(
                            elevator,
                            Constants.ElevatorConstants.ReactionArmConstants
                                .REACTION_ARM_AMP_ROTATIONS))
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
      AprilTagFieldLayout aprilTagLayout,
      DrivetrainWrapper wrapper,
      LED led,
      Consumer<Double> rumbleMagnitude) {
    Supplier<Pose2d> robotLocalization =
        () ->
            Constants.isRedAlliance()
                ? wrapper.getPoseEstimatorPoseStageRed(false)
                : wrapper.getPoseEstimatorPoseStageBlue(false);
    Supplier<Pose2d> targetPose =
        () -> AutoClimb.getTargetPose(aprilTagLayout, robotLocalization.get());
    DriveToPose driveCommand = new DriveToPose(wrapper, targetPose, robotLocalization, true, 0.02);
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

  public static Command prepMechForClimb(
      Elevator elevator,
      Arm arm,
      EndEffector endEffector,
      Shooter shooter,
      Intake intake,
      BooleanSupplier reactionArmsDown,
      Supplier<Command> toggleReactionArms) {
    return new ConditionalCommand(Commands.none(), toggleReactionArms.get(), reactionArmsDown)
        .andThen(MechanismActions.climbPrepPosition(elevator, arm, endEffector, shooter, intake));
  }

  public static Command stageAlignFast(
      AprilTagFieldLayout aprilTagLayout,
      DrivetrainWrapper wrapper,
      LED led,
      Consumer<Double> rumbleMagnitude,
      VisionGamepiece visionGamepiece,
      Elevator elevator,
      Arm arm,
      EndEffector endEffector,
      Shooter shooter,
      Intake intake,
      BooleanSupplier reactionArmsDown,
      Supplier<Command> toggleReactionArms) {

    Supplier<Pose2d> robotLocalization =
        () ->
            Constants.isRedAlliance()
                ? wrapper.getPoseEstimatorPoseStageRed(true)
                : wrapper.getPoseEstimatorPoseStageBlue(true);

    RotateToAngle rotateToAngle =
        new RotateToAngle(
            wrapper,
            () -> getStageTargetRotation(robotLocalization.get().getRotation()),
            robotLocalization);

    AlignWithChain alignWithChain =
        new AlignWithChain(visionGamepiece, wrapper, rotateToAngle::getError);

    Command stageAlign =
        rotateToAngle
            .alongWith(alignWithChain)
            .alongWith(
                Commands.run(
                    () -> {
                      if (!alignWithChain.driving()) {
                        led.setBaseRobotState(BaseRobotState.CLIMB_ALIGN_STAGE1);
                      } else if (Math.abs(visionGamepiece.getTagYaw()) > 1) {
                        led.setBaseRobotState(BaseRobotState.CLIMB_ALIGN_STAGE2);
                      } else {
                        led.setBaseRobotState(BaseRobotState.CLIMB_ALIGN_STAGE3);
                      }
                    }))
            .alongWith(
                prepMechForClimb(
                    elevator,
                    arm,
                    endEffector,
                    shooter,
                    intake,
                    reactionArmsDown,
                    toggleReactionArms))
            .finallyDo(
                () -> {
                  rumbleMagnitude.accept(0.0);
                  led.setBaseRobotState(BaseRobotState.NOTE_STATUS);
                });
    stageAlign.setName("stageAlign");
    return stageAlign;
  }

  public static Rotation2d getStageTargetRotation(Rotation2d currentRotation) {
    double currentRot = GeometryUtil.optimizeRotationInDegrees(currentRotation.getDegrees());
    Double closestRot = null;

    for (int i = -1; i < 2; i++) {
      double subjectRot = 120.0 * i;

      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
          subjectRot -= 60.0;
        }
      }

      subjectRot = GeometryUtil.optimizeRotationInDegrees(subjectRot);

      if (closestRot == null) {
        closestRot = subjectRot;
      } else {
        if (getDistWithWrapping(currentRot, closestRot)
            > getDistWithWrapping(currentRot, subjectRot)) {
          closestRot = subjectRot;
        }
      }
    }

    Logger.recordOutput("StageAlignFast/closestRot", closestRot);
    return Rotation2d.fromDegrees(closestRot);
  }

  public static double getDistWithWrapping(double rotation1, double rotation2) {
    double errorBound = (180 - (-180)) / 2.0;
    return Math.abs(MathUtil.inputModulus(rotation1 - rotation2, -errorBound, errorBound));
  }

  private static final LoggedTunableNumber stageApproachSpeed =
      new LoggedTunableNumber("DriveToChain/stageApproachSpeed", 1.5);

  public static Command driveToChain(
      AprilTagFieldLayout aprilTagFieldLayout,
      DrivetrainWrapper wrapper,
      LED led,
      VisionGamepiece visionGamepiece) {
    Supplier<Pose2d> robotLocalization =
        () ->
            Constants.isRedAlliance()
                ? wrapper.getPoseEstimatorPoseStageRed(true)
                : wrapper.getPoseEstimatorPoseStageBlue(true);

    PIDController yOffsetController = new PIDController(0.05, 0, 0);

    RotateToAngle rotateToAngle =
        new RotateToAngle(
            wrapper,
            () -> getStageTargetRotation(robotLocalization.get().getRotation()),
            robotLocalization);

    Command stageApproach =
        Commands.run(
                () ->
                    wrapper.setVelocityOverride(
                        new ChassisSpeeds(
                            rotateToAngle.withinTolerance() ? stageApproachSpeed.get() : 0.0,
                            visionGamepiece.seesStageTags()
                                ? yOffsetController.calculate(visionGamepiece.getTagYaw(), 0)
                                : 0.0,
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

  //   public static Command driveToChainFast(
  //       AprilTagFieldLayout aprilTagFieldLayout,
  //       DrivetrainWrapper wrapper,
  //       LED led,
  //       VisionGamepiece visionGamepiece) {
  //     Supplier<Pose2d> robotLocalization =
  //         () ->
  //             Constants.isRedAlliance()
  //                 ? wrapper.getPoseEstimatorPoseStageRed(false)
  //                 : wrapper.getPoseEstimatorPoseStageBlue(false);
  //     RotateToAngle rotateToAngle =
  //         new RotateToAngle(
  //             wrapper,
  //             () ->
  //                 AutoClimb.getTargetPose(aprilTagFieldLayout, robotLocalization.get())
  //                     .getRotation()
  //                     .plus(Rotation2d.fromDegrees(180)),
  //             robotLocalization);

  //     Command stageApproach =
  //         new DriveToChain(visionGamepiece, wrapper, rotateToAngle::withinTolerance)
  //             .alongWith(rotateToAngle)
  //             .finallyDo(
  //                 () -> {
  //                   wrapper.resetVelocityOverride();
  //                   wrapper.resetRotationOverride();
  //                   led.setBaseRobotState(BaseRobotState.NOTE_STATUS);
  //                 })
  //             .alongWith(new LedSetBaseState(led, BaseRobotState.CLIMB_LINE_UP));

  //     stageApproach.setName("stageApproach");

  //     return stageApproach;
  //   }

  public static final Command autoClimb(
      Elevator elevator, Arm arm, EndEffector endEffector, Shooter shooter, Intake intake) {
    return MechanismActions.climbDownPosition(elevator, arm)
        .andThen(Commands.waitSeconds(0.2))
        // .deadlineWith(new EndEffectorPrepareNoteForTrap(endEffector))
        .andThen(
            new ConditionalCommand(
                    MechanismActions.climbTrapPosition(elevator, arm)
                    // .andThen(
                    //     // Commands.runOnce(() -> endEffector.setVelocity(2500), endEffector)
                    //     //     .until(() -> !endEffector.noteInEndEffector())
                    //     //     .andThen(Commands.waitSeconds(0.5))
                    //         // .deadlineWith(new EndEffectorPrepareNoteForTrap(endEffector))
                    //         // .andThen(
                    //         //     Commands.runOnce(
                    //         //         () -> endEffector.setPercentOut(0.0), endEffector))
                    //         // .andThen(MechanismActions.climbFinalRestPosition(elevator, arm))
                    //         )
                    ,
                    Commands.none(),
                    () ->
                        endEffector.intakeSideTOFDetectGamepiece()
                            || endEffector.shooterSideTOFDetectGamepiece())
                .asProxy())
        .finallyDo(() -> endEffector.setPercentOut(0.0));
  }
}
