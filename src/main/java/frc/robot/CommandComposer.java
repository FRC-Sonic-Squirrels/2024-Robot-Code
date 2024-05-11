package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.commands.drive.AlignWithChain;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.endEffector.EndEffectorCenterNoteBetweenToFs;
import frc.robot.commands.endEffector.EndEffectorPercentOut;
import frc.robot.commands.led.LedSetBaseState;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.commands.mechanism.elevator.ReactionArmsSetAngle;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
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

    Command scoreAmp =
        Commands.waitUntil(() -> confirmation.getAsBoolean())
            .deadlineWith(
                new ReactionArmsSetAngle(
                        elevator,
                        Constants.ElevatorConstants.ReactionArmConstants.REACTION_ARM_AMP_ROTATIONS)
                    .andThen(MechanismActions.ampPrepPosition(elevator, arm))
                    .andThen(
                        new ReactionArmsSetAngle(
                            elevator,
                            Constants.ElevatorConstants.ReactionArmConstants
                                .REACTION_ARM_HOME_ROTATIONS))
                    .deadlineWith(
                        new EndEffectorCenterNoteBetweenToFs(endEffector, intake, shooter)))
            .andThen(
                MechanismActions.ampPosition(elevator, arm)
                    .andThen(Commands.run(() -> endEffector.setVelocity(2500), endEffector))
                    .until(noGamepieceInEE))
            .alongWith(new LedSetBaseState(led, BaseRobotState.AMP_LINE_UP))
            .finallyDo(() -> rumbleCommand.accept(0.0));

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

  private static final LoggerGroup logGroupAutoClimb = LoggerGroup.build("AutoClimb");

  private static final LoggedTunableNumber distFromStage =
      new LoggedTunableNumber("StageAlign/distFromStage", 1.738);

  private static final LoggerEntry.Struct<Pose2d> log_closestPose =
      logGroupAutoClimb.buildStruct(Pose2d.class, "closestPose");

  public static Pose2d getTargetPose(AprilTagFieldLayout aprilTagFieldLayout, Pose2d robotPose) {
    int[] tags;

    if (Constants.isRedAlliance()) {
      tags = new int[] {11, 12, 13};
    } else {
      tags = new int[] {14, 15, 16};
    }

    var offset = new Translation2d(distFromStage.get(), Constants.zeroRotation2d);
    var offset_transform = new Transform2d(offset, Constants.zeroRotation2d);

    Pose2d closestPose = null;
    for (int tag : tags) {
      var tagPose = aprilTagFieldLayout.getTagPose(tag);
      if (tagPose.isEmpty()) continue;

      var tagPose2d = tagPose.get().toPose2d();
      var tagPose2d_offset = tagPose2d.transformBy(offset_transform);
      if (closestPose == null) {
        closestPose = tagPose2d_offset;
      } else {
        if (GeometryUtil.getDist(tagPose2d_offset, robotPose)
            < GeometryUtil.getDist(closestPose, robotPose)) {
          closestPose = tagPose2d_offset;
        }
      }
    }

    log_closestPose.info(closestPose);
    return closestPose;
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
    Supplier<Pose2d> targetPose = () -> getTargetPose(aprilTagLayout, robotLocalization.get());
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
                      } else if (Math.abs(visionGamepiece.getTagYaw()) > 1
                          || !visionGamepiece.isValidTarget()) {
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
                            stageApproachSpeed.get(),
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
                  yOffsetController.close();
                })
            .alongWith(new LedSetBaseState(led, BaseRobotState.CLIMB_LINE_UP));

    stageApproach.setName("stageApproach");

    return stageApproach;
  }

  public static final Command autoClimb(
      Elevator elevator, Arm arm, EndEffector endEffector, Shooter shooter, Intake intake) {
    return MechanismActions.climbDownPosition(elevator, arm)
        .andThen(Commands.waitSeconds(0.2))
        .andThen(
            new ConditionalCommand(
                    MechanismActions.climbTrapPosition(elevator, arm),
                    Commands.none(),
                    () ->
                        endEffector.intakeSideTOFDetectGamepiece()
                            || endEffector.shooterSideTOFDetectGamepiece())
                .asProxy())
        .finallyDo(() -> endEffector.setPercentOut(0.0));
  }
}
