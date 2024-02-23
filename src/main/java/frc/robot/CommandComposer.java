package frc.robot;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
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
    /*
     * Step 1: drive to in front of the chain,
     * at the same time, if the robot is within 2 meters of the stage:
     *  if the robot is under the stage
     *      bring mech into under stage prep,
     *  else
     *       bring the mech into climb prep positon,
     * if we are farther than 2 meters,
     *  stay in stow/loading position
     * Step 2: run climb command
     */
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
        new DriveToPose(drivetrainWrapper, Constants.FieldConstants::getAmpScoringPose);

    Measure<Distance> distToElevateMech = Units.Meters.of(3.5);

    BooleanSupplier withinRangeOfAmp =
        () ->
            GeometryUtil.getDist(
                    drivetrainWrapper.getPoseEstimatorPose(),
                    Constants.FieldConstants.getAmpScoringPose())
                <= distToElevateMech.in(Units.Meters);

    Command scoreAmp =
        driveToAmp
            .until(driveToAmp::atGoal)
            .alongWith(
                new ConditionalCommand(
                    MechanismActions.ampPosition(elevator, arm),
                    MechanismActions.loadingPosition(elevator, arm)
                        .andThen(Commands.waitSeconds(1000.0))
                        .until(withinRangeOfAmp)
                        .andThen(MechanismActions.ampPosition(elevator, arm)),
                    withinRangeOfAmp))
            .andThen(new EndEffectorPercentOut(endEffector, 0.8).until(noGamepieceInEE));

    scoreAmp.setName("ScoreAmp");

    return scoreAmp;
  }

  public static Command cancelScoreAmp(
      DrivetrainWrapper drivetrainWrapper, Elevator elevator, Arm arm) {
    Command cancelScoreAmp =
        // Commands.waitUntil(() -> drivetrainWrapper.getPoseEstimatorPose().getY() <= 7.4)
        //     .andThen(
        MechanismActions.ampPositionToLoadPosition(elevator, arm)
        // )
        ;

    cancelScoreAmp.setName("CancelScoreAmp");
    return cancelScoreAmp;
  }
}
