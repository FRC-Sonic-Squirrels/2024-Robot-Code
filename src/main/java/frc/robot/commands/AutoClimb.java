// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.GeometryUtil;
import frc.robot.Constants;
import frc.robot.commands.mechanism.MechanismPositions;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.visualization.ClimbVisualization;
import org.littletonrobotics.junction.Logger;

public class AutoClimb extends Command {

  private DrivetrainWrapper drive;
  private Elevator elevator;
  private Arm arm;
  private EndEffector endEffector;
  private int stage = 1;
  private Pose2d initialPose;
  private Trigger endEffectorTrigger;
  private double additionalRobotHeight;

  /** Creates a new AutoClimb. */
  public AutoClimb(DrivetrainWrapper drive, Elevator elevator, Arm arm, EndEffector endEffector) {
    this.drive = drive;
    this.elevator = elevator;
    this.arm = arm;
    this.endEffector = endEffector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, arm, endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endEffectorTrigger =
        new Trigger(
                () ->
                    !endEffector.intakeSideTOFDetectGamepiece()
                        && !endEffector.shooterSideTOFDetectGamepiece())
            .debounce(0.5);
    Logger.recordOutput("AutoClimb/stage", stage);
    ClimbVisualization.getInstance().updateAdditionalHeight(additionalRobotHeight);
    /*
     * STAGE 0 (before command starts): move in front of chain
     * STAGE 1: bring elevator and arm to position to begin climb
     * STAGE 2: move into chain
     * STAGE 3: bring elevator back down
     * STAGE 4: bring elevator and arm to position to score
     * STAGE 5: push open trap
     * STAGE 6: run end effector
     * STAGE 7: end command
     */
    switch (stage) {
      case 1:
        elevator.setHeight(Constants.ElevatorConstants.HEIGHT_ABOVE_CHAIN);
        arm.setAngle(Rotation2d.fromDegrees(90.0));
        if (elevator.isAtTarget(Constants.ElevatorConstants.HEIGHT_ABOVE_CHAIN)
            && arm.isAtTargetAngle()) {
          stage++;
          initialPose = drive.getPoseEstimatorPose(false);
        }

        break;
      case 2:
        Translation2d centerToRobot =
            drive
                .getPoseEstimatorPose(false)
                .getTranslation()
                .minus(Constants.FieldConstants.getStageCenter());
        double speedMetersPerSecond = 0.4;
        double flippedSpeedMetersPerSecond =
            centerToRobot.getX() >= 0.0 ? -speedMetersPerSecond : speedMetersPerSecond;
        double travelDist = 0.67;
        double vx =
            flippedSpeedMetersPerSecond
                / Math.sqrt(1.0 + Math.pow(centerToRobot.getY() / centerToRobot.getX(), 2.0));
        double vy = centerToRobot.getY() * vx / centerToRobot.getX();
        if (GeometryUtil.getDist(initialPose, drive.getPoseEstimatorPose(false)) <= travelDist) {
          drive.setVelocityOverride(
              ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0.0, drive.getRotationGyroOnly()));
        } else {
          drive.resetVelocityOverride();
          stage++;
        }
        break;
      case 3:
        elevator.setHeight(Units.Inches.of(0.0));
        additionalRobotHeight =
            Math.max(
                Constants.ElevatorConstants.HEIGHT_ABOVE_CHAIN.in(Units.Inches)
                    - elevator.getHeightInches()
                    - 2.0,
                0.0);
        if (elevator.isAtTarget(Units.Inches.of(0.0))) {
          stage++;
        }
        break;
      case 4:
        MechanismPosition position = MechanismPositions.climbTrapPosition();
        elevator.setHeight(position.elevatorHeight());
        arm.setAngle(position.armAngle());
        Logger.recordOutput("AutoClimb/armIsAtPosition", arm.isAtTargetAngle(position.armAngle()));
        if (elevator.isAtTarget(position.elevatorHeight())
            && arm.isAtTargetAngle(position.armAngle())) stage++;
        break;
      case 5:
        arm.setAngle(Constants.ArmConstants.TRAP_SCORE_ANGLE);
        if (arm.isAtTargetAngle(Constants.ArmConstants.TRAP_SCORE_ANGLE)) stage++;
        break;
      case 6:
        endEffector.setPercentOut(0.7);
        if (endEffectorTrigger.getAsBoolean()) {
          endEffector.setPercentOut(0.0);
          stage++;
        }
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    additionalRobotHeight = 0.0;
    ClimbVisualization.getInstance().updateAdditionalHeight(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 7;
  }

  public Pose2d getTargetPose(Pose2d robotPose) {
    Pose2d flippedPose = AllianceFlipUtil.flipPoseForAlliance(robotPose);
    Pose2d[] poses = Constants.FieldConstants.getClimbPositionsBlueAlliance(1.738);
    Logger.recordOutput("AutoClimb/poses", poses);
    Pose2d closestPose = null;
    for (int i = 0; i < poses.length; i++) {
      if (closestPose == null) {
        closestPose = poses[i];
      } else {
        Logger.recordOutput(
            "AutoClimb/" + i + "_Distance", GeometryUtil.getDist(poses[i], flippedPose));
        if (GeometryUtil.getDist(poses[i], flippedPose)
            < GeometryUtil.getDist(closestPose, flippedPose)) {
          closestPose = poses[i];
        }
      }
    }

    closestPose = AllianceFlipUtil.flipPoseForAlliance(closestPose);

    Logger.recordOutput("AutoClimb/ClosestPose", closestPose);
    return closestPose;
  }

  public boolean underStage(Pose2d robotPose) {
    Pose2d flippedPose = AllianceFlipUtil.flipPoseForAlliance(robotPose);
    /*
     * Corners:
     * 1: (5.814887046813965, 5.759140968322754)
     * 2: (2.9535560607910156, 4.1112542152404785)
     * 3: (5.769944667816162, 2.538270950317383)
     * Equations:
     * y < 0.575916x + 2.41025
     * y > −0.558511x + 5.76085
     * x < 5.81
     */
    return flippedPose.getY() < 0.575916 * flippedPose.getX() + 2.41025
        && flippedPose.getY() > -0.558511 * flippedPose.getX() + 5.76085
        && flippedPose.getX() < 5.81;
  }
}
