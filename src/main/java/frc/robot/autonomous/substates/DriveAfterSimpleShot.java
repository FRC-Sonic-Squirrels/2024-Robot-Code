package frc.robot.autonomous.substates;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.StateMachine;
import frc.lib.team6328.GeomUtil;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class DriveAfterSimpleShot extends StateMachine {
  DrivetrainWrapper drive;
  DriveToPose driveToPose;
  private LoggedTunableNumber driveDistance = new LoggedTunableNumber("Autonomous/SimpleShot/DistanceMeters", 1.5);
  Pose2d initialPose;

  public DriveAfterSimpleShot(DrivetrainWrapper drive) {
    this.drive = drive;
    setInitialState(this::startTimer);
  }

  private StateHandler startTimer() {
      Logger.recordOutput("Autonomous/SimpleShot/startTimer", true);
    initialPose = drive.getPoseEstimatorPose();
    driveToPose = new DriveToPose(drive, () -> 
      {
      double distance = Constants.isRedAlliance() ? -driveDistance.get() : driveDistance.get();
      Pose2d targetPose = initialPose.plus(GeomUtil.poseToTransform(new Pose2d(distance, 0.0, new Rotation2d())));
      Logger.recordOutput("Autonomous/SimpleShot/targetPose", targetPose);
      return targetPose;
      }
    );
    return suspendForCommand(driveToPose, (command) -> {return setDone();});
  }
}
