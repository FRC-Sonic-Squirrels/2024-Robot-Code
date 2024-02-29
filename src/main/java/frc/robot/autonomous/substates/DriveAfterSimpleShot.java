package frc.robot.autonomous.substates;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.StateMachine;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import org.littletonrobotics.junction.Logger;

public class DriveAfterSimpleShot extends StateMachine {
  DrivetrainWrapper drive;
  DriveToPose driveToPose;
  private LoggedTunableNumber driveDistance =
      new LoggedTunableNumber("Autonomous/SimpleShot/DistanceMeters", 1.5);
  Pose2d initialPose;

  public DriveAfterSimpleShot(DrivetrainWrapper drive) {
    super("DriveAfterSimpleShot");

    this.drive = drive;

    setInitialState(stateWithName("startTimer", this::startTimer));
  }

  private StateHandler startTimer() {
    initialPose = drive.getPoseEstimatorPose();
    driveToPose =
        new DriveToPose(
            drive,
            () -> {
              double distance =
                  Constants.isRedAlliance() ? -driveDistance.get() : driveDistance.get();
              Pose2d targetPose =
                  new Pose2d(initialPose.getX() + distance, initialPose.getY(), new Rotation2d());
              Logger.recordOutput("Autonomous/SimpleShot/targetPose", targetPose);
              return targetPose;
            });
    return suspendForCommand(driveToPose, (command) -> setDone());
  }
}
