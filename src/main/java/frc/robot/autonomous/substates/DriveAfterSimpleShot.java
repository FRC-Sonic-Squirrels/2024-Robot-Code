package frc.robot.autonomous.substates;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class DriveAfterSimpleShot extends StateMachine {
  private static final String ROOT_TABLE = "Autonomous/SimpleShot";

  private static final LoggerGroup logGroup = new LoggerGroup(ROOT_TABLE);
  private static final LoggerEntry log_targetPose = logGroup.build("targetPose");

  private static final TunableNumberGroup groupTunable = new TunableNumberGroup(ROOT_TABLE);
  private static final LoggedTunableNumber driveDistance =
      groupTunable.build("DistanceMeters", 1.5);

  DrivetrainWrapper drive;
  DriveToPose driveToPose;
  Pose2d initialPose;

  public DriveAfterSimpleShot(DrivetrainWrapper drive) {
    super("DriveAfterSimpleShot");

    this.drive = drive;

    setInitialState(stateWithName("startTimer", this::startTimer));
  }

  private StateHandler startTimer() {
    initialPose = drive.getPoseEstimatorPose(true);
    driveToPose =
        new DriveToPose(
            drive,
            () -> {
              double distance =
                  Constants.isRedAlliance() ? -driveDistance.get() : driveDistance.get();
              Pose2d targetPose =
                  new Pose2d(
                      initialPose.getX() + distance, initialPose.getY(), Constants.zeroRotation2d);

              log_targetPose.info(targetPose);
              return targetPose;
            });
    return suspendForCommand(driveToPose, (command) -> setDone());
  }
}
