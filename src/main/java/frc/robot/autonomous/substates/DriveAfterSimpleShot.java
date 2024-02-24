package frc.robot.autonomous.substates;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team2930.StateMachine;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class DriveAfterSimpleShot extends StateMachine {
  DrivetrainWrapper drive;
  Timer timer = new Timer();
  private LoggedTunableNumber driveTimer = new LoggedTunableNumber("Autonomous/driveTime", 1.5);

  public DriveAfterSimpleShot(DrivetrainWrapper drive) {
    this.drive = drive;
    setInitialState(this::startTimer);
  }

  private StateHandler startTimer() {
    timer.start();
    return this::drive;
  }

  private StateHandler drive() {
    drive.setVelocityOverride(
        ChassisSpeeds.fromFieldRelativeSpeeds(1.0, 0.0, 0.0, drive.getRotation()));
    if (timer.get() >= driveTimer.get()) {
      drive.resetVelocityOverride();
      return setDone();
    }
    return null;
  }
}
