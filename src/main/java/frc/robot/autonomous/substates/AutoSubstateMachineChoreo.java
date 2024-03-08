package frc.robot.autonomous.substates;

import com.choreo.lib.ChoreoTrajectory;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public class AutoSubstateMachineChoreo extends AutoSubstateMachine {
  /** Creates a new AutoSubstateMachine. */
  public AutoSubstateMachineChoreo(
      DrivetrainWrapper drive,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      RobotConfig config,
      Elevator elevator,
      Arm arm,
      ChoreoTrajectory trajToGamepiece,
      ChoreoTrajectory trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    super(
        drive,
        shooter,
        endEffector,
        intake,
        config,
        elevator,
        arm,
        trajToGamepiece,
        trajToShoot,
        closestGamepiece,
        null);
  }
}
