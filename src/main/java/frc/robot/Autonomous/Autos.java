package frc.robot.Autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.Drivetrain;
import java.util.Optional;

public class Autos {

  private Drivetrain drivetrain;
  private RobotConfig config;

  public Autos(Drivetrain drivetrain, RobotConfig config) {
    this.drivetrain = drivetrain;
    this.config = config;
  }

  private AutoCommand doNothing() {
    return new AutoCommand("doNothing", new InstantCommand(), new Pose2d(0, 0, new Rotation2d(0)));
  }

  private AutoCommand testAuto() {
    return new AutoCommand(
        "testAuto",
        generateFollowPathCommand("testAuto"),
        Choreo.getTrajectory("testAuto").getInitialPose());
  }

  public AutoCommand[] autoCommands() {
    return new AutoCommand[] {doNothing(), testAuto()};
  }

  private Command generateFollowPathCommand(String name, AutoEvent... events) {
    ChoreoTrajectory traj = Choreo.getTrajectory(name);
    Command command =
        Choreo.choreoSwerveCommand(
            traj,
            drivetrain::getPose,
            new PIDController(
                config.getAutoTranslationKP().get(),
                config.getAutoTranslationKI().get(),
                config.getAutoTranslationKD().get()),
            new PIDController(
                config.getAutoTranslationKP().get(),
                config.getAutoTranslationKI().get(),
                config.getAutoTranslationKD().get()),
            new PIDController(
                config.getAutoThetaKP().get(),
                config.getAutoThetaKI().get(),
                config.getAutoThetaKD().get()),
            (ChassisSpeeds speeds) -> drivetrain.runVelocity(speeds),
            () -> {
              Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == Alliance.Red;
            },
            drivetrain);
    for (AutoEvent autoEvent : events) {
      command =
          command.alongWith(
              new SequentialCommandGroup(
                  new WaitCommand(autoEvent.timeSeconds), autoEvent.command));
    }
    return command;
  }
}
