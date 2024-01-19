package frc.robot.autonomous;

import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.intake.EjectGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Drivetrain;

public class Autos {

  // FIXME: fill in drive characterization for choreo and regenerate paths

  private Drivetrain drivetrain;
  private RobotConfig config;

  private Intake intake;

  public Autos(Drivetrain drivetrain, Intake intake, RobotConfig config) {
    this.drivetrain = drivetrain;
    this.config = config;
    this.intake = intake;
  }

  private AutoCommand doNothing() {
    return new AutoCommand("doNothing", new InstantCommand(), new Pose2d(), new ChoreoTrajectory());
  }

  private AutoCommand testAuto() {
    return new AutoCommand(
        "testAuto",
        generateFollowPathCommand("TestAuto"),
        Choreo.getTrajectory("TestAuto").getInitialPose(),
        Choreo.getTrajectory("TestAuto"));
  }

  // TODO: decide on better names for autos

  private AutoCommand auto1() {
    return new AutoCommand(
        "Auto1",
        generateFollowPathCommand("Auto1"),
        Choreo.getTrajectory("Auto1").getInitialPose(),
        Choreo.getTrajectory("Auto1"));
  }

  private AutoCommand auto2() {
    return new AutoCommand(
        "Auto2",
        generateFollowPathCommand("Auto2"),
        Choreo.getTrajectory("Auto2").getInitialPose(),
        Choreo.getTrajectory("Auto2"));
  }

  private AutoCommand exampleBrokenAuto() {
    AutoEvent[] events = {new AutoEvent(new EjectGamepiece(intake), 0), new AutoEvent(new EjectGamepiece(intake), 4) };

    return new AutoCommand("broken auto", generateFollowPathCommand("Auto1", events), 
    Choreo.getTrajectory("Auto2").getInitialPose(), 
    Choreo.getTrajectory("Auto2"));
  }

  private AutoCommand exampleWorkingAuto() {
    AutoEvent[] events = {new AutoEvent(new EjectGamepiece(intake).asProxy(), 0), new AutoEvent(new EjectGamepiece(intake).asProxy(), 4) };

    return new AutoCommand("broken auto", generateFollowPathCommand("Auto1", events), 
    Choreo.getTrajectory("Auto2").getInitialPose(), 
    Choreo.getTrajectory("Auto2"));
  }

  public AutoCommand[] autoCommands() {
    return new AutoCommand[] {doNothing(), testAuto(), auto1(), auto2(), exampleBrokenAuto(), exampleWorkingAuto()};
  }

  private Command generateFollowPathCommand(String name, AutoEvent... events) {
    ChoreoTrajectory traj = Choreo.getTrajectory(name);
    Command command =
        Choreo.choreoSwerveCommand(
            traj,
            Robot.isSimulation()
                ? (drivetrain::getRawOdometryPose)
                : (drivetrain::getPoseEstimatorPose),
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
