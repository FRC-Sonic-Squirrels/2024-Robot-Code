package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Drivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutosManager {
  private Drivetrain drivetrain;
  private Intake intake;

  private RobotConfig config;

  private LoggedDashboardChooser<Supplier<AutoCommand>> chooser;

  // FIXME: add all other subssystems
  public AutosManager(
      Drivetrain drivetrain,
      Intake intake,
      RobotConfig config,
      LoggedDashboardChooser<Supplier<AutoCommand>> chooser) {
    this.drivetrain = drivetrain;
    this.intake = intake;

    this.config = config;

    this.chooser = chooser;
    fillChooser();
  }

  private List<Supplier<AutoCommand>> allCompetitionAutos() {
    var list = new ArrayList<Supplier<AutoCommand>>();

    list.add(this::doNothing);
    list.add(this::testAuto);
    list.add(this::auto1);
    list.add(this::auto2);
    list.add(this::testFlipping);

    return list;
  }

  private void fillChooser() {
    var compAutos = this.allCompetitionAutos();

    for (int i = 0; i < compAutos.size(); i++) {
      var supplier = compAutos.get(i);
      if (i == 0) {
        // FIXME: maybe we dont want do nothing as our default auto? maybe shoot and mobility as
        // default?
        // Do nothing command must be first in list.
        chooser.addDefaultOption(supplier.get().name, supplier);
      } else {
        chooser.addOption(supplier.get().name, supplier);
      }
    }
  }

  private AutoCommand doNothing() {
    return new AutoCommand("doNothing", new InstantCommand(), new Pose2d());
  }

  private AutoCommand testAuto() {
    return new AutoCommand(
        "testAuto",
        generateFollowPathCommand("TestAuto"),
        Choreo.getTrajectory("TestAuto").getInitialPose());
  }

  // TODO: decide on better names for autos

  private AutoCommand auto1() {
    return new AutoCommand(
        "Auto1",
        generateFollowPathCommand("Auto1"),
        Choreo.getTrajectory("Auto1").getInitialPose());
  }

  private AutoCommand auto2() {
    return new AutoCommand(
        "Auto2",
        generateFollowPathCommand("Auto2"),
        Choreo.getTrajectory("Auto2").getInitialPose());
  }

  public AutoCommand testFlipping() {
    return new AutoCommand(
        "testFlipping",
        generateFollowPathCommand("TestFlipping"),
        Choreo.getTrajectory("testFlipping").getInitialPose());
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
          command.deadlineWith(
              Commands.sequence(Commands.waitSeconds(autoEvent.timeSeconds), autoEvent.command));
    }
    return command;
  }
}
