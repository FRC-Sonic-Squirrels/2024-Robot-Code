package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutosManager {
  private Drivetrain drivetrain;
  private Shooter shooter;
  private EndEffector endEffector;
  private Intake intake;

  private RobotConfig config;

  private LoggedDashboardChooser<Supplier<AutoCommand>> chooser;

  // FIXME: add all other subssystems

  // FIXME: IMPORTANT!!!!!!!!!!! full auto traj files are currently BROKEN due to bug with choreo.
  // issue has been submitted
  public AutosManager(
      Drivetrain drivetrain,
      Shooter shooter,
      Intake intake,
      EndEffector endEffector,
      RobotConfig config,
      LoggedDashboardChooser<Supplier<AutoCommand>> chooser) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;

    this.config = config;

    this.chooser = chooser;
    fillChooser();
  }

  private List<Supplier<AutoCommand>> allCompetitionAutos() {
    var list = new ArrayList<Supplier<AutoCommand>>();

    list.add(this::doNothing);
    list.add(this::testAuto);
    list.add(this::middle5Piece);
    list.add(this::middle6Piece);
    list.add(this::middle8Piece);
    list.add(this::amp2Piece);
    list.add(this::amp3Piece);
    list.add(this::amp5Piece);
    list.add(this::amp6Piece);
    list.add(this::source2Piece);
    list.add(this::source3Piece);
    list.add(this::source4Piece);
    list.add(this::source5Piece);
    list.add(this::source6Piece);
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
        generateFollowPathCommand("TestAuto", false),
        Choreo.getTrajectory("TestAuto").getInitialPose());
  }

  // TODO: decide on better names for autos

  private AutoCommand middle5Piece() {
    return new AutoCommand(
        "middle4Piece",
        generateFollowPathCommand("middleAuto.1", true),
        Choreo.getTrajectory("middleAuto.1").getInitialPose());
  }

  private AutoCommand middle6Piece() {
    return new AutoCommand(
        "middle5Piece",
        Commands.sequence(
            generateFollowPathCommand("middleAuto.1", true),
            generateFollowPathCommand("middleAuto.2", true)),
        Choreo.getTrajectory("middleAuto.1").getInitialPose());
  }

  private AutoCommand middle8Piece() {
    return new AutoCommand(
        "middle8Piece",
        Commands.sequence(
            generateFollowPathCommand("middleAuto.1", true),
            generateFollowPathCommand("middleAuto.2", true),
            generateFollowPathCommand("middleAuto.3", true)),
        Choreo.getTrajectory("middleAuto").getInitialPose());
  }

  private AutoCommand amp2Piece() {
    return new AutoCommand(
        "amp2Piece",
        generateFollowPathCommand("ampAuto.1", true),
        Choreo.getTrajectory("ampAuto.1").getInitialPose());
  }

  private AutoCommand amp3Piece() {
    return new AutoCommand(
        "amp3Piece",
        Commands.sequence(
            generateFollowPathCommand("ampAuto.1", true),
            generateFollowPathCommand("ampAuto.2", true)),
        Choreo.getTrajectory("ampAuto.1").getInitialPose());
  }

  private AutoCommand amp5Piece() {
    return new AutoCommand(
        "amp5Piece",
        Commands.sequence(
            generateFollowPathCommand("ampAuto.1", true),
            generateFollowPathCommand("ampAuto.2", true),
            generateFollowPathCommand("ampAuto.3", true)),
        Choreo.getTrajectory("ampAuto.1").getInitialPose());
  }

  private AutoCommand amp6Piece() {
    return new AutoCommand(
        "amp6Piece",
        Commands.sequence(
            generateFollowPathCommand("ampAuto.1", true),
            generateFollowPathCommand("ampAuto.2", true),
            generateFollowPathCommand("ampAuto.3", true),
            generateFollowPathCommand("ampAuto.4", true)),
        Choreo.getTrajectory("ampAuto.1").getInitialPose());
  }

  private AutoCommand source2Piece() {
    return new AutoCommand(
        "source2Piece",
        generateFollowPathCommand("sourceAuto.1", true),
        Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  private AutoCommand source3Piece() {
    return new AutoCommand(
        "source3Piece",
        Commands.sequence(
            generateFollowPathCommand("sourceAuto.1", true),
            generateFollowPathCommand("sourceAuto.2", true)),
        Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  private AutoCommand source4Piece() {
    return new AutoCommand(
        "source4Piece",
        Commands.sequence(
            generateFollowPathCommand("sourceAuto.1", true),
            generateFollowPathCommand("sourceAuto.2", true),
            generateFollowPathCommand("sourceAuto.3", true)),
        Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  private AutoCommand source5Piece() {
    return new AutoCommand(
        "source5Piece",
        Commands.sequence(
            generateFollowPathCommand("sourceAuto.1", true),
            generateFollowPathCommand("sourceAuto.2", true),
            generateFollowPathCommand("sourceAuto.3", true),
            generateFollowPathCommand("sourceAuto.4", true)),
        Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  private AutoCommand source6Piece() {
    return new AutoCommand(
        "source6Piece",
        Commands.sequence(
            generateFollowPathCommand("sourceAuto.1", true),
            generateFollowPathCommand("sourceAuto.2", true),
            generateFollowPathCommand("sourceAuto.3", true),
            generateFollowPathCommand("sourceAuto.4", true),
            generateFollowPathCommand("sourceAuto.5", true)),
        Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  public AutoCommand testFlipping() {
    return new AutoCommand(
        "testFlipping",
        generateFollowPathCommand("TestFlipping", false),
        Choreo.getTrajectory("testFlipping").getInitialPose());
  }

  private Command generateFollowPathCommand(
      String trajName, boolean intaking, AutoEvent... events) {
    ChoreoTrajectory traj = Choreo.getTrajectory(trajName);
    Command command =
        Choreo.choreoSwerveCommand(
            traj,
            drivetrain::getPoseEstimatorPose,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController(),
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
    if (intaking) {
      command = command.deadlineWith(new IntakeGamepiece(intake));
    }
    return command;
  }

  // work in progress

  // private AutoEvent[] setShootPoints(double[] timesSec){

  //   ArrayList<AutoEvent> autoEvents = new ArrayList<>();

  //   for (double timeSec : timesSec) {
  //     autoEvents.add(new AutoEvent(new IndexGamepiece(endEffector), timeSec));
  //   }

  //   autoEvents.add(new ShooterShootMode(shooter, drivetrain))

  //   return autoEvents.toArray(new AutoEvent[]{});
  // }
}
