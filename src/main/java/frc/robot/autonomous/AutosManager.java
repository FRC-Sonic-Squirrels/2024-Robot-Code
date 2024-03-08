package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.substates.DriveAfterSimpleShot;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.commands.mechanism.MechanismActionsSafe;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutosManager {
  private DrivetrainWrapper drivetrain;
  private Shooter shooter;
  private EndEffector endEffector;
  private Intake intake;
  private final Elevator elevator;
  private final Arm arm;
  private VisionGamepiece visionGamepiece;

  private RobotConfig config;

  private boolean includeDebugPaths = true;

  public record Auto(String name, Command command, Pose2d initPose) {}

  public AutosManager(
      DrivetrainWrapper drivetrain,
      Shooter shooter,
      Intake intake,
      EndEffector endEffector,
      Elevator elevator,
      Arm arm,
      VisionGamepiece visionGamepiece,
      RobotConfig config,
      LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<Auto>> stringToAutoSupplierMap) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.elevator = elevator;
    this.arm = arm;
    this.visionGamepiece = visionGamepiece;

    this.config = config;

    fillChooserAndMap(chooser, stringToAutoSupplierMap);
  }

  private List<Supplier<Auto>> allCompetitionAutos() {
    var list = new ArrayList<Supplier<Auto>>();

    list.add(this::doNothing);
    list.add(this::sourceAuto);
    list.add(this::middleAuto);
    list.add(this::ampAuto);
    list.add(this::simpleShootAuto);
    list.add(this::swerveCharacterization);

    if (includeDebugPaths) {
      list.add(() -> testPath("TestDrive1Meter", true));
      list.add(() -> testPath("TestDrive10Meter", true));
      list.add(() -> testPath("TestDrive2Meters", true));
      list.add(() -> testPath("TestDrive2MetersRotating", true));
      list.add(() -> testPath("TestDrive2MetersThenLeft", true));
      list.add(() -> testPath("TestDrive2MetersThenLeftRotating", true));
      list.add(() -> testPath("TestCircle", true));
      list.add(() -> testPath("TestCircle", false, "TestCircleDontResetPose"));
      list.add(() -> testPath("TestZigZag", false));
      list.add(this::characterization);
      list.add(
          () -> {
            var cmd = MechanismActions.loadingPosition(elevator, arm);

            cmd = cmd.andThen(MechanismActions.ampPosition(elevator, arm));

            cmd = cmd.andThen(MechanismActions.ampPositionToLoadPosition(elevator, arm));

            return new Auto("MechanismActions - Home to Amp to Home", cmd, null);
          });

      list.add(
          () -> {
            var cmd = MechanismActionsSafe.loadingPosition(elevator, arm);

            cmd = cmd.andThen(MechanismActionsSafe.ampPosition(elevator, arm));

            cmd = cmd.andThen(MechanismActionsSafe.ampPositionToLoadPosition(elevator, arm));

            return new Auto("MechanismActionsSafe - Home to Amp to Home", cmd, null);
          });
    }

    return list;
  }

  private void fillChooserAndMap(
      LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<Auto>> stringToAutoSupplierMap) {
    var compAutos = this.allCompetitionAutos();

    for (int i = 0; i < compAutos.size(); i++) {
      var supplier = compAutos.get(i);
      var name = supplier.get().name;
      if (i == 0) {
        // FIXME: maybe we dont want do nothing as our default auto? maybe shoot and mobility as
        // default?
        // Do nothing command must be first in list.
        chooser.addDefaultOption(name, name);
      } else {
        chooser.addOption(name, name);
      }

      stringToAutoSupplierMap.put(name, supplier);
    }
  }

  public Auto doNothing() {
    return new Auto("doNothing", new InstantCommand(), new Pose2d());
  }

  public Auto swerveCharacterization() {
    var sysidConfig =
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> {
              if (state != SysIdRoutineLog.State.kNone) {
                Logger.recordOutput("Sysid/swervesysidstate", state);
              }
            });

    var mechanism = new SysIdRoutine(sysidConfig, drivetrain.getSysIdMechanism());

    var command1 = mechanism.quasistatic(Direction.kForward);
    var command2 = mechanism.quasistatic(Direction.kReverse);
    var command3 = mechanism.dynamic(Direction.kForward);
    var command4 = mechanism.dynamic(Direction.kReverse);

    var finalCommand =
        command1
            // .andThen(Commands.waitSeconds(1.0))
            .andThen(command2)
            // .andThen(Commands.waitSeconds(1.0))
            .andThen(command3)
            // .andThen(Commands.waitSeconds(1.0))
            .andThen(command4)
            .andThen(
                Commands.runOnce(
                    () ->
                        Logger.recordOutput(
                            "Sysid/swervesysidstate", SysIdRoutineLog.State.kNone)));

    return new Auto("swerveCharacterization", finalCommand, new Pose2d());
  }

  private Auto sourceAuto() {
    ArrayList<Pair<String, String>> paths = new ArrayList<Pair<String, String>>();
    paths.add(new Pair<String, String>("Ssource-G5", "G5-S3"));
    paths.add(new Pair<String, String>("S3-G4", "G4-S2"));
    paths.add(new Pair<String, String>("S2-G3", "G3-S1"));
    paths.add(new Pair<String, String>("S1-G2", "G2-S1"));
    AutoStateMachine state =
        new AutoStateMachine(
            drivetrain,
            shooter,
            endEffector,
            elevator,
            arm,
            intake,
            visionGamepiece,
            paths,
            0.47,
            config);
    return new Auto(
        "ampAuto", state.asCommand(), Choreo.getTrajectory("Ssource-G5").getInitialPose());
  }

  private Auto ampAuto() {
    ArrayList<Pair<String, String>> paths = new ArrayList<Pair<String, String>>();
    paths.add(new Pair<String, String>("Samp-CG1", null));
    paths.add(new Pair<String, String>("CG1-G1", "G1-S1"));
    paths.add(new Pair<String, String>("S1-G2", "G2-S2"));
    paths.add(new Pair<String, String>("S2-G3", "G3-S2"));
    AutoStateMachine state =
        new AutoStateMachine(
            drivetrain,
            shooter,
            endEffector,
            elevator,
            arm,
            intake,
            visionGamepiece,
            paths,
            0.47,
            config);
    return new Auto(
        "ampAuto", state.asCommand(), Choreo.getTrajectory("Samp-CG1").getInitialPose());
  }

  private Auto middleAuto() {
    ArrayList<Pair<String, String>> paths = new ArrayList<Pair<String, String>>();
    paths.add(new Pair<String, String>("Smiddle-CG3", "CG3-CS2"));
    paths.add(new Pair<String, String>("CS2-CG2", "CG2-CS1"));
    paths.add(new Pair<String, String>("CS1-CG1", null));
    paths.add(new Pair<String, String>("CG1-G1", "G1-S1"));
    paths.add(new Pair<String, String>("S1-G2", "G2-S1"));
    AutoStateMachine state =
        new AutoStateMachine(
            drivetrain,
            shooter,
            endEffector,
            elevator,
            arm,
            intake,
            visionGamepiece,
            paths,
            0.47,
            config);
    return new Auto(
        "middleAuto", state.asCommand(), Choreo.getTrajectory("Smiddle-CG3").getInitialPose());
  }

  private Auto simpleShootAuto() {
    AutoStateMachine state =
        new AutoStateMachine(
            drivetrain,
            shooter,
            endEffector,
            elevator,
            arm,
            intake,
            visionGamepiece,
            new StateMachine[] {new DriveAfterSimpleShot(drivetrain)},
            10.0,
            config);

    return new Auto("simpleShootAuto", state.asCommand(), null);
  }

  private Auto testPath(String pathName, boolean useInitialPose) {
    return testPath(pathName, useInitialPose, pathName);
  }

  private Auto testPath(String pathName, boolean useInitialPose, String autoName) {
    ChoreoTrajectory traj = Choreo.getTrajectory(pathName);
    return new Auto(
        autoName,
        new Command() {
          ChoreoHelper helper;
          ChassisSpeeds speeds;

          @Override
          public void initialize() {
            helper =
                new ChoreoHelper(
                    Timer.getFPGATimestamp(),
                    drivetrain.getPoseEstimatorPose(),
                    traj,
                    config.getAutoTranslationPidController(),
                    config.getAutoTranslationPidController(),
                    config.getAutoThetaPidController());
          }

          @Override
          public void execute() {
            speeds =
                helper.calculateChassisSpeeds(
                    drivetrain.getPoseEstimatorPose(), Timer.getFPGATimestamp());
            drivetrain.setVelocityOverride(speeds);
          }

          @Override
          public void end(boolean interrupted) {
            drivetrain.resetVelocityOverride();
          }

          @Override
          public boolean isFinished() {
            return speeds == null;
          }
        },
        useInitialPose ? traj.getInitialPose() : null);
  }

  private Auto characterization() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Characterization");
    return new Auto(
        "Characterization",
        AutoBuilder.followPath(path).finallyDo(drivetrain::resetVelocityOverride),
        new Pose2d());
  }

  /* Copy these to get waypoints for choreo. If pasted in choreo, they will automatically be turned into waypoints

  G1: {
  "dataType":"choreo/waypoint","x":8.273,"y":7.474,"heading":0,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":17
  }
  G2: {
  "dataType":"choreo/waypoint","x":8.273,"y":5.792,"heading":0,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }
  G3: {
  "dataType":"choreo/waypoint","x":8.273,"y":4.11,"heading":0,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }
  G4: {
  "dataType":"choreo/waypoint","x":8.273,"y":2.428,"heading":0,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }
  G5: {
  "dataType":"choreo/waypoint","x":8.273,"y":0.746,"heading":0,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }

  S1: {
  "dataType":"choreo/waypoint","x":4.241,"y":6.103,"heading":0.185,
  "isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }
  S2: {
  "dataType":"choreo/waypoint","x":4.609,"y":4.741,"heading":-0.159,
  "isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }
  S3: {
  "dataType":"choreo/waypoint","x":3.259,"y":2.545,"heading":-0.583,
  "isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }

  CG1: {"dataType":"choreo/waypoint","x":2.89,"y":7.02,"heading":0.611,"isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40}
  CG2: {"dataType":"choreo/waypoint","x":2.89,"y":5.56,"heading":0,"isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40}
  CG3: {"dataType":"choreo/waypoint","x":2.89,"y":4.1,"heading":0,"isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":12}

  CS1: {"dataType":"choreo/waypoint","x":2.055,"y":6.613,"heading":0.463,"isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40}
  CS2: {"dataType":"choreo/waypoint","x":1.913,"y":5.569,"heading":0,"isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40}
  CS3: {"dataType":"choreo/waypoint","x":2.110,"y":4.438,"heading":-0.390,"isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40}
  */
  public static Pose2d getPoseFromString(String string) {
    if (string.charAt(0) == 'G') {
      double y = string.charAt(1) * (-841.0 / 500.0) + (2289.0 / 250.0);
      return new Pose2d(8.273, y, new Rotation2d());
    }
    if (string.charAt(0) == 'C' && string.charAt(1) == 'G') {
      int index = string.charAt(2);
      double y = index * (-73.0 / 50.0) + (212.0 / 25.0);
      return new Pose2d(2.89, y, Rotation2d.fromRadians(index == 1 ? 0.611 : 0.0));
    }
    if (string.charAt(0) == 'S') {
      int index = string.charAt(1);
      if (index == 1) {
        return new Pose2d(4.241, 6.103, Rotation2d.fromRadians(0.185));
      }
      if (index == 2) {
        return new Pose2d(4.609, 4.741, Rotation2d.fromRadians(-0.159));
      }
      if (index == 3) {
        return new Pose2d(3.259, 2.545, Rotation2d.fromRadians(-0.583));
      }
    }
    if (string.charAt(0) == 'C' && string.charAt(1) == 'S') {
      int index = string.charAt(2);
      if (index == 1) {
        return new Pose2d(2.055, 6.613, Rotation2d.fromRadians(0.463));
      }
      if (index == 2) {
        return new Pose2d(1.913, 5.569, Rotation2d.fromRadians(0.0));
      }
      if (index == 3) {
        return new Pose2d(2.110, 4.438, Rotation2d.fromRadians(-0.390));
      }
    }
    return null;
  }
}
