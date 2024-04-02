package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.robot.Constants;
import frc.robot.autonomous.substates.DriveAfterSimpleShot;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.commands.mechanism.MechanismActionsSafe;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutosManager {
  private static final LoggerGroup logGroupSysid = LoggerGroup.build("Sysid");
  private static final LoggerEntry.EnumValue<SysIdRoutineLog.State> logSwerveSysidState =
      logGroupSysid.buildEnum("swervesysidstate");

  private final AutosSubsystems subsystems;
  private final RobotConfig config;

  public final boolean includeDebugPaths = false;

  public record Auto(String name, Command command, Pose2d initPose) {}

  public AutosManager(
      AutosSubsystems subsystems,
      RobotConfig config,
      LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<Auto>> stringToAutoSupplierMap) {
    this.subsystems = subsystems;
    this.config = config;

    fillChooserAndMap(chooser, stringToAutoSupplierMap);
  }

  private List<Supplier<Auto>> allCompetitionAutos() {
    var list = new ArrayList<Supplier<Auto>>();

    list.add(this::doNothing);
    list.add(this::subWooferCloseFirst);
    list.add(this::rushCenterGP1First);
    list.add(this::sourceAuto);
    list.add(this::sourceAuto4GP);
    list.add(this::sourceAuto5GP);
    list.add(this::simpleShootAuto);

    if (includeDebugPaths) {
      list.add(this::portableAuto);
      list.add(this::swerveCharacterization);
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
            var elevator = subsystems.elevator();
            var arm = subsystems.arm();

            var cmd = MechanismActions.loadingPosition(elevator, arm);

            cmd = cmd.andThen(MechanismActions.ampPosition(elevator, arm));

            cmd = cmd.andThen(MechanismActions.ampPositionToLoadPosition(elevator, arm));

            return new Auto("MechanismActions - Home to Amp to Home", cmd, null);
          });

      list.add(
          () -> {
            var elevator = subsystems.elevator();
            var arm = subsystems.arm();

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
    return new Auto("doNothing", new InstantCommand(), Constants.zeroPose2d);
  }

  private Auto sourceAuto() {
    List<PathDescriptor> paths = new ArrayList<>();
    paths.add(new PathDescriptor("Ssource-G5", "G5-S3", true));
    paths.add(new PathDescriptor("S3-G4", "G4-S2", true));
    paths.add(new PathDescriptor("S2-G3", "G3-S2", true));
    AutoStateMachine state = new AutoStateMachine(subsystems, config, paths);
    return new Auto(
        "SOURCE_3GP", state.asCommand(), Choreo.getTrajectory("Ssource-G5").getInitialPose());
  }

  private Auto sourceAuto4GP() {
    List<PathDescriptor> paths = new ArrayList<>();
    paths.add(new PathDescriptor("Ssource-G5", "G5-S3", true));
    paths.add(new PathDescriptor("S3-G4", "G4-S2", true));
    paths.add(new PathDescriptor("S2-G3", "G3-S2", true));
    paths.add(new PathDescriptor("S1-G2", "G2-S1", true));
    AutoStateMachine state = new AutoStateMachine(subsystems, config, paths);
    return new Auto(
        "SOURCE_4GP", state.asCommand(), Choreo.getTrajectory("Ssource-G5").getInitialPose());
  }

  private Auto sourceAuto5GP() {
    List<PathDescriptor> paths = new ArrayList<>();
    paths.add(new PathDescriptor("Ssource-G5", "G5-S3", true));
    paths.add(new PathDescriptor("S3-G4", "G4-S2", true));
    paths.add(new PathDescriptor("S2-G3", "G3-S2", true));
    paths.add(new PathDescriptor("S1-G2", "G2-S1", true));
    paths.add(new PathDescriptor("S1-G1", "G1-S1", true));
    AutoStateMachine state = new AutoStateMachine(subsystems, config, paths);
    return new Auto(
        "SOURCE_5GP", state.asCommand(), Choreo.getTrajectory("Ssource-G5").getInitialPose());
  }

  private Auto rushCenterGP1First() {
    List<PathDescriptor> paths = new ArrayList<>();
    paths.add(new PathDescriptor("Samp2-G1", "G1-S1", true));
    paths.add(new PathDescriptor("S1-G2", "G2-S1", true));
    paths.add(new PathDescriptor("S1-G3", "G3-S1", true));
    // paths.add(new PathDescriptor("S2-G4", "G4-S3", true));
    // paths.add(new PathDescriptor("S3-G5", "G5-S3", true));
    AutoStateMachine state = new AutoStateMachine(subsystems, config, true, "Samp-Samp2", paths);
    return new Auto(
        "RUSH_CENTER_GP_1_FIRST",
        state.asCommand(),
        Choreo.getTrajectory("Samp-Samp2").getInitialPose());
  }

  private Auto subWooferCloseFirst() {
    List<PathDescriptor> paths = new ArrayList<>();
    paths.add(new PathDescriptor("Smiddle-CG3", "CG3-CS2", false));
    paths.add(new PathDescriptor("CG3-CG2", null, false));
    paths.add(new PathDescriptor("CG2-CG1", null, false));
    paths.add(new PathDescriptor("CG1-G1", "G1-S1", true));
    paths.add(new PathDescriptor("S1-G2", "G2-S1", true));
    AutoStateMachine state = new AutoStateMachine(subsystems, config, paths);
    return new Auto(
        "SUBWOOFER_CLOSE_FIRST",
        state.asCommand(),
        Choreo.getTrajectory("Smiddle-CG3").getInitialPose());
  }

  private Auto portableAuto() {
    List<PathDescriptor> paths = new ArrayList<>();
    paths.add(new PathDescriptor("TestPortable1", "TestPortable2", true));
    paths.add(new PathDescriptor("TestPortable3", "TestPortable4", true));
    var state = new AutoStateMachine(subsystems, config, paths);
    return new Auto(
        "TestPortable", state.asCommand(), Choreo.getTrajectory("TestPortable1").getInitialPose());
  }

  private Auto simpleShootAuto() {
    StateMachine[] paths = new StateMachine[] {new DriveAfterSimpleShot(subsystems.drivetrain())};
    var state = new AutoStateMachine(subsystems, config, paths);

    return new Auto("simpleShootAuto", state.asCommand(), null);
  }

  private Auto testPath(String pathName, boolean useInitialPose) {
    return testPath(pathName, useInitialPose, pathName);
  }

  private Auto testPath(String pathName, boolean useInitialPose, String autoName) {
    var traj = ChoreoTrajectoryWithName.getTrajectory(pathName);
    return new Auto(
        autoName,
        new Command() {
          private final DrivetrainWrapper drivetrain = subsystems.drivetrain();
          ChoreoHelper helper;
          boolean atEndOfPath;

          @Override
          public void initialize() {
            drivetrain.setPose(Constants.zeroPose2d);
            helper =
                new ChoreoHelper(
                    Timer.getFPGATimestamp(),
                    drivetrain.getPoseEstimatorPose(true),
                    traj,
                    config.getDriveBaseRadius() / 2,
                    1.0,
                    config.getAutoTranslationPidController(),
                    config.getAutoTranslationPidController(),
                    config.getAutoThetaPidController());
          }

          @Override
          public void execute() {
            var result =
                helper.calculateChassisSpeeds(
                    drivetrain.getPoseEstimatorPose(true), Timer.getFPGATimestamp());
            drivetrain.setVelocityOverride(result.chassisSpeeds());
            atEndOfPath = result.atEndOfPath();
          }

          @Override
          public void end(boolean interrupted) {
            drivetrain.resetVelocityOverride();
          }

          @Override
          public boolean isFinished() {
            return atEndOfPath;
          }
        },
        useInitialPose ? traj.getInitialPose(true) : null);
  }

  private Auto characterization() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Characterization");
    return new Auto(
        "Characterization",
        Commands.runOnce(() -> subsystems.drivetrain().setPose(Constants.zeroPose2d))
            .andThen(AutoBuilder.followPath(path))
            .finallyDo(subsystems.drivetrain()::resetVelocityOverride),
        Constants.zeroPose2d);
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

  CG1: {"dataType":"choreo/waypoint","x":2.89,"y":7.02,"heading":0.611,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40}
  CG2: {"dataType":"choreo/waypoint","x":2.89,"y":5.56,"heading":0,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40}
  CG3: {"dataType":"choreo/waypoint","x":2.89,"y":4.1,"heading":0,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":12}

  CS1: {"dataType":"choreo/waypoint","x":2.055,"y":6.613,"heading":0.463,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40}
  CS2: {"dataType":"choreo/waypoint","x":1.913,"y":5.569,"heading":0,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40}
  CS3: {"dataType":"choreo/waypoint","x":2.110,"y":4.438,"heading":-0.390,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40}
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

  public Auto swerveCharacterization() {
    var sysidConfig =
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> {
              if (state != SysIdRoutineLog.State.kNone) {
                logSwerveSysidState.info(state);
              }
            });

    var mechanism = new SysIdRoutine(sysidConfig, subsystems.drivetrain().getSysIdMechanism());

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
            .andThen(Commands.runOnce(() -> logSwerveSysidState.info(SysIdRoutineLog.State.kNone)));

    return new Auto("swerveCharacterization", finalCommand, Constants.zeroPose2d);
  }
}
