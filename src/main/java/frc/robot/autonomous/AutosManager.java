package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.substates.AutoSubstateMachine;
import frc.robot.autonomous.substates.DriveAfterSimpleShot;
import frc.robot.autonomous.substates.MiddleFirstSubstate;
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
    AutoStateMachine state =
        new AutoStateMachine(
            drivetrain,
            shooter,
            endEffector,
            elevator,
            arm,
            intake,
            new AutoSubstateMachine[] {
              generateSubstateMachine("sourceAuto", "G5S3"), generateSubstateMachine("S3G4", "G4S2")
              // generateSubstateMachine("S2G3", "G3S1"),
              // generateSubstateMachine("S1G2", "G2S1"),
              // generateSubstateMachine("S1G1", "G1S1")
            },
            1.31);
    return new Auto(
        "sourceAuto", state.asCommand(), Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  private Auto ampAuto() {
    AutoStateMachine state =
        new AutoStateMachine(
            drivetrain,
            shooter,
            endEffector,
            elevator,
            arm,
            intake,
            new AutoSubstateMachine[] {
              generateSubstateMachine("ampAuto.1", "G1S1"),
              generateSubstateMachine("S1G2", "G2S2"),
              generateSubstateMachine("S2G3", "G3S2")
            },
            1.31);
    return new Auto(
        "ampAuto", state.asCommand(), Choreo.getTrajectory("ampAuto.1").getInitialPose());
  }

  private Auto middleAuto() {
    AutoStateMachine state =
        new AutoStateMachine(
            drivetrain,
            shooter,
            endEffector,
            elevator,
            arm,
            intake,
            new StateMachine[] {
              new MiddleFirstSubstate(
                  drivetrain,
                  shooter,
                  endEffector,
                  intake,
                  elevator,
                  arm,
                  config,
                  visionGamepiece::getClosestGamepiece)
              // generateSubstateMachine("MiddleG1", "G2S2"),
              // generateSubstateMachine("S2G3", "G3S3"),
              // generateSubstateMachine("S3G4", "G4S3"),
              // generateSubstateMachine("S3G5", "G5S3")
            },
            0.47);
    return new Auto(
        "middleAuto", state.asCommand(), Choreo.getTrajectory("middleAuto.1").getInitialPose());
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
            new StateMachine[] {new DriveAfterSimpleShot(drivetrain)},
            10.0);

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
                    drivetrain.getPoseEstimatorPoseWithGyroOnlyRotation(),
                    traj,
                    config.getAutoTranslationPidController(),
                    config.getAutoTranslationPidController(),
                    config.getAutoThetaPidController());
          }

          @Override
          public void execute() {
            speeds =
                helper.calculateChassisSpeeds(
                    drivetrain.getPoseEstimatorPoseWithGyroOnlyRotation(),
                    Timer.getFPGATimestamp());
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

  private AutoSubstateMachine generateSubstateMachine(String trajToGP, String trajToShoot) {
    return new AutoSubstateMachine(
        drivetrain,
        shooter,
        endEffector,
        intake,
        config,
        elevator,
        arm,
        trajToGP,
        trajToShoot,
        visionGamepiece::getClosestGamepiece);
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
  "dataType":"choreo/waypoint","x":8.273,"y":0.742,"heading":0,"isInitialGuess":false,
  "translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }

  S1: {
  "dataType":"choreo/waypoint","x":4.241904258728027,"y":6.103699207305908,"heading":0.185945735814592,
  "isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }
  S2: {
  "dataType":"choreo/waypoint","x":4.60924768447876,"y":4.741092681884766,"heading":-0.1594733550343424,
  "isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }
  S3: {
  "dataType":"choreo/waypoint","x":3.2595736980438232,"y":2.54587984085083,"heading":-0.5838703049653,
  "isInitialGuess":false,"translationConstrained":true,"headingConstrained":true,"controlIntervalCount":40
  }

  */
}
