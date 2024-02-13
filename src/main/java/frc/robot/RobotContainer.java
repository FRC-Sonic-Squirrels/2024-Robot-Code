// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.ArrayUtil;
import frc.lib.team2930.GeometryUtil;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.autonomous.AutoCommand;
import frc.robot.autonomous.AutosManager;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.commands.drive.DriveToGamepiece;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.commands.shooter.ShooterStowMode;
import frc.robot.configs.SimulatorRobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOReal;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionModuleConfiguration;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIO;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIOReal;
import frc.robot.visualization.MechanismVisualization;
import frc.robot.visualization.SimpleMechanismVisualization;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain;
  private final DrivetrainWrapper drivetrainWrapper;
  private final Vision vision;
  private final Arm arm;
  private final Elevator elevator;
  private final Intake intake;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final LED led;
  private final VisionGamepiece visionGamepiece;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final ShuffleBoardLayouts shuffleBoardLayouts;

  private final LoggedDashboardChooser<Supplier<AutoCommand>> autoChooser =
      new LoggedDashboardChooser<Supplier<AutoCommand>>("Auto Routine");
  private final AutosManager autoManager;

  ScoreSpeaker scoreSpeaker;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    RobotType robotType = Constants.RobotMode.getRobot();
    Mode mode = Constants.RobotMode.getMode();

    Logger.recordOutput("RobotType", robotType);
    Logger.recordOutput("RobotMode", mode);

    var config = robotType.config.get();
    AprilTagFieldLayout aprilTagLayout;

    try {
      aprilTagLayout = config.getAprilTagFieldLayout();
    } catch (Exception e) {
      // FIXME: throw an error.
      aprilTagLayout = new AprilTagFieldLayout(new ArrayList<AprilTag>(), 0.0, 0.0);
    }

    if (mode == Mode.REPLAY) {
      drivetrain = new Drivetrain(config, new GyroIO() {}, config.getReplaySwerveModuleObjects());

      vision =
          new Vision(
              aprilTagLayout,
              drivetrain::getPoseEstimatorPose,
              drivetrain::addVisionEstimate,
              config.getReplayVisionModules());

      arm = new Arm(new ArmIO() {});
      elevator = new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
      shooter = new Shooter(new ShooterIO() {});
      endEffector = new EndEffector(new EndEffectorIO() {});
      led = new LED();
      visionGamepiece =
          new VisionGamepiece(new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPose);

    } else { // REAL and SIM robots HERE
      switch (robotType) {
        case ROBOT_SIMBOT_REAL_CAMERAS:
        case ROBOT_SIMBOT:
          com.ctre.phoenix6.unmanaged.Unmanaged.setPhoenixDiagnosticsStartTime(0.0);

          drivetrain = new Drivetrain(config, new GyroIO() {}, config.getSwerveModuleObjects());

          if (robotType == RobotType.ROBOT_SIMBOT_REAL_CAMERAS) {
            // Sim Robot, Real Cameras
            vision =
                new Vision(
                    aprilTagLayout,
                    drivetrain::getPoseEstimatorPose,
                    drivetrain::addVisionEstimate,
                    config.getVisionModuleObjects());

            visionGamepiece =
                new VisionGamepiece(new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPose);

          } else {
            VisionModuleConfiguration[] visionModules = {
              new VisionModuleConfiguration(
                  new VisionIOSim(
                      config,
                      drivetrain::getPoseEstimatorPose,
                      SimulatorRobotConfig.INTAKE_SIDE_LEFT,
                      SimulatorRobotConfig.INTAKE_SIDE_LEFT_CAMERA_NAME),
                  SimulatorRobotConfig.INTAKE_SIDE_LEFT_CAMERA_NAME,
                  SimulatorRobotConfig.INTAKE_SIDE_LEFT),
              new VisionModuleConfiguration(
                  new VisionIOSim(
                      config,
                      drivetrain::getPoseEstimatorPose,
                      SimulatorRobotConfig.INTAKE_SIDE_RIGHT,
                      SimulatorRobotConfig.INTAKE_SIDE_RIGHT_CAMERA_NAME),
                  SimulatorRobotConfig.INTAKE_SIDE_RIGHT_CAMERA_NAME,
                  SimulatorRobotConfig.INTAKE_SIDE_RIGHT),
              new VisionModuleConfiguration(
                  new VisionIOSim(
                      config,
                      drivetrain::getPoseEstimatorPose,
                      SimulatorRobotConfig.SHOOTER_SIDE_LEFT,
                      SimulatorRobotConfig.SHOOTER_SIDE_LEFT_CAMERA_NAME),
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT),
              new VisionModuleConfiguration(
                  new VisionIOSim(
                      config,
                      drivetrain::getPoseEstimatorPose,
                      SimulatorRobotConfig.SHOOTER_SIDE_RIGHT,
                      SimulatorRobotConfig.SHOOTER_SIDE_RIGHT_CAMERA_NAME),
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT),
            };
            // Sim Cameras
            vision =
                new Vision(
                    aprilTagLayout,
                    drivetrain::getPoseEstimatorPose,
                    drivetrain::addVisionEstimate,
                    visionModules);

            visionGamepiece =
                new VisionGamepiece(new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPose);
          }

          arm = new Arm(new ArmIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          intake = new Intake(new IntakeIOSim());
          shooter = new Shooter(new ShooterIOSim());
          endEffector = new EndEffector(new EndEffectorIOSim());
          led = new LED();
          break;

        case ROBOT_2023_RETIRED_ROBER:
          drivetrain =
              new Drivetrain(config, new GyroIOPigeon2(config), config.getSwerveModuleObjects());

          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          led = new LED();
          visionGamepiece =
              new VisionGamepiece(new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPose);
          break;

        case ROBOT_2024:
          // README: for development purposes, comment any of the real IO's you DON'T want to use
          // and
          // uncomment the empty IO's as a replacement

          // -- All real IO's
          drivetrain =
              new Drivetrain(config, new GyroIOPigeon2(config), config.getSwerveModuleObjects());

          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());
          visionGamepiece =
              new VisionGamepiece(new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPose);

          intake = new Intake(new IntakeIOReal());
          elevator = new Elevator(new ElevatorIOReal());
          arm = new Arm(new ArmIOReal());
          endEffector = new EndEffector(new EndEffectorIOReal());
          shooter = new Shooter(new ShooterIOReal());

          led = new LED();

          // -- All empty IO's
          // drivetrain =
          //     new Drivetrain(config, new GyroIO() {}, config.getReplaySwerveModuleObjects());

          // vision =
          //     new Vision(
          //         aprilTagLayout,
          //         drivetrain::getPoseEstimatorPose,
          //         drivetrain::addVisionEstimate,
          //         config.getReplayVisionModules());

          // visionGamepiece =
          //     new VisionGamepiece(new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPose);

          // intake = new Intake(new IntakeIO() {});
          // elevator = new Elevator(new ElevatorIO() {});
          // arm = new Arm(new ArmIO() {});
          // endEffector = new EndEffector(new EndEffectorIO() {});
          // shooter = new Shooter(new ShooterIO() {});

          // led = new LED();
          break;

        default:
          drivetrain =
              new Drivetrain(config, new GyroIO() {}, config.getReplaySwerveModuleObjects());
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          led = new LED();
          visionGamepiece =
              new VisionGamepiece(new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPose);
          break;
      }
    }

    autoManager = new AutosManager(drivetrain, shooter, intake, endEffector, config, autoChooser);

    drivetrainWrapper = new DrivetrainWrapper(drivetrain);

    drivetrain.setDefaultCommand(
        new DrivetrainDefaultTeleopDrive(
            drivetrainWrapper,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    intake.setDefaultCommand(new IntakeGamepiece(intake, driverController.getHID()));

    shooter.setDefaultCommand(new ShooterStowMode(shooter));

    // Set up named commands for PathPlanner
    // NamedCommands.registerCommand(
    //     "Run Flywheel",
    //     Commands.startEnd(
    //         () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    // autoChooser.addOption(
    //     "Flywheel FF Characterization",
    //     new FeedForwardCharacterization(
    //         flywheel, flywheel::runCharacterizationVolts,
    // flywheel::getCharacterizationVelocity));

    shuffleBoardLayouts =
        new ShuffleBoardLayouts(arm, elevator, endEffector, intake, shooter, drivetrain);

    scoreSpeaker =
        new ScoreSpeaker(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            drivetrain,
            shooter,
            driverController.a());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
    Button binding summary:

      Driver:

        right trigger: spin intake

        left trigger: drive to GP

        right bumper: speaker mode

        A button: confirm speaker shot

        left bumper: score in amp (may have A button confirmation)

        B button: force stow everything

        X button: move all position controlled subsystems to hard stops to reset sensor position

        Y button: toggle climb mode (A: confirm climb)

        Start: reset odometry

        POV down: eject gamepiece (through intake)

      Operator:

        B: (when not connected to FMS) run systems check command: press B to continue onto next stage of systems check
          Drivetrain
            1. Forward
            2. Spin
          Intake
            3. Spin
          Shooter
            4. Pivot
            5. Flywheel + kicker
          Elevator
            6. up
          Arm
            7. Rotate to amp or trap position
          Wrist
            8. move through range of motion
          End Effector
            9. (after moving mech back to stow position) spin
          ----- end command -----
          Vision
            10. check localization vision (preferably with april tag)

            12. look at LEDs
            13. double check connection from pigeon

        X: cancel systems check

        Right trigger (plus right stick): manual control of elevator

        POV down: initiate no vision mode?


    */
    // ----------- DRIVER CONTROLS ------------

    driverController
        .leftTrigger()
        .whileTrue(
            new DriveToGamepiece(
                visionGamepiece::getClosestGamepiece, drivetrainWrapper, intake::getBeamBreak));

    driverController.rightBumper().whileTrue(scoreSpeaker);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public LoggedDashboardChooser<Supplier<AutoCommand>> getAutonomousChooser() {
    return autoChooser;
  }

  public void setPose(Pose2d pose) {
    drivetrain.setPose(pose);
  }

  public double[] getCurrentDraws() {
    double[] current =
        new double[] {
          intake.getCurrentDraw(),
        };

    current = ArrayUtil.concatWithArrayCopy(current, drivetrain.getCurrentDrawAmps());
    return current;
  }

  public void applyToDrivetrain() {
    drivetrainWrapper.apply();
  }

  public void updateVisualization() {
    SimpleMechanismVisualization.updateVisualization(
        new Rotation2d(),
        shooter.getPitch(),
        Math.hypot(
            GeometryUtil.getDist(
                drivetrain.getPoseEstimatorPose().getTranslation(),
                Constants.FieldConstants.getSpeakerTranslation()),
            Constants.FieldConstants.SPEAKER_HEIGHT_METERS),
        shooter.getRPM(),
        elevator.getHeightInches());

    SimpleMechanismVisualization.logMechanism();

    scoreSpeaker.updateVisualization();

    MechanismVisualization.updateVisualization(
        arm.getAngle(),
        shooter.getPitch(),
        elevator.getHeightInches(),
        endEffector.intakeSideTOFDetectGamepiece() || endEffector.shooterSideTOFDetectGamepiece());

    MechanismVisualization.logMechanism();
  }
}
