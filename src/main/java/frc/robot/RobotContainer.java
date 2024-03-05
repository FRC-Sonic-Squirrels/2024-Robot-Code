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

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.ArrayUtil;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.commands.RunsWhenDisabledInstantCommand;
import frc.lib.team2930.lib.controller_rumble.ControllerRumbleUntilButtonPress;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.autonomous.AutosManager;
import frc.robot.autonomous.AutosManager.Auto;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.commands.Shimmy;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.commands.endEffector.EndEffectorCenterNoteBetweenToFs;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.commands.mechanism.HomeMechanism;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.commands.mechanism.arm.ArmSetAngle;
import frc.robot.commands.mechanism.elevator.ElevatorSetHeight;
import frc.robot.commands.shooter.HomeShooter;
import frc.robot.commands.shooter.ShooterScoreSpeakerStateMachine;
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
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionModuleConfiguration;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIO;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIOReal;
import frc.robot.visualization.ClimbVisualization;
import frc.robot.visualization.GamepieceVisualization;
import frc.robot.visualization.MechanismVisualization;
import frc.robot.visualization.SimpleMechanismVisualization;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
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
  public final Vision vision;
  private final Arm arm;
  private final Elevator elevator;
  private final Intake intake;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final LED led;
  private final VisionGamepiece visionGamepiece;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final LoggedTunableNumber tunableX = new LoggedTunableNumber("tunableXFromSpeaker", 50.0);
  private final LoggedTunableNumber tunableY = new LoggedTunableNumber("tunableYFromSpeaker", 0);

  private final ShuffleBoardLayouts shuffleBoardLayouts;

  private final LoggedDashboardChooser<String> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private final HashMap<String, Supplier<Auto>> stringToAutoSupplierMap = new HashMap<>();
  private final AutosManager autoManager;

  public DigitalInput breakModeButton = new DigitalInput(0);
  public DigitalInput homeSensorsButton = new DigitalInput(1);

  Trigger breakModeButtonTrigger =
      new Trigger(() -> !breakModeButton.get() && !DriverStation.isEnabled());

  Trigger homeSensorsButtonTrigger =
      new Trigger(() -> !homeSensorsButton.get() && !DriverStation.isEnabled());

  // private LoggedTunableNumber tunableX = new LoggedTunableNumber("Localization/tunableXPose",
  // 0.0);
  // private LoggedTunableNumber tunableY = new LoggedTunableNumber("Localization/tunableYPose",
  // 0.0);

  boolean brakeModeTriggered = false;

  ScoreSpeaker scoreSpeaker;
  AutoClimb autoClimb;

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

    // BUGBUG: practice field usable tags
    ArrayList<AprilTag> goodTags = new ArrayList<>();
    for (var tag : aprilTagLayout.getTags()) {
      // switch (tag.ID) {
      //   case 3:
      //   case 4:
      //   case 5:
      //   case 6:
      //   case 7:
      //   case 8:
      //     goodTags.add(tag);
      //     break;
      // }
      goodTags.add(tag);
    }

    aprilTagLayout =
        new AprilTagFieldLayout(
            goodTags, aprilTagLayout.getFieldLength(), aprilTagLayout.getFieldWidth());

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
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.INTAKE_SIDE_LEFT_CAMERA_NAME,
                  SimulatorRobotConfig.INTAKE_SIDE_LEFT,
                  config,
                  drivetrain::getPoseEstimatorPose),
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.INTAKE_SIDE_RIGHT_CAMERA_NAME,
                  SimulatorRobotConfig.INTAKE_SIDE_RIGHT,
                  config,
                  drivetrain::getPoseEstimatorPose),
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT,
                  config,
                  drivetrain::getPoseEstimatorPose),
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT,
                  config,
                  drivetrain::getPoseEstimatorPose),
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

        case ROBOT_2024_MAESTRO:
          // README: for development purposes, comment any of the real IO's you DON'T want to use
          // and
          // uncomment the empty IO's as a replacement

          // -- All real IO's
          drivetrain =
              new Drivetrain(config, new GyroIOPigeon2(config), config.getSwerveModuleObjects());

          // vision =
          //     new Vision(
          //         aprilTagLayout,
          //         drivetrain::getPoseEstimatorPose,
          //         drivetrain::addVisionEstimate,
          //         config.getVisionModuleObjects());
          // visionGamepiece =
          //     new VisionGamepiece(new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPose);

          intake = new Intake(new IntakeIOReal());
          // elevator = new Elevator(new ElevatorIOReal());
          // arm = new Arm(new ArmIOReal());
          endEffector = new EndEffector(new EndEffectorIOReal());
          shooter = new Shooter(new ShooterIOReal());

          led = new LED();

          // -- All empty IO's
          // drivetrain =
          //     new Drivetrain(config, new GyroIO() {}, config.getReplaySwerveModuleObjects());

          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());

          visionGamepiece =
              new VisionGamepiece(new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPose);

          // intake = new Intake(new IntakeIO() {});
          elevator = new Elevator(new ElevatorIOReal());
          arm = new Arm(new ArmIOReal());
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

    drivetrainWrapper = new DrivetrainWrapper(drivetrain);

    AutoBuilder.configureHolonomic(
        drivetrain::getPoseEstimatorPose,
        drivetrain::setPose,
        drivetrain::getChassisSpeeds,
        drivetrainWrapper::setVelocityOverride,
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            config
                .getDriveBaseRadius(), // Drive base radius in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> false,
        drivetrain);

    autoManager =
        new AutosManager(
            drivetrainWrapper,
            shooter,
            intake,
            endEffector,
            elevator,
            arm,
            visionGamepiece,
            config,
            autoChooser,
            stringToAutoSupplierMap);

    drivetrain.setDefaultCommand(
        new DrivetrainDefaultTeleopDrive(
            drivetrainWrapper,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // arm.setDefaultCommand(autoClimb);

    // shooter.setDefaultCommand(new ShooterStowMode(shooter));

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

    scoreSpeaker = new ScoreSpeaker(drivetrainWrapper, shooter, endEffector, driverController.a());

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

        Y button: toggle climb mode

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

    // driverController
    //     .leftTrigger()
    //     .whileTrue(
    //         new DriveToGamepiece(
    //             visionGamepiece::getClosestGamepiece,
    //             drivetrainWrapper,
    //             endEffector::intakeSideTOFDetectGamepiece));

    driverController
        .back()
        .onTrue(
            Commands.runOnce(
                () ->
                    drivetrain.setPose(
                        new Pose2d(
                            drivetrain.getPoseEstimatorPose().getX(),
                            drivetrain.getPoseEstimatorPose().getY(),
                            Constants.zeroRotation2d)),
                drivetrain));

    driverController
        .start()
        .onTrue(
            Commands.runOnce(() -> vision.useMaxDistanceAwayFromExistingEstimate(false), vision))
        .onFalse(
            Commands.runOnce(() -> vision.useMaxDistanceAwayFromExistingEstimate(true), vision));

    driverController
        .rightBumper()
        .whileTrue(
            new IntakeGamepiece(
                intake,
                endEffector,
                shooter,
                arm,
                elevator,
                (rumble) -> {
                  driverController.getHID().setRumble(RumbleType.kBothRumble, rumble);
                }));
    // .beforeStarting(MechanismActions.loadingPosition(elevator, arm))
    // .andThen(new EndEffectorCenterNoteBetweenToFs(endEffector).withTimeout(1.5)));

    // driverController
    //     .rightTrigger()
    //     .whileTrue(
    //         ShooterScoreSpeakerStateMachine.getAsCommand(
    //             drivetrainWrapper,
    //             shooter,
    //             endEffector,
    //             intake,
    //             1000,
    //             driverController.a(),
    //             (rumble) -> driverController.getHID().setRumble(RumbleType.kBothRumble,
    // rumble)));

    // driverController
    //     .leftBumper()
    //     .onTrue(Commands.run(() -> shooter.setPivotPosition(Rotation2d.fromDegrees(45)),
    // shooter));

    // driverController
    //     .leftBumper()
    //     .whileTrue(CommandComposer.scoreAmp(endEffector, drivetrainWrapper, elevator, arm))
    //     .onFalse(CommandComposer.cancelScoreAmp(drivetrainWrapper, endEffector, elevator,  arm));

    driverController
        .leftBumper()
        .onTrue(
            CommandComposer.scoreAmp(
                endEffector,
                drivetrainWrapper,
                elevator,
                arm,
                intake,
                shooter,
                false,
                new ControllerRumbleUntilButtonPress(
                    (r) -> {
                      driverController.getHID().setRumble(RumbleType.kBothRumble, r);
                    },
                    driverController.a(),
                    0.5)))
        .onFalse(CommandComposer.cancelScoreAmp(drivetrainWrapper, endEffector, elevator, arm));

    operatorController.start().whileTrue(new Shimmy(intake, endEffector, shooter));

    if (Constants.unusedCode) {
      driverController
          .y()
          .onTrue(
              Commands.runOnce(
                  () ->
                      drivetrain.setPose(
                          new Pose2d(
                              Constants.FieldConstants.getSpeakerTranslation()
                                  .plus(
                                      new Translation2d(
                                          Units.Inches.of(tunableX.get()).in(Units.Meters), 0.0)),
                              Constants.zeroRotation2d))));
    }

    // driverController
    //     .rightTrigger()
    //     .whileTrue(
    //         Commands.parallel(
    //             Commands.run(() -> intake.setPercentOut(0.75), intake),
    //             Commands.run(() -> endEffector.setPercentOut(0.75), endEffector),
    //             Commands.run(
    //                 () -> {
    //                   shooter.setKickerPercentOut(0.75);
    //                   shooter.setLauncherVoltage(11.0);
    //                 },
    //                 shooter)))
    //     .onFalse(
    //         Commands.parallel(
    //             Commands.run(() -> intake.setPercentOut(0.0), intake),
    //             Commands.run(() -> endEffector.setPercentOut(0.0), endEffector),
    //             Commands.run(
    //                 () -> {
    //                   shooter.setKickerPercentOut(0.0);
    //                   shooter.setLauncherVoltage(0.0);
    //                 },
    //                 shooter)));

    // driverController
    //     .a()
    //     .whileTrue(
    //         new ShooterSimpleShoot(
    //                 shooter,
    //                 endEffector,
    //                 () -> 11,
    //                 () -> Rotation2d.fromDegrees(tunablePivotPitch.get()),
    //                 () -> 0.95,
    //                 () -> 0.5)
    //             .alongWith(Commands.runOnce(() -> intake.setPercentOut(0.5), intake)))
    //     .onFalse(Commands.runOnce(() -> intake.setPercentOut(0.0), intake));

    // driverController
    //     .start()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> arm.setAngle(Rotation2d.fromDegrees(tunablePivotPitch.get())), arm));

    if (Constants.unusedCode) {
      driverController
          .x()
          .whileTrue(
              new DrivetrainDefaultTeleopDrive(
                  drivetrainWrapper,
                  () -> -driverController.getLeftY(),
                  () -> -driverController.getLeftX(),
                  () -> -driverController.getRightX()));
    }

    // driverController
    //     .y()
    //     .whileTrue(CommandComposer.autoClimb(drivetrainWrapper, elevator, arm, endEffector));

    driverController.povDown().onTrue(MechanismActions.loadingPosition(elevator, arm));

    driverController
        .leftTrigger()
        .whileTrue(new DriveToPose(drivetrainWrapper, Constants.FieldConstants::getAmpScoringPose));

    PIDController rotationController = new PIDController(4.9, 0, 0);

    driverController
        .povRight()
        .whileTrue(
            Commands.run(
                () -> {
                  drivetrainWrapper.setRotationOverride(
                      rotationController.calculate(
                          drivetrainWrapper.getRotation().getRadians(),
                          Units.Degrees.of(90.0).in(Units.Radians)));
                }));

    // driverController
    //     .y()
    //     .whileTrue(
    //         new DriveToPose(
    //             drivetrainWrapper,
    //             () ->
    //                 new Pose2d(
    //                     Constants.FieldConstants.getSpeakerTranslation()
    //                         .plus(
    //                             new Translation2d(
    //                                 Units.Inches.of(tunableX.get()).in(Units.Meters),
    //                                 Units.Inches.of(tunableY.get()).in(Units.Meters))),
    //                     Constants.zeroRotation2d)));
    // driverController
    //     .povUp()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> elevator.setHeight(Units.Inches.of(tunableElevatorHeightOne.get())),
    //             elevator));
    // driverController
    //     .povRight()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> elevator.setHeight(Units.Inches.of(tunableElevatorHeightTwo.get())),
    //             elevator));

    // driverController.povLeft().onTrue(MechanismActions.ampFast(elevator, arm));

    // driverController.povUp().onTrue(MechanismActions.ampPosition(elevator, arm));

    // driverController.povRight().onTrue(MechanismActions.ampPositionToLoadPosition(elevator,
    // arm));

    // driverController
    //     .x()
    //     .onTrue(Commands.runOnce(() -> endEffector.setPercentOut(0.5), endEffector))
    //     .onFalse(Commands.runOnce(() -> endEffector.setPercentOut(0.0), endEffector));

    // driverController.povRight().onTrue(MechanismActions.climbPrepPosition(elevator, arm));
    // driverController
    //     .b()
    //     .onTrue(new InstantCommand(() -> elevator.setHeight(Units.Meters.of(0.0)), elevator));

    // driverController.b().onTrue(new InstantCommand(() -> elevator.setVoltage(0.0), elevator));

    DoubleSupplier elevatorDelta =
        () -> MathUtil.applyDeadband(-operatorController.getLeftY() * 1, 0.3) / 5.0;
    DoubleSupplier armDelta =
        () -> MathUtil.applyDeadband(operatorController.getRightY() * -1, 0.3) * 10.0;

    operatorController
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                new ElevatorSetHeight(
                    elevator,
                    () ->
                        Units.Inches.of(elevator.getHeightInches() + elevatorDelta.getAsDouble())),
                new ArmSetAngle(
                    arm,
                    () -> arm.getAngle().plus(Rotation2d.fromDegrees(armDelta.getAsDouble())))));

    // ---------- OPERATOR CONTROLS -----------
    if (Constants.unusedCode) {
      operatorController.a().whileTrue(new HomeMechanism(elevator, arm));
      operatorController.b().whileTrue(new HomeShooter(shooter));

      operatorController
          .y()
          .onTrue(Commands.runOnce(() -> shooter.setKickerPercentOut(0.8), shooter))
          .onFalse(Commands.runOnce(() -> shooter.setKickerPercentOut(0.0), shooter));

      operatorController
          .povLeft()
          .onTrue(new InstantCommand(elevator::resetSensorToHomePosition, elevator));
      operatorController.povUp().onTrue(new InstantCommand(arm::resetSensorToHomePosition, arm));
      operatorController
          .a()
          .onTrue(new InstantCommand(() -> endEffector.setPercentOut(0.8), endEffector))
          .onFalse(new InstantCommand(() -> endEffector.setPercentOut(0.0), endEffector));
    }

    operatorController
        .povUp()
        .onTrue(MechanismActions.climbPrepPosition(elevator, arm, endEffector, shooter, intake));
    operatorController.povLeft().onTrue(MechanismActions.climbChainCheck(elevator, arm));
    operatorController.povDown().onTrue(MechanismActions.climbDownPosition(elevator, arm));
    operatorController.povRight().onTrue(MechanismActions.climbTrapPosition(elevator, arm));

    operatorController.y().onTrue(MechanismActions.deployReactionArms(elevator, arm));
    operatorController.x().onTrue(MechanismActions.climbFinalRestPosition(elevator, arm));
    // operatorController.povLeft().onTrue(MechanismActions.climbTrapPushPosition(elevator, arm));
    operatorController
        .b()
        .onTrue(Commands.runOnce(() -> endEffector.setPercentOut(0.5), endEffector))
        .onFalse(Commands.runOnce(() -> endEffector.setPercentOut(0.0), endEffector));

    operatorController
        .a()
        .whileTrue(new EndEffectorCenterNoteBetweenToFs(endEffector, intake, shooter));

    driverController
        .y()
        .whileTrue(
            ShooterScoreSpeakerStateMachine.getAsCommand(
                drivetrainWrapper,
                shooter,
                endEffector,
                intake,
                5.0,
                () -> true,
                (r) -> {},
                true,
                Constants.ShooterConstants.Pivot.MAX_ANGLE_RAD.minus(Rotation2d.fromDegrees(3.0))));

    operatorController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> vision.setUsingVision(false), vision))
        .onFalse(Commands.runOnce(() -> vision.setUsingVision(true), vision));

    // -- buttons on robot

    homeSensorsButtonTrigger.onTrue(
        Commands.runOnce(elevator::resetSensorToHomePosition, elevator)
            .andThen(arm::resetSensorToHomePosition, arm)
            .andThen(shooter::pivotResetHomePosition, shooter)
            .ignoringDisable(true));

    breakModeButtonTrigger.onTrue(
        new ConditionalCommand(
            Commands.runOnce(
                    () -> {
                      arm.setNeutralMode(NeutralModeValue.Coast);
                      elevator.setNeutralMode(NeutralModeValue.Coast);
                      shooter.setNeutralMode(NeutralModeValue.Coast);
                      brakeModeTriggered = false;
                    },
                    elevator,
                    arm)
                .ignoringDisable(true),
            Commands.runOnce(
                    () -> {
                      arm.setNeutralMode(NeutralModeValue.Brake);
                      elevator.setNeutralMode(NeutralModeValue.Brake);
                      shooter.setNeutralMode(NeutralModeValue.Brake);
                      brakeModeTriggered = true;
                    },
                    elevator,
                    arm)
                .ignoringDisable(true),
            () -> brakeModeTriggered));

    // Add Reset and Reboot buttons to SmartDashboard
    SmartDashboard.putData(
        "PV Restart SW 1_Shooter_Left",
        new RunsWhenDisabledInstantCommand(() -> Vision.restartPhotonVision("10.29.30.13")));
    SmartDashboard.putData(
        "PV REBOOT 1_Shooter_Left",
        new RunsWhenDisabledInstantCommand(() -> Vision.rebootPhotonVision("10.29.30.13")));

    SmartDashboard.putData(
        "PV Restart SW 2_Shooter_Right",
        new RunsWhenDisabledInstantCommand(() -> Vision.restartPhotonVision("10.29.30.14")));
    SmartDashboard.putData(
        "PV REBOOT 2_Shooter_Right",
        new RunsWhenDisabledInstantCommand(() -> Vision.rebootPhotonVision("10.29.30.14")));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public LoggedDashboardChooser<String> getAutonomousChooser() {
    return autoChooser;
  }

  public Supplier<Auto> getAutoSupplierForString(String string) {
    return stringToAutoSupplierMap.getOrDefault(string, autoManager::doNothing);
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

  public void resetDrivetrainResetOverrides() {
    drivetrainWrapper.resetVelocityOverride();
    drivetrainWrapper.resetRotationOverride();
  }

  public void updateVisualization() {
    SimpleMechanismVisualization.updateVisualization(
        Constants.zeroRotation2d,
        shooter.getPitch(),
        Math.hypot(
            GeometryUtil.getDist(
                drivetrain.getPoseEstimatorPose().getTranslation(),
                Constants.FieldConstants.getSpeakerTranslation()),
            Constants.FieldConstants.SPEAKER_HEIGHT.in(Units.Meters)),
        shooter.getRPM(),
        elevator.getHeightInches());

    SimpleMechanismVisualization.logMechanism();

    MechanismVisualization.updateVisualization(
        arm.getAngle(),
        shooter.getPitch(),
        elevator.getHeightInches(),
        endEffector.intakeSideTOFDetectGamepiece() || endEffector.shooterSideTOFDetectGamepiece());

    MechanismVisualization.logMechanism();

    GamepieceVisualization.getInstance().logTraj();

    ClimbVisualization.getInstance().updateVisualization(drivetrainWrapper.getPoseEstimatorPose());
    ClimbVisualization.getInstance().logPose();
  }

  public void resetSubsystems() {
    elevator.resetSubsystem();
    arm.resetSubsystem();
  }

  public void setBrakeMode() {
    brakeModeTriggered = true;
    elevator.setNeutralMode(NeutralModeValue.Brake);
    arm.setNeutralMode(NeutralModeValue.Brake);
    shooter.setNeutralMode(NeutralModeValue.Brake);
  }
}
