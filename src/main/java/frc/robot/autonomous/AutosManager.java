package frc.robot.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive.RotateToSpeaker;
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
import org.littletonrobotics.junction.Logger;
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
        generateFollowPathCommand("TestAuto", false, new AutoRotationState[] {}),
        Choreo.getTrajectory("TestAuto").getInitialPose());
  }

  // TODO: decide on better names for autos

  private AutoCommand middle5Piece() {
    return new AutoCommand(
        "middle5Piece",
        generateFollowPathCommand(
            "middleAuto.1",
            true,
            new AutoRotationState[] {
              new AutoRotationState(ChoreoRotationMode.ROTATE_TO_GAMEPIECE, 0.0),
              new AutoRotationState(ChoreoRotationMode.FOLLOW_PATH, 0.48),
              new AutoRotationState(ChoreoRotationMode.ROTATE_TO_GAMEPIECE, 1.05),
              new AutoRotationState(ChoreoRotationMode.FOLLOW_PATH, 4.35)
            }),
        Choreo.getTrajectory("middleAuto.1").getInitialPose());
  }

  private AutoCommand middle6Piece() {
    return new AutoCommand(
        "middle6Piece",
        Commands.sequence(
            middle5Piece().command,
            generateFollowPathCommand("middleAuto.2", true, new AutoRotationState[] {})),
        Choreo.getTrajectory("middleAuto.1").getInitialPose());
  }

  private AutoCommand middle8Piece() {
    return new AutoCommand(
        "middle8Piece",
        Commands.sequence(
            middle6Piece().command,
            generateFollowPathCommand("middleAuto.3", true, new AutoRotationState[] {})),
        Choreo.getTrajectory("middleAuto").getInitialPose());
  }

  private AutoCommand amp2Piece() {
    return new AutoCommand(
        "amp2Piece",
        generateFollowPathCommand("ampAuto.1", true, new AutoRotationState[] {}),
        Choreo.getTrajectory("ampAuto.1").getInitialPose());
  }

  private AutoCommand amp3Piece() {
    return new AutoCommand(
        "amp3Piece",
        Commands.sequence(
            amp2Piece().command,
            generateFollowPathCommand("ampAuto.2", true, new AutoRotationState[] {})),
        Choreo.getTrajectory("ampAuto.1").getInitialPose());
  }

  private AutoCommand amp5Piece() {
    return new AutoCommand(
        "amp5Piece",
        Commands.sequence(
            amp3Piece().command,
            generateFollowPathCommand("ampAuto.3", true, new AutoRotationState[] {})),
        Choreo.getTrajectory("ampAuto.1").getInitialPose());
  }

  private AutoCommand amp6Piece() {
    return new AutoCommand(
        "amp6Piece",
        Commands.sequence(
            amp5Piece().command,
            generateFollowPathCommand("ampAuto.4", true, new AutoRotationState[] {})),
        Choreo.getTrajectory("ampAuto.1").getInitialPose());
  }

  private AutoCommand source2Piece() {
    return new AutoCommand(
        "source2Piece",
        generateFollowPathCommand("sourceAuto.1", true, new AutoRotationState[] {}),
        Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  private AutoCommand source3Piece() {
    return new AutoCommand(
        "source3Piece",
        Commands.sequence(
            source2Piece().command,
            generateFollowPathCommand("sourceAuto.2", true, new AutoRotationState[] {})),
        Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  private AutoCommand source4Piece() {
    return new AutoCommand(
        "source4Piece",
        Commands.sequence(
            source3Piece().command,
            generateFollowPathCommand("sourceAuto.3", true, new AutoRotationState[] {})),
        Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  private AutoCommand source5Piece() {
    return new AutoCommand(
        "source5Piece",
        Commands.sequence(
            source4Piece().command,
            generateFollowPathCommand("sourceAuto.4", true, new AutoRotationState[] {})),
        Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  private AutoCommand source6Piece() {
    return new AutoCommand(
        "source6Piece",
        Commands.sequence(
            source5Piece().command,
            generateFollowPathCommand("sourceAuto.5", true, new AutoRotationState[] {})),
        Choreo.getTrajectory("sourceAuto.1").getInitialPose());
  }

  public AutoCommand testFlipping() {
    return new AutoCommand(
        "testFlipping",
        generateFollowPathCommand("TestFlipping", false, new AutoRotationState[] {}),
        Choreo.getTrajectory("testFlipping").getInitialPose());
  }

  private Command generateFollowPathCommand(
      String trajName, boolean intaking, AutoRotationState[] rotationStates, AutoEvent... events) {
    ChoreoTrajectory traj = Choreo.getTrajectory(trajName);
    Command command =
        Choreo.choreoSwerveCommand(
            traj,
            drivetrain::getPoseEstimatorPose,
            choreoSwerveController(
                config.getAutoTranslationPidController(),
                config.getAutoTranslationPidController(),
                config.getAutoThetaPidController(),
                shooter::getRPM,
                rotationStates),
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

  /**
   * Creates a control function for following a ChoreoTrajectoryState.
   *
   * @param xController A PIDController for field-relative X translation (input: X error in meters,
   *     output: m/s).
   * @param yController A PIDController for field-relative Y translation (input: Y error in meters,
   *     output: m/s).
   * @param rotationController A PIDController for robot rotation (input: heading error in radians,
   *     output: rad/s). This controller will have its continuous input range set to -pi..pi by
   *     ChoreoLib.
   * @return A ChoreoControlFunction to track ChoreoTrajectoryStates. This function returns
   *     robot-relative ChassisSpeeds.
   */
  private ChoreoControlFunction choreoSwerveController(
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Supplier<Double> shooterRPM,
      AutoRotationState... rotationStates) {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    return (pose, referenceState) -> {
      double xFF = referenceState.velocityX;
      double yFF = referenceState.velocityY;
      double rotationFF;

      AutoRotationState lastState = new AutoRotationState(ChoreoRotationMode.FOLLOW_PATH, -1.0);

      for (AutoRotationState autoRotationState : rotationStates) {
        if (autoRotationState.timestamp <= referenceState.timestamp
            && autoRotationState.timestamp >= lastState.timestamp) lastState = autoRotationState;
      }

      double xFeedback = xController.calculate(pose.getX(), referenceState.x);
      double yFeedback = yController.calculate(pose.getY(), referenceState.y);

      double xVel = xFF + xFeedback;
      double yVel = yFF + yFeedback;

      double targetHeading;
      switch (lastState.rotationMode) {
        case FOLLOW_PATH:
          targetHeading = referenceState.heading;
          rotationFF = referenceState.angularVelocity;
          break;
        case ROTATE_TO_GAMEPIECE:
          targetHeading = referenceState.heading;
          rotationFF = referenceState.angularVelocity;
          break;
        case ROTATE_TO_SPEAKER:
          targetHeading =
              RotateToSpeaker.calculateTargetRot(
                      new Rotation2d(Math.PI), shooterRPM.get(), pose, pose, yVel)
                  .getRadians();
          rotationFF =
              RotateToSpeaker.calculateRotVelRadPerSec(
                  xVel, yVel, new Rotation2d(targetHeading), pose, shooterRPM.get());
          break;
        default:
          targetHeading = referenceState.heading;
          rotationFF = referenceState.angularVelocity;
          break;
      }
      double rotationFeedback =
          rotationController.calculate(pose.getRotation().getRadians(), targetHeading);

      Logger.recordOutput("autonomous/targetHeading", targetHeading);
      Logger.recordOutput("autonomous/rotationMode", lastState.rotationMode);

      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xVel, yVel, rotationFF + rotationFeedback, pose.getRotation());
    };
  }

  public static enum ChoreoRotationMode {
    FOLLOW_PATH,
    ROTATE_TO_GAMEPIECE,
    ROTATE_TO_SPEAKER
  }
}
