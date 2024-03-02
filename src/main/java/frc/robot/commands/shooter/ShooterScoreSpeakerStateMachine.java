// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.*;
import frc.lib.team2930.ShootingSolver.Solution;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.visualization.GamepieceVisualization;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShooterScoreSpeakerStateMachine extends StateMachine {
  private static final TunableNumberGroup group = new TunableNumberGroup("ShooterScoreSpeaker");

  private static final LoggedTunableNumber tunableRPM = group.build("tunableRPM", 9000.0);

  private static final LoggedTunableNumber tunablePitchOffset =
      group.build("tunablePitchOffset", 0.0);

  private static final LoggedTunableNumber rotationKp = group.build("rotationKp", 6.0);
  private static final LoggedTunableNumber rotationKd = group.build("rotationKd", 0.0);

  private static final LoggedTunableNumber rumbleIntensity = group.build("rumbleIntensity", 0.5);

  private static final LoggedTunableNumber slowLoadingEEPercent =
      group.build("loading/slowEEPercent", 0.1);
  private static final LoggedTunableNumber slowLoadingIntakePercent =
      group.build("loading/slowLoadingIntakePercent", 0.1);
  private static final LoggedTunableNumber slowLoadingKickerPercent =
      group.build("loading/slowLoadingKickerPercent", 0.1);

  private static final LoggedTunableNumber loadingEEPercent =
      group.build("loading/LoadingEEPercent", 0.3);
  private static final LoggedTunableNumber loadingIntakePercent =
      group.build("loading/LoadingIntakePercent", 0.3);
  private static final LoggedTunableNumber loadingKickerPercent =
      group.build("loading/LoadingKickerPercent", 0.3);

  private static final LoggedTunableNumber shootingKickerPercent =
      group.build("shootingKickerPercent", 1.0);
  private final DrivetrainWrapper drivetrainWrapper;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final Intake intake;
  // such as driver confirmation, in auto this is always true
  private final Consumer<Double> rumbleConsumer;
  private final BooleanSupplier externalConfirmation;
  private final double forceShotIn;

  private double startOfShooting;

  // FIXME: are these constants correct?
  private final ShootingSolver solver =
      new ShootingSolver(
          Constants.FieldConstants.getSpeakerTranslation3D(),
          GeometryUtil.translation3dFromMeasures(
              Units.Inches.of(-1.0), Units.Inches.of(0), Units.Inches.of(9.0)),
          GeometryUtil.translation3dFromMeasures(
              Units.Inches.of(0), Units.Inches.of(-10), Units.Inches.of(0.0)),
          Constants.ShooterConstants.SHOOTING_SPEED,
          Constants.ShooterConstants.SHOOTING_TIME,
          false);

  private final PIDController rotationController;

  private Solution solverResult;
  private Rotation2d desiredShootingPitch;
  private Rotation2d desiredShootingHeading;

  public ShooterScoreSpeakerStateMachine(
      DrivetrainWrapper drivetrainWrapper,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      double forceShotIn,
      BooleanSupplier externalConfirmation,
      Consumer<Double> rumbleConsumer) {
    super("ShooterScoreSpeaker");

    this.drivetrainWrapper = drivetrainWrapper;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.rumbleConsumer = rumbleConsumer;
    this.externalConfirmation = externalConfirmation;
    this.forceShotIn = forceShotIn;

    rotationController = new PIDController(rotationKp.get(), 0, rotationKd.get());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    setInitialState(stateWithName("prepWhileLoadingGamepiece", this::prepWhileLoadingGamepiece));
  }

  public static Command getAsCommand(
      DrivetrainWrapper drivetrainWrapper,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      double forceShotIn,
      BooleanSupplier externalSupplier,
      Consumer<Double> rumbleConsumer) {

    Command command =
        new RunStateMachineCommand(
            () ->
                new ShooterScoreSpeakerStateMachine(
                    drivetrainWrapper,
                    shooter,
                    endEffector,
                    intake,
                    forceShotIn,
                    externalSupplier,
                    rumbleConsumer),
            shooter,
            endEffector);

    // FIXME: probably dont need this add requirments here once we add the asCommand(Subsytem...
    // requirments) method to the StateMachine class
    command.addRequirements(shooter, endEffector, intake);
    command.setName("ShooterScoreSpeakerStateMachine");

    // once statemachine is over, stop shooter and end effector subsystems
    command =
        command.finallyDo(
            () -> {
              shooter.setKickerPercentOut(0.0);
              endEffector.setPercentOut(0.0);
              shooter.setLauncherVoltage(0.0);
              shooter.setPivotPosition(Constants.ShooterConstants.Pivot.HOME_POSITION);
              drivetrainWrapper.resetRotationOverride();
              rumbleConsumer.accept(0.0);
              intake.setPercentOut(0.0);
            });

    return command;
  }

  public static Command getAsCommand(
      DrivetrainWrapper drivetrainWrapper,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      double forceShotIn) {
    return getAsCommand(
        drivetrainWrapper, shooter, endEffector, intake, forceShotIn, () -> true, (ignore) -> {});
  }

  public StateHandler prepWhileLoadingGamepiece() {
    // aim towards speaker

    updateSolver();
    rotateToSpeaker();

    // ramp up rpm
    shooter.setLauncherRPM(
        // ShooterConstants.SHOOTING_RPM
        tunableRPM.get());
    // shooter.setLauncherVoltage(10.0);

    if (shooter.noteInShooter()) {
      shooter.setKickerPercentOut(0.0);
      endEffector.setPercentOut(0.0);
      intake.setPercentOut(0.0);
      return stateWithName(
          "finalConfirmationsBeforeShooting", this::finalConfirmationsBeforeShooting);
    }

    // FIXME: look into calculating gear ratio to make it so kicker and EE spin at same speed
    shooter.setPivotPosition(Constants.ShooterConstants.Pivot.MIN_ANGLE_RAD);
    if (shooter.isPivotIsAtTarget(Constants.ShooterConstants.Pivot.MIN_ANGLE_RAD)) {
      shooter.markStartOfNoteLoading();
      if (endEffector.intakeSideTOFDetectGamepiece()) {
        shooter.setKickerPercentOut(loadingKickerPercent.get());

        endEffector.markStartOfNoteDropping();
        endEffector.setPercentOut(loadingEEPercent.get());
        intake.setPercentOut(loadingIntakePercent.get());
      } else {
        shooter.setKickerPercentOut(slowLoadingKickerPercent.get());
        endEffector.setPercentOut(slowLoadingEEPercent.get());
        intake.setPercentOut(slowLoadingIntakePercent.get());
      }
    }

    return null;
  }

  public StateHandler finalConfirmationsBeforeShooting() {
    updateSolver();

    // still be continuously aiming towards speaker
    // check launcher rpm correct
    // check pivot position correct
    // check robot theta correct
    // check external confirmation active
    // check are we in valid shooting position
    boolean pivotAtAngle;
    var launcherAtRpm = shooter.isAtTargetRPM();
    if (solverResult != null) {
      shooter.setPivotPosition(desiredShootingPitch);
      pivotAtAngle = shooter.isPivotIsAtTarget(desiredShootingPitch);
    } else {
      pivotAtAngle = false;
    }

    rotateToSpeaker();

    var externalConfirmation = this.externalConfirmation.getAsBoolean();
    boolean atThetaTarget;

    if (solverResult != null) {
      var thetaError =
          desiredShootingHeading.getRadians() - drivetrainWrapper.getRotation().getRadians();
      while (thetaError <= -Math.PI) thetaError += Math.PI * 2;
      while (thetaError >= Math.PI) thetaError -= Math.PI * 2;
      atThetaTarget = Math.abs(thetaError) <= Math.toRadians(5.0);
    } else {
      atThetaTarget = false;
    }

    var forceShoot = stateRunningLongerThan(forceShotIn);

    log("shootingConfimation/launcherAtRPM", launcherAtRpm);
    log("shootingConfimation/pivotAtAngle", pivotAtAngle);
    log("shootingConfimation/shootingPosition", validShootingPosition());
    log("shootingConfimation/forceShoot", forceShoot);
    log("shootingConfimation/driverConfirmation", externalConfirmation);
    log("shootingConfimation/atThetaTarget", atThetaTarget);
    if ((launcherAtRpm && pivotAtAngle && validShootingPosition() && atThetaTarget) || forceShoot) {
      rumbleConsumer.accept(rumbleIntensity.get());
      if (externalConfirmation) {
        startOfShooting = Timer.getFPGATimestamp();
        return stateWithName("shoot", this::shoot);
      }
    }

    return null;
  }

  public StateHandler shoot() {
    rumbleConsumer.accept(0.0);

    updateSolver();
    rotateToSpeaker();

    shooter.markStartOfNoteShooting();
    shooter.setPivotPosition(desiredShootingPitch);
    // run kicker for X seconds
    shooter.setKickerPercentOut(shootingKickerPercent.get());
    endEffector.setPercentOut(0.5);

    if (!shooter.noteInShooter()) {
      Logger.recordOutput("ShooterScoreSpeaker/TimeToShoot",  Timer.getFPGATimestamp() - startOfShooting);
      return stateWithName("shootEnding", this::shootEnding);
    }
    return null;
  }

  public StateHandler shootEnding() {
    updateSolver();
    rotateToSpeaker();
    if (!stateRunningLongerThan(0.2)) return null;
    // visualize gamepiece

    return visualizeGamepiece();
  }

  public StateHandler visualizeGamepiece() {
    GamepieceVisualization.getInstance()
        .updateVisualization(
            drivetrainWrapper.getPoseEstimatorPose(),
            drivetrainWrapper.getFieldRelativeVelocities().getTranslation(),
            shooter.getPitch(),
            shooter.getRPM());

    return setDone();
  }

  private void rotateToSpeaker() {
    if (solverResult != null) {
      log("shooterSolver", true);

      double currentRotation = drivetrainWrapper.getRotation().getRadians();
      double omega =
          rotationController.calculate(currentRotation, desiredShootingHeading.getRadians());
      drivetrainWrapper.setRotationOverride(omega);

      log("omega", omega);
      log("rotationalError", Math.toDegrees(rotationController.getPositionError()));
    } else {
      log("shooterSolver", false);
    }

    log("headingDegreesEstimator", drivetrainWrapper.getPoseEstimatorPose().getRotation());
  }

  private boolean validShootingPosition() {
    Pose2d reflectedPose =
        AllianceFlipUtil.flipPoseForAlliance(drivetrainWrapper.getPoseEstimatorPose());

    double x = reflectedPose.getX();
    double y = reflectedPose.getY();

    // look at constraints: https://www.desmos.com/calculator/dvrwcfwnz8
    // check if shot is legal
    if ((DriverStation.isAutonomous() && x >= 6.2697529792785645) || x >= 10.257804870605469) {
      return false;
    }
    // y <= 0.808x + 0.793
    // y >= -0.64x + 6.1
    // y <= 6.103558540344238
    // check if stage is blocking
    if (y <= 0.808 * x + 0.3 && y >= -0.64 * x + 6.1 && y <= 6.103558540344238) {
      return false;
    }

    // check if distance is less than max shooting distance - REMOVED TO ALLOW FOR CLUTCH SHOTS
    // var maxDistance = Constants.ShooterConstants.MAX_SHOOTING_DISTANCE.in(Units.Meters);
    // if (solverResult == null || solverResult.xyDistance() > maxDistance) {
    //   return false;
    // }

    return true;
  }

  public void updateSolver() {
    solverResult =
        solver.computeAngles(
            Timer.getFPGATimestamp(),
            drivetrainWrapper.getPoseEstimatorPose(),
            drivetrainWrapper.getFieldRelativeVelocities().getTranslation());

    double offset = tunablePitchOffset.get();

    if (solverResult != null) {
      double distance = solverResult.xyDistance();
      if (Math.abs(offset) < 0.1) {
        offset = Constants.ShooterConstants.Pivot.getPitchOffset(Units.Meters.of(distance));
      }
      double desirePitchDegrees = solverResult.pitch().getDegrees() + offset;
      desiredShootingPitch =
          Rotation2d.fromDegrees(
              Math.min(
                  desirePitchDegrees, Constants.ShooterConstants.Pivot.MAX_ANGLE_RAD.getDegrees()));
      desiredShootingHeading = solverResult.heading();

      log("headingTargetDegrees", desiredShootingHeading);
      log("pitchTargetDegrees", desiredShootingPitch);
      log("pitchOffset", offset);
      log("xyDistanceFromSpeaker", distance);
    }
  }

  private static void log(String key, boolean value) {
    Logger.recordOutput("ShooterScoreSpeaker/" + key, value);
  }

  private static void log(String key, double value) {
    Logger.recordOutput("ShooterScoreSpeaker/" + key, value);
  }

  private static void log(String key, Rotation2d value) {
    log(key, value.getDegrees());
  }
}
