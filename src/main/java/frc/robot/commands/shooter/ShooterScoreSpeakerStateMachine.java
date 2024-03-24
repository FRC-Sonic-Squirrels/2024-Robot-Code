// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private static final String ROOT_TABLE = "ShooterScoreSpeaker";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Bool log_cancelled = logGroup.buildBoolean("cancelled");
  private static final LoggerEntry.Decimal log_TimeToShoot = logGroup.buildDecimal("TimeToShoot");
  private static final LoggerEntry.Bool log_simpleShot = logGroup.buildBoolean("simpleShot");

  // --
  private static final LoggerGroup logGroupConfirmation = logGroup.subgroup("shootingConfimation");
  private static final LoggerEntry.Bool log_launcherAtRPM = logGroupConfirmation.buildBoolean("launcherAtRPM");
  private static final LoggerEntry.Bool log_pivotAtAngle = logGroupConfirmation.buildBoolean("pivotAtAngle");
  private static final LoggerEntry.Bool log_shootingPosition = logGroupConfirmation.buildBoolean("shootingPosition");
  private static final LoggerEntry.Bool log_driverConfirmation = logGroupConfirmation
      .buildBoolean("driverConfirmation");
  private static final LoggerEntry.Bool log_atThetaTarget = logGroupConfirmation.buildBoolean("atThetaTarget");
  private static final LoggerEntry.Bool log_belowMaxSpeed = logGroupConfirmation.buildBoolean("belowMaxSpeed");
  private static final LoggerEntry.Bool log_belowMaxRotVel = logGroupConfirmation.buildBoolean("belowMaxRotVel");
  private static final LoggerEntry.Bool log_shooterSolver = logGroupConfirmation.buildBoolean("shooterSolver");
  private static final LoggerEntry.Bool log_forceShoot = logGroupConfirmation.buildBoolean("forceShoot");

  // --
  private static final LoggerGroup logGroupData = logGroup.subgroup("data");
  private static final LoggerEntry.Decimal log_omega = logGroupData.buildDecimal("omega");
  private static final LoggerEntry.Decimal log_rotationalError = logGroupData.buildDecimal("rotationalError");
  private static final LoggerEntry.Decimal log_CurrentHeadingDegrees = logGroupData
      .buildDecimal("CurrentHeadingDegrees");
  private static final LoggerEntry.Decimal log_headingTargetDegrees = logGroupData.buildDecimal("headingTargetDegrees");
  private static final LoggerEntry.Decimal log_headingTolerance = logGroupData.buildDecimal("log_headingTolerance");
  private static final LoggerEntry.Decimal log_pitchTargetDegrees = logGroupData.buildDecimal("pitchTargetDegrees");
  private static final LoggerEntry.Decimal log_pitchOffset = logGroupData.buildDecimal("pitchOffset");
  private static final LoggerEntry.Decimal log_xyDistanceFromSpeaker = logGroupData
      .buildDecimal("xyDistanceFromSpeaker");
  private static final LoggerEntry.Decimal log_shooterPitchError = logGroupData.buildDecimal("log_shooterPitchError");

  // --
  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);
  private static final LoggedTunableNumber tunableRPM = group.build("tunableRPM", 8750.0);
  private static final LoggedTunableNumber tunablePitchOffset = group.build("tunablePitchOffset", 0.0);
  private static final LoggedTunableNumber rotationKp = group.build("rotationKp", 6.0);
  private static final LoggedTunableNumber rotationKd = group.build("rotationKd", 0.0);
  private static final LoggedTunableNumber rumbleIntensity = group.build("rumbleIntensity", 0.5);
  public static final LoggedTunableNumber loadingRPM = group.build("loading/LoadingRPM", 1200);
  public static final LoggedTunableNumber slowLoadingRPM = group.build("loading/slowLoadingRPM", 500);
  private static final LoggedTunableNumber shootingLoadingVelocity = group.build("shootingLoadingVelocity", 3000);
  private static final LoggedTunableNumber maxRotVel = group.build("maxRotVelDegPerSec", 10);

  private static final LoggedTunableNumber maxVel = group.build("maxVelMetersPerSec", 1);

  private final DrivetrainWrapper drivetrainWrapper;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final Intake intake;
  // such as driver confirmation, in auto this is always true
  private final Consumer<Double> rumbleConsumer;
  private final BooleanSupplier externalConfirmation;
  private final double forceShotIn;

  private final boolean simpleShot;
  private final Rotation2d simpleShotShooterPitch;

  private final Trigger noNoteInRobot;

  private double startOfShooting;

  // FIXME: are these constants correct?
  private final ShootingSolver solver = new ShootingSolver(
      Constants.FieldConstants.getSpeakerTranslation3D(),
      GeometryUtil.translation3dFromMeasures(
          Units.Inches.of(-1.0), Units.Inches.of(0), Units.Inches.of(9.0)),
      GeometryUtil.translation3dFromMeasures(
          Units.Inches.of(0), Units.Inches.of(-10), Units.Inches.of(0.0)),
      Constants.ShooterConstants.SHOOTING_SPEED,
      Constants.ShooterConstants.SHOOTING_TIME,
      true);

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
    this(
        drivetrainWrapper,
        shooter,
        endEffector,
        intake,
        forceShotIn,
        externalConfirmation,
        rumbleConsumer,
        false,
        Constants.zeroRotation2d);
  }

  public ShooterScoreSpeakerStateMachine(
      DrivetrainWrapper drivetrainWrapper,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      double forceShotIn,
      BooleanSupplier externalConfirmation,
      Consumer<Double> rumbleConsumer,
      boolean simpleShot,
      Rotation2d shooterPitch) {
    super("ShooterScoreSpeaker");

    this.drivetrainWrapper = drivetrainWrapper;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.intake = intake;
    this.rumbleConsumer = rumbleConsumer;
    this.externalConfirmation = externalConfirmation;
    this.forceShotIn = forceShotIn;
    this.simpleShot = simpleShot;
    this.simpleShotShooterPitch = shooterPitch;

    rotationController = new PIDController(rotationKp.get(), 0, rotationKd.get());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    noNoteInRobot = new Trigger(
        () -> !(endEffector.shooterSideTOFDetectGamepiece()
            || endEffector.intakeSideTOFDetectGamepiece()
            || shooter.getToFActivated()))
        .debounce(0.2);

    setInitialState(stateWithName("prepWhileLoadingGamepiece", this::prepWhileLoadingGamepiece));
  }

  public static Command getAsCommand(
      DrivetrainWrapper drivetrainWrapper,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      double forceShotIn,
      BooleanSupplier externalSupplier,
      Consumer<Double> rumbleConsumer,
      boolean simpleShot,
      Rotation2d shooterPitch) {

    Command command = new RunStateMachineCommand(
        () -> new ShooterScoreSpeakerStateMachine(
            drivetrainWrapper,
            shooter,
            endEffector,
            intake,
            forceShotIn,
            externalSupplier,
            rumbleConsumer,
            simpleShot,
            shooterPitch),
        shooter,
        endEffector);

    // FIXME: probably dont need this add requirments here once we add the
    // asCommand(Subsytem...
    // requirments) method to the StateMachine class
    command.addRequirements(shooter, endEffector, intake);
    command.setName("ShooterScoreSpeakerStateMachine");

    // once statemachine is over, stop shooter and end effector subsystems
    command = command.finallyDo(
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
      double forceShotIn,
      BooleanSupplier externalSupplier,
      Consumer<Double> rumbleConsumer) {
    return getAsCommand(
        drivetrainWrapper,
        shooter,
        endEffector,
        intake,
        forceShotIn,
        externalSupplier,
        rumbleConsumer,
        false,
        Constants.zeroRotation2d);
  }

  public static Command getAsCommand(
      DrivetrainWrapper drivetrainWrapper,
      Shooter shooter,
      EndEffector endEffector,
      Intake intake,
      double forceShotIn) {
    return getAsCommand(
        drivetrainWrapper,
        shooter,
        endEffector,
        intake,
        forceShotIn,
        () -> true,
        (ignore) -> {
        },
        false,
        Constants.zeroRotation2d);
  }

  public StateHandler prepWhileLoadingGamepiece() {
    // aim towards speaker

    updateSolver();
    rotateToSpeaker();
    log_cancelled.info(false);
    if (noNoteInRobot.getAsBoolean()) {
      log_cancelled.info(true);
      return setDone();
    }

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

    // FIXME: look into calculating gear ratio to make it so kicker and EE spin at
    // same speed
    shooter.setPivotPosition(Constants.ShooterConstants.Pivot.MIN_ANGLE_RAD);
    if (shooter.isPivotIsAtTarget(Constants.ShooterConstants.Pivot.MIN_ANGLE_RAD)) {
      shooter.markStartOfNoteLoading();
      if (endEffector.intakeSideTOFDetectGamepiece()) {
        shooter.setKickerVelocity(loadingRPM.get());
        endEffector.markStartOfNoteDropping();
        endEffector.setVelocity(loadingRPM.get());
        intake.setVelocity(loadingRPM.get());
      } else {
        shooter.setKickerVelocity(slowLoadingRPM.get());
        endEffector.setVelocity(slowLoadingRPM.get());
        intake.setVelocity(slowLoadingRPM.get());
      }
    }

    if (timeFromStartOfState() > 1.0 && endEffector.getRPM() < 20.0) {
      return stateWithName("loadingPause", this::loadingPause);
    }

    return null;
  }

  public StateHandler loadingPause() {
    updateSolver();
    rotateToSpeaker();
    shooter.setKickerPercentOut(0.0);
    endEffector.setPercentOut(0.0);
    intake.setPercentOut(0.0);
    if (timeFromStartOfState() < 0.3) {
      return null;
    }
    return stateWithName("prepWhileLoadingGamepiece", this::prepWhileLoadingGamepiece);
  }

  public StateHandler finalConfirmationsBeforeShooting() {
    updateSolver();

    // still be continuously aiming towards speaker
    // check launcher rpm correct
    // check pivot position correct
    // check robot theta correct
    // check external confirmation active
    // check are we in valid shooting position
    // check if we are below max vel

    double maxVelMetersPerSec = maxVel.get();
    boolean belowMaxSpeed = drivetrainWrapper
        .getFieldRelativeVelocities()
        .getTranslation()
        .getDistance(new Translation2d()) <= maxVelMetersPerSec;

    double maxRotVelRads = Math.toRadians(maxRotVel.get());
    boolean belowMaxRotVel = drivetrainWrapper.getFieldRelativeVelocities().getRotation().getRadians() <= maxRotVelRads;

    boolean pivotAtAngle;
    var launcherAtRpm = shooter.isAtTargetRPM();
    if (solverResult != null) {
      shooter.setPivotPosition(desiredShootingPitch);
      pivotAtAngle = shooter.isPivotIsAtTarget(
          desiredShootingPitch
      // , getPitchTolerance(Units.Meters.of(solverResult.xyDistance()))
      );
    } else {
      pivotAtAngle = false;
    }

    rotateToSpeaker();

    var externalConfirmation = this.externalConfirmation.getAsBoolean();
    boolean atThetaTarget;

    if (simpleShot) {
      atThetaTarget = true;
    } else {
      if (solverResult != null) {
        var thetaError = desiredShootingHeading.getRadians()
            - drivetrainWrapper.getRotationGyroOnly().getRadians();

        thetaError = GeometryUtil.optimizeRotation(thetaError);
        double tolerance = getYawTolerance(solverResult.xyOffset().toTranslation2d()).getRadians();
        log_headingTolerance.info(Math.toDegrees(tolerance));
        atThetaTarget = Math.abs(thetaError) <= Math.toRadians(2.0);
      } else {
        atThetaTarget = false;
      }
    }

    var forceShoot = stateRunningLongerThan(forceShotIn);

    log_simpleShot.info(simpleShot);

    log_launcherAtRPM.info(launcherAtRpm);
    log_pivotAtAngle.info(pivotAtAngle);
    log_shootingPosition.info(validShootingPosition());
    log_forceShoot.info(forceShoot);
    log_driverConfirmation.info(externalConfirmation);
    log_atThetaTarget.info(atThetaTarget);
    log_belowMaxSpeed.info(belowMaxSpeed);
    log_belowMaxRotVel.info(belowMaxRotVel);

    if ((launcherAtRpm
        && pivotAtAngle
        && validShootingPosition()
        && atThetaTarget
        && belowMaxSpeed
        && belowMaxRotVel)
        || forceShoot) {
      rumbleConsumer.accept(rumbleIntensity.get());
      if (externalConfirmation) {
        startOfShooting = Timer.getFPGATimestamp();
        solver.startShooting(startOfShooting);
        shooter.markStartOfNoteShooting();
        return stateWithName("shoot", this::shoot);
      }
    }

    return null;
  }

  public StateHandler shoot() {
    rumbleConsumer.accept(0.0);

    updateSolver();
    rotateToSpeaker();

    shooter.setPivotPosition(desiredShootingPitch);
    // run kicker for X seconds
    shooter.setKickerVelocity(shootingLoadingVelocity.get());
    endEffector.setVelocity(shootingLoadingVelocity.get());

    if (!shooter.noteInShooter()) {
      return stateWithName("shootEnding", this::shootEnding);
    }
    return null;
  }

  public StateHandler shootEnding() {
    updateSolver();
    rotateToSpeaker();
    if (!stateRunningLongerThan(0.2))
      return null;
    // visualize gamepiece

    log_TimeToShoot.info(Timer.getFPGATimestamp() - startOfShooting);
    return visualizeGamepiece();
  }

  public StateHandler visualizeGamepiece() {
    GamepieceVisualization.getInstance()
        .updateVisualization(
            drivetrainWrapper.getPoseEstimatorPose(false),
            drivetrainWrapper.getFieldRelativeVelocities().getTranslation(),
            shooter.getPitch(),
            shooter.getRPM());

    return setDone();
  }

  private void rotateToSpeaker() {
    if (solverResult != null) {
      log_shooterSolver.info(true);

      double currentRotation = drivetrainWrapper.getRotationGyroOnly().getRadians();
      double omega = rotationController.calculate(currentRotation, desiredShootingHeading.getRadians());
      if (!simpleShot) {
        drivetrainWrapper.setRotationOverride(omega);
      }

      log_omega.info(omega);
      log_rotationalError.info(Math.toDegrees(rotationController.getPositionError()));
    } else {
      log_shooterSolver.info(false);
    }

    log_CurrentHeadingDegrees.info(drivetrainWrapper.getRotationGyroOnly().getDegrees());
  }

  private boolean validShootingPosition() {
    if (simpleShot)
      return true;
    Pose2d reflectedPose = AllianceFlipUtil.flipPoseForAlliance(drivetrainWrapper.getPoseEstimatorPose(false));

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

    // check if distance is less than max shooting distance - REMOVED TO ALLOW FOR
    // CLUTCH SHOTS IN
    // TELEOP
    var maxDistance = Constants.ShooterConstants.MAX_SHOOTING_DISTANCE.in(Units.Meters);
    if (solverResult == null
        || solverResult.xyDistance() > maxDistance && DriverStation.isAutonomous()) {
      return false;
    }

    return true;
  }

  public void updateSolver() {
    solverResult = solver.computeAngles(
        Timer.getFPGATimestamp(),
        drivetrainWrapper.getPoseEstimatorPose(false),
        drivetrainWrapper.getFieldRelativeVelocities().getTranslation());

    double offset = tunablePitchOffset.get();

    if (solverResult != null) {
      double distance = solverResult.xyDistance();
      if (Math.abs(offset) < 0.1) {
        offset = Constants.ShooterConstants.Pivot.getPitchOffset(Units.Meters.of(distance));
      }
      double desirePitchDegrees = solverResult.pitch().getDegrees() + offset;
      desiredShootingPitch = simpleShot
          ? simpleShotShooterPitch
          : Rotation2d.fromDegrees(
              Math.min(
                  desirePitchDegrees,
                  Constants.ShooterConstants.Pivot.MAX_ANGLE_RAD.getDegrees()));
      desiredShootingHeading = solverResult.heading();

      log_headingTargetDegrees.info(desiredShootingHeading);
      log_pitchTargetDegrees.info(desiredShootingPitch);
      log_pitchOffset.info(offset);
      log_xyDistanceFromSpeaker.info(distance);
      double pivotError = desiredShootingPitch.getDegrees() - shooter.getPitch().getDegrees();
      log_shooterPitchError.info(pivotError);
    }
  }

  private Rotation2d getPitchTolerance(Measure<Distance> distance) {
    return Rotation2d.fromRadians(
        Math.max(
            Math.atan2(
                Constants.FieldConstants.SPEAKER_HEIGHT.in(Units.Meters)
                    + Constants.FieldConstants.SPEAKER_GOAL_HEIGHT.in(Units.Meters),
                distance.in(Units.Meters)
                    - Constants.FieldConstants.SPEAKER_GOAL_LENGTH.in(Units.Meters))
                - Math.atan2(
                    Constants.FieldConstants.SPEAKER_HEIGHT.in(Units.Meters),
                    distance.in(Units.Meters))
                - Math.toRadians(0.3),
            Math.toRadians(0.2)));
  }

  private Rotation2d getYawTolerance(Translation2d offset) {
    return Rotation2d.fromRadians(Math.max(Math.abs(
        Math.atan2(offset.getY() - Constants.FieldConstants.SPEAKER_GOAL_WIDTH.in(Units.Meters) / 2.0 + Constants.FieldConstants.Gamepieces.NOTE_OUTER_RADIUS.in(Units.Meters) / 2.0, offset.getX())
            - Math.atan2(offset.getY() + Constants.FieldConstants.SPEAKER_GOAL_WIDTH.in(Units.Meters) / 2.0 - Constants.FieldConstants.Gamepieces.NOTE_OUTER_RADIUS.in(Units.Meters) / 2.0,
                offset.getX())) - 1.0, 0.5));
  }
}
