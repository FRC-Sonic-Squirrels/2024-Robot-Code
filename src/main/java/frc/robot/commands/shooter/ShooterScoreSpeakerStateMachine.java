// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.RunStateMachineCommand;
import frc.lib.team2930.ShootingSolver;
import frc.lib.team2930.ShootingSolver.Solution;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShooterScoreSpeakerStateMachine extends StateMachine {
  private final DrivetrainWrapper drivetrainWrapper;
  private final Shooter shooter;
  private final EndEffector endEffector;
  // such as driver confirmation, in auto this is always true
  private final BooleanSupplier externalConfirmation;
  private final double forceShotIn;

  // FIXME: are these constants correct?
  private ShootingSolver solver =
      new ShootingSolver(
          Constants.FieldConstants.getSpeakerTranslation3D(),
          new Translation3d(Units.inchesToMeters(-1.0), 0, Units.inchesToMeters(9.0)),
          new Translation3d(0, -10, 0),
          Constants.ShooterConstants.SHOOTING_SPEED,
          Constants.ShooterConstants.SHOOTING_TIME);

  private boolean loadGamepieceCommandFinished = false;

  private LoggedTunableNumber neverShoot = group.build("neverShoot", 0);

  private static final TunableNumberGroup group = new TunableNumberGroup("ShooterScoreSpeaker");

  private LoggedTunableNumber tunableRPM = group.build("tunableRPM", 8000.0);
  private LoggedTunableNumber tunableAngle = group.build("tunableAngle", 40.0);

  private final LoggedTunableNumber rotationKp = group.build("rotationKp", 6.0);
  private final LoggedTunableNumber rotationKd = group.build("rotationKd", 0.0);

  private final PIDController rotationController;

  private double shotTime;

  private Solution solverResult;

  public ShooterScoreSpeakerStateMachine(
      DrivetrainWrapper drivetrainWrapper,
      Shooter shooter,
      EndEffector endEffector,
      double forceShotIn) {
    this(drivetrainWrapper, shooter, endEffector, forceShotIn, () -> true);
  }

  public ShooterScoreSpeakerStateMachine(
      DrivetrainWrapper drivetrainWrapper,
      Shooter shooter,
      EndEffector endEffector,
      double forceShotIn,
      BooleanSupplier externalConfirmation) {
    this.drivetrainWrapper = drivetrainWrapper;
    this.shooter = shooter;
    this.endEffector = endEffector;
    this.externalConfirmation = externalConfirmation;
    this.forceShotIn = forceShotIn;
    rotationController = new PIDController(rotationKp.get(), 0, rotationKd.get());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    Logger.recordOutput("ShooterScoreSpeaker/state", 0);
    setInitialState(this::prepWhileLoadingGamepiece);
  }

  public static Command getAsCommand(
      DrivetrainWrapper drivetrainWrapper,
      Shooter shooter,
      EndEffector endEffector,
      double forceShotIn) {

    Command command =
        new RunStateMachineCommand(
            () ->
                new ShooterScoreSpeakerStateMachine(
                    drivetrainWrapper, shooter, endEffector, forceShotIn),
            shooter,
            endEffector);

    // FIXME: probably dont need this add requirments here once we add the asCommand(Subsytem...
    // requirments) method to the StateMachine class
    command.addRequirements(shooter, endEffector);

    // once statemachine is over, stop shooter and end effector subsystems
    command =
        command.finallyDo(
            () -> {
              shooter.setKickerPercentOut(0.0);
              endEffector.setPercentOut(0.0);
              shooter.setLauncherVoltage(0.0);
              shooter.setPivotPosition(Constants.ShooterConstants.Pivot.HOME_POSITION);
              drivetrainWrapper.resetRotationOverride();
            });

    return command;
  }

  //   public StateHandler spawnLoadShooterCommand() {
  //     spawnCommand(
  //         new ShooterLoadFromEndEffector().withTimeout(1.5),
  //         (ignore) -> {
  //           loadGamepieceCommandFinished = true;
  //           return null;
  //         });

  //     return this::prepWhileLoadingGamepiece;
  //   }

  public StateHandler prepWhileLoadingGamepiece() {
    // aim towards speaker
    Logger.recordOutput("ShooterScoreSpeaker/state", 1);

    solverResult =
        solver.computeAngles(
            Timer.getFPGATimestamp(),
            drivetrainWrapper.getPoseEstimatorPose(),
            drivetrainWrapper.getFieldRelativeVelocities().getTranslation());
    rotateToSpeaker();

    if (solverResult != null) {
      Logger.recordOutput(
          "ShooterScoreSpeaker/headingTargetDegrees", solverResult.heading().getDegrees());
    }
    Logger.recordOutput(
        "ShooterScoreSpeaker/headingDegrees",
        drivetrainWrapper.getPoseEstimatorPose().getRotation().getDegrees());

    // ramp up rpm
    shooter.setLauncherRPM(
        // ShooterConstants.SHOOTING_RPM
        tunableRPM.get());
    // shooter.setLauncherVoltage(10.0);

    if (shooter.noteInShooter()) {
      shooter.setKickerPercentOut(0.0);
      endEffector.setPercentOut(0.0);
      return this::finalConfirmationsBeforeShooting;
    } else {
      // FIXME: look into calculating gear ratio to make it so kicker and EE spin at same speed
      shooter.setPivotPosition(Constants.ShooterConstants.Pivot.MIN_ANGLE_RAD);
      if (shooter.isPivotIsAtTarget(Constants.ShooterConstants.Pivot.MIN_ANGLE_RAD)) {
        shooter.setKickerPercentOut(0.1);
        endEffector.setPercentOut(0.1);
      }
    }

    return null;
  }

  public StateHandler finalConfirmationsBeforeShooting() {
    Logger.recordOutput("ShooterScoreSpeaker/state", 2);
    solverResult =
        solver.computeAngles(
            Timer.getFPGATimestamp(),
            drivetrainWrapper.getPoseEstimatorPose(),
            drivetrainWrapper.getFieldRelativeVelocities().getTranslation());

    // still be continuously aiming towards speaker
    // check launcher rpm correct
    // check pivot position correct
    // check robot theta correct
    // check external confirmation active
    // check are we in valid shooting position
    boolean pivotAtAngle;
    var launcherAtRpm = shooter.isAtTargetRPM();
    if (solverResult != null) {
      double omega =
          rotationController.calculate(
              drivetrainWrapper.getRotation().getRadians(), solverResult.heading().getRadians());

      shooter.setPivotPosition(solverResult.pitch());
      pivotAtAngle = shooter.isPivotIsAtTarget(solverResult.pitch());
      logPositions();
      Logger.recordOutput("ShooterScoreSpeaker/omega", omega);
    } else {
      pivotAtAngle = false;
    }

    rotateToSpeaker();

    // var robotAtTheta = drivetrainWrapper.isAtTheta();
    var externalConfirmation = this.externalConfirmation.getAsBoolean();
    boolean atThetaTarget = false;

    if(solverResult != null){
      var thetaError = solverResult.heading().getRadians() - drivetrainWrapper.getRotation().getRadians();
      while (thetaError <= -Math.PI) thetaError += Math.PI * 2;
      while (thetaError >= Math.PI) thetaError -= Math.PI * 2;
      atThetaTarget = Math.abs(thetaError) <= Math.toRadians(0.5); 
    }

    // FIXME: log each condition

    // FIXME: all all condition
    Logger.recordOutput("ShooterScoreSpeaker/launcherAtRPM", launcherAtRpm);
    Logger.recordOutput("ShooterScoreSpeaker/pivotAtAngle", pivotAtAngle);
    Logger.recordOutput("ShooterScoreSpeaker/shootingPosition", shootingPosition());
    Logger.recordOutput("ShooterScoreSpeaker/atThetaTarget", atThetaTarget);  
    if ((launcherAtRpm && pivotAtAngle && shootingPosition() && atThetaTarget) || timeFromStart() >= forceShotIn
    // && externalConfirmation
    ) {

      return this::shoot;
    }

    return null;
  }

  public StateHandler shoot() {
    Logger.recordOutput("ShooterScoreSpeaker/state", 3);
    solverResult =
        solver.computeAngles(
            Timer.getFPGATimestamp(),
            drivetrainWrapper.getPoseEstimatorPose(),
            drivetrainWrapper.getFieldRelativeVelocities().getTranslation());
    rotateToSpeaker();
    logPositions();
    shooter.setPivotPosition(solverResult.pitch());
    // run kicker for X seconds
    shooter.setKickerPercentOut(0.5);

    if (!shooter.noteInShooter()) {

      shotTime = this.timeFromStart();
      return this::shootEnding;
    }
    return null;
  }

  public StateHandler shootEnding() {
    Logger.recordOutput("ShooterScoreSpeaker/state", 4);
    if (timeFromStart() < shotTime + 0.2) return null;
    // visualize gamepiece

    return visualizeGamepiece();
  }

  public StateHandler visualizeGamepiece() {
    Logger.recordOutput("ShooterScoreSpeaker/state", 5);
    // visualize gamepiece

    return setDone();
  }

  private void logPositions() {
    if (solverResult != null) {
      Logger.recordOutput(
          "ShooterScoreSpeaker/headingTargetDegrees", solverResult.heading().getDegrees());
      Logger.recordOutput(
          "ShooterScoreSpeaker/pitchTargetDegrees", solverResult.pitch().getDegrees());
      Logger.recordOutput(
          "ShooterScoreSpeaker/headingDegrees",
          drivetrainWrapper.getPoseEstimatorPose().getRotation().getDegrees());
      Logger.recordOutput("ShooterScoreSpeaker/pitchDegrees", shooter.getPitch().getDegrees());
      
    }
  }

  private void rotateToSpeaker() {
    if (solverResult != null) {
      drivetrainWrapper.setRotationOverride(
          rotationController.calculate(
              drivetrainWrapper.getRotation().getRadians(), solverResult.heading().getRadians()));
      Logger.recordOutput("ShooterScoreSpeaker/shooterSolver", true);
    } else {
      Logger.recordOutput("ShooterScoreSpeaker/shooterSolver", false);
    }
    Logger.recordOutput(
        "ShooterScoreSpeaker/rotationalError", Math.toDegrees(rotationController.getPositionError()));
  }

  private boolean shootingPosition() {
    Pose2d currentPose = drivetrainWrapper.getPoseEstimatorPose();
    Pose2d reflectedPose =
        Constants.isRedAlliance()
            ? AllianceFlipUtil.mirrorPose2DOverCenterLine(currentPose)
            : currentPose;
    // look at constraints: https://www.desmos.com/calculator/dvrwcfwnz8
    // check if shot is legal
    if ((DriverStation.isAutonomous() && reflectedPose.getX() >= 6.2697529792785645)
        || reflectedPose.getX() >= 10.257804870605469) {
      return false;
    }
    // y <= 0.808x + 0.793
    // y >= -0.64x + 6.1
    // y <= 6.103558540344238
    // check if stage is blocking
    if (reflectedPose.getY() <= 0.808 * reflectedPose.getX() + 0.3
        && reflectedPose.getY() >= -0.64 * reflectedPose.getX() + 6.1
        && reflectedPose.getY() <= 6.103558540344238) {
      return false;
    }
    return true;
  }

  public StateHandler skipToShootIfForceShot() {
    return timeFromStart() >= forceShotIn ? this::shoot : null;
  }
}
