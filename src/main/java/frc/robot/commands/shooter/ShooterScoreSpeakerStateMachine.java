// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
  private static ShootingSolver solver =
      new ShootingSolver(
          Constants.FieldConstants.getSpeakerTranslation3D(),
          new Translation3d(0, 0, 0),
          new Translation3d(0, -10, 0),
          Constants.ShooterConstants.SHOOTING_SPEED,
          Constants.ShooterConstants.SHOOTING_TIME);

  private boolean loadGamepieceCommandFinished = false;

  private Timer timeSinceSeen = new Timer();

  private static final TunableNumberGroup group = new TunableNumberGroup("ShooterScoreSpeaker");

  private LoggedTunableNumber tunableRPM = group.build("tunableRPM", 8000.0);
  private LoggedTunableNumber tunableAngle = group.build("tunableAngle", 40.0);

  private final LoggedTunableNumber rotationKp = group.build("rotationKp", 4.9);
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

    Logger.recordOutput(
        "ShooterScoreSpeaker/headingTargetDegrees", solverResult.heading().getDegrees());
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
    double omega =
        rotationController.calculate(
            drivetrainWrapper.getRotation().getRadians(), solverResult.heading().getRadians());
    if (solverResult != null) {
      shooter.setPivotPosition(solverResult.pitch());
      pivotAtAngle = shooter.isPivotIsAtTarget(solverResult.pitch());
      rotateToSpeaker();
      logPositions();
    } else {
      pivotAtAngle = false;
    }
    Logger.recordOutput(
        "ShooterScoreSpeaker/rotationalError", rotationController.getPositionError());
    Logger.recordOutput("ShooterScoreSpeaker/omega", omega);

    // var robotAtTheta = drivetrainWrapper.isAtTheta();
    var externalConfirmation = this.externalConfirmation.getAsBoolean();
    // var validShootingPosition = validPosition

    // FIXME: log each condition

    // FIXME: all all condition
    if (launcherAtRpm && pivotAtAngle
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
    // if (!shooter.noteInShooter()) {

    //   shotTime = this.timeFromStart();
    //   return this::shootEnding;
    // }
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
    Logger.recordOutput(
        "ShooterScoreSpeaker/headingTargetDegrees", solverResult.heading().getDegrees());
    Logger.recordOutput(
        "ShooterScoreSpeaker/pitchTargetDegrees", solverResult.pitch().getDegrees());
    Logger.recordOutput(
        "ShooterScoreSpeaker/headingDegrees",
        drivetrainWrapper.getPoseEstimatorPose().getRotation().getDegrees());
    Logger.recordOutput("ShooterScoreSpeaker/pitchDegrees", shooter.getPitch().getDegrees());
  }

  private void rotateToSpeaker() {
    drivetrainWrapper.setRotationOverride(
        rotationController.calculate(
            drivetrainWrapper.getRotation().getRadians(), solverResult.heading().getRadians()));
  }
}