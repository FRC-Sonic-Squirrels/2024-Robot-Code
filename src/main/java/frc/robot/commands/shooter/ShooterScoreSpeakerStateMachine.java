// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.ShootingSolver;
import frc.lib.team2930.StateMachine;
import frc.robot.Constants;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.BooleanSupplier;

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

    setInitialState(this::prepWhileLoadingGamepiece);
  }

  public static Command getAsCommand(
      DrivetrainWrapper drivetrainWrapper,
      Shooter shooter,
      EndEffector endEffector,
      double forceShotIn) {

    var command =
        new ShooterScoreSpeakerStateMachine(drivetrainWrapper, shooter, endEffector, forceShotIn)
            .asCommand();
    // FIXME: probably dont need this add requirments here once we add the asCommand(Subsytem...
    // requirments) method to the StateMachine class
    command.addRequirements(shooter, endEffector);

    // once statemachine is over, stop shooter and end effector subsystems
    command.andThen(new ShooterStowAndStopRollers(shooter).alongWith(endEffector.stopCmd()));
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

    // ramp up rpm
    // shooter.setLauncherRPM(ShooterConstants.SHOOTING_RPM);
    shooter.setLauncherVoltage(10.0);

    if (shooter.noteInShooter()) {
      shooter.setKickerPercentOut(0.0);
      endEffector.setPercentOut(0.0);
      return this::finalConfirmationsBeforeShooting;
    } else {
      // FIXME: look into calculating gear ratio to make it so kicker and EE spin at same speed
      shooter.setKickerPercentOut(0.1);
      endEffector.setPercentOut(0.1);
    }

    return null;
  }

  public StateHandler finalConfirmationsBeforeShooting() {
    // still be continuously aiming towards speaker
    // check launcher rpm correct
    // check pivot position correct
    // check robot theta correct
    // check external confirmation active
    // check are we in valid shooting position
    var launcherAtRpm = shooter.isAtTargetRPM();
    var pivotAtAngle = shooter.isPivotIsAtTarget();
    // var robotAtTheta = drivetrainWrapper.isAtTheta();
    var externalConfirmation = this.externalConfirmation.getAsBoolean();
    // var validShootingPosition = validPosition

    // FIXME: log each condition

    // FIXME: all all condition
    if (launcherAtRpm && pivotAtAngle && externalConfirmation) {
      return this::shoot;
    }

    return null;
  }

  public StateHandler shoot() {
    // run kicker for X seconds

    return visualizeGamepiece();
  }

  public StateHandler visualizeGamepiece() {
    // visualize gamepiece

    return setDone();
  }
}
