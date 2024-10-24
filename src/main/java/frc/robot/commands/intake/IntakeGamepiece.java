// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Consumer;

public class IntakeGamepiece extends Command {
  private static final TunableNumberGroup group = new TunableNumberGroup("IntakeGamepiece");
  private static final LoggedTunableNumber rumbleIntensityPercent =
      group.build("rumbleIntensityPercent", 0.5);
  private static final LoggedTunableNumber intakingVelocity = group.build("intakingVelocity", 2500);

  private final Intake intake;
  private final EndEffector endEffector;
  private final Shooter shooter;
  private final Arm arm;
  private final Elevator elevator;

  private final Trigger noteInEEDebounced;

  private final Consumer<Double> rumbleConsumer;

  /** Creates a new IntakeDefaultIdleRPM. */
  public IntakeGamepiece(
      Intake intake,
      EndEffector endEffector,
      Shooter shooter,
      Arm arm,
      Elevator elevator,
      Consumer<Double> rumbleConsumer) {
    this.intake = intake;
    this.endEffector = endEffector;
    this.shooter = shooter;
    this.arm = arm;
    this.elevator = elevator;
    this.rumbleConsumer = rumbleConsumer;

    noteInEEDebounced = new Trigger(endEffector::noteInEndEffector).debounce(0.15);

    addRequirements(intake);
    setName("IntakeGamepiece");
  }

  public IntakeGamepiece(
      Intake intake, EndEffector endEffector, Shooter shooter, Arm arm, Elevator elevator) {
    this(intake, endEffector, shooter, arm, elevator, (ignore) -> {});
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var intakeSideTofSeenGamepiece = endEffector.intakeSideTOFDetectGamepiece();
    var shooterSideTofSeenGamepiece = endEffector.shooterSideTOFDetectGamepiece();
    var rumbleValue = rumbleIntensityPercent.get();

    if (!shooterSideTofSeenGamepiece) {
      shooter.setKickerPercentOut(0.0);

      if (!intakeSideTofSeenGamepiece) {

        if (arm.getAngle().getDegrees() > -88 || elevator.getHeightInches() > 8.0) {
          intake.setPercentOut(0.0);
          endEffector.setPercentOut(0.0);
        } else {
          intake.setVelocity(intakingVelocity.get());
          endEffector.setVelocity(intakingVelocity.get());
        }

        // dont rumble if we dont see gamepiece
        rumbleValue = 0.0;

        // if we see the game piece slow down system to maintain better control over the note
      } else {
        intake.setVelocity(intakingVelocity.get());
        endEffector.setVelocity(intakingVelocity.get());
      }
    } else {
      intake.setPercentOut(0.0);
      if (!intakeSideTofSeenGamepiece) {
        endEffector.setVelocity(-500);
        shooter.setKickerVelocity(-500);
      } else {
        endEffector.setPercentOut(0.0);
        shooter.setKickerPercentOut(0.0);
      }
    }

    rumbleConsumer.accept(rumbleValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPercentOut(0.0);
    endEffector.setPercentOut(0.0);
    shooter.setKickerPercentOut(0.0);

    rumbleConsumer.accept(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noteInEEDebounced.getAsBoolean();
  }
}
