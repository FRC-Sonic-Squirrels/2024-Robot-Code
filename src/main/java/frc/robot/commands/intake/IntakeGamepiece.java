// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Consumer;

public class IntakeGamepiece extends Command {
  private Intake intake;
  private EndEffector endEffector;

  private LoggedTunableNumber rumbleIntensityPercent =
      new LoggedTunableNumber("IntakeGamepiece/rumbleIntensityPercent", 0.3);

  private final Consumer<Double> rumbleConsumer;

  private final Trigger intakeSideTofSeenGamepiece;
  private final Trigger shooterSideTofSeenGamepiece;

  /** Creates a new IntakeDefaultIdleRPM. */
  public IntakeGamepiece(Intake intake, EndEffector endEffector, Consumer<Double> rumbleConsumer) {
    this.intake = intake;
    this.endEffector = endEffector;
    this.rumbleConsumer = rumbleConsumer;
    intakeSideTofSeenGamepiece =
        new Trigger(() -> endEffector.intakeSideTOFDetectGamepiece()).debounce(0.05);
    shooterSideTofSeenGamepiece =
        new Trigger(() -> endEffector.shooterSideTOFDetectGamepiece()).debounce(0.05);

    addRequirements(intake);
    setName("IntakeGamepiece");
  }

  public IntakeGamepiece(Intake intake, EndEffector endEffector) {
    this(intake, endEffector, (Double) -> {});
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (timeSinceLastGamepiece.get() >= 0.01
    //     && timeSinceLastGamepiece.get() <= rumbleDurationSeconds.get()) {
    //   if (controller != null)
    //     controller.setRumble(RumbleType.kBothRumble, rumbleIntensityPercent.get());
    //   controllerRumbled = true;
    // } else if (controllerRumbled) {
    //   if (controller != null) controller.setRumble(RumbleType.kBothRumble, 0.0);
    //   controllerRumbled = false;
    // }
    // intake.setPercentOut(Constants.IntakeConstants.INTAKE_IDLE_PERCENT_OUT);
    // if (endEffector.gamepieceInEndEffector()) {
    //   endEffector.setPercentOut(0.0);
    //   timeSinceLastGamepiece.start();
    // } else {
    //   endEffector.setPercentOut(0.8);
    // }

    var rumbleValue = rumbleIntensityPercent.get();
    if (!intakeSideTofSeenGamepiece.getAsBoolean() && !shooterSideTofSeenGamepiece.getAsBoolean()) {
      intake.setPercentOut(IntakeConstants.INTAKE_INTAKING_PERCENT_OUT);
      endEffector.setPercentOut(EndEffectorConstants.INTAKING_PERCENT_OUT);

      // dont rumble if we dont see gamepiece
      rumbleConsumer.accept(rumbleValue);

      // if we see the game piece slow down system to maintain better control over the note
    } else if (intakeSideTofSeenGamepiece.getAsBoolean()
        && !shooterSideTofSeenGamepiece.getAsBoolean()) {
      intake.setPercentOut(IntakeConstants.NOTE_IN_ROBOT_WHILE_INTAKING_PERCENT_OUT);
      endEffector.setPercentOut(EndEffectorConstants.NOTE_IN_ROBOT_WHILE_INTAKING_PERCENT_OUT);

    } else if (!intakeSideTofSeenGamepiece.getAsBoolean()
        && shooterSideTofSeenGamepiece.getAsBoolean()) {
      intake.setPercentOut(0.0);
      endEffector.setPercentOut(EndEffectorConstants.CENTERING_NOTE_REVERSE);

    } else if (intakeSideTofSeenGamepiece.getAsBoolean()
        && shooterSideTofSeenGamepiece.getAsBoolean()) {
      intake.setPercentOut(0.0);
      endEffector.setPercentOut(0.0);
    }

    rumbleConsumer.accept(rumbleValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPercentOut(0.0);
    endEffector.setPercentOut(0.0);

    rumbleConsumer.accept(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSideTofSeenGamepiece.getAsBoolean() && shooterSideTofSeenGamepiece.getAsBoolean();
  }
}
