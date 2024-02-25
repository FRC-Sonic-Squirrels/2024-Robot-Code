package frc.lib.team2930.lib.controller_rumble;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class ControllerRumbleUntilButtonPress extends Command {
  private Consumer<Double> rumbleConsumer;
  private double rumbleStrength;
  private BooleanSupplier rumbleSupplier;

  /** Rumble the controller until the specified button is pressed */
  public ControllerRumbleUntilButtonPress(
      Consumer<Double> rumbleConsumer, BooleanSupplier buttonPressSupplier, double rumbleStrength) {
    this.rumbleConsumer = rumbleConsumer;
    this.rumbleStrength = rumbleStrength;
    this.rumbleSupplier = buttonPressSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rumbleConsumer.accept(rumbleStrength);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rumbleConsumer.accept(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rumbleSupplier.getAsBoolean();
  }
}
