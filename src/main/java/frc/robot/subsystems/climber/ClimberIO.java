package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimberIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}
}
