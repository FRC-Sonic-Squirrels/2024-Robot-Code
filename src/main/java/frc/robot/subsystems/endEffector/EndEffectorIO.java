package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class EndEffectorIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(EndEffectorIOInputs inputs) {}
}
