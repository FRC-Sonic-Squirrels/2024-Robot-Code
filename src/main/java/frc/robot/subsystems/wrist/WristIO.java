package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class WristIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}
}
