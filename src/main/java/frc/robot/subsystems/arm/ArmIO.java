package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}
}
