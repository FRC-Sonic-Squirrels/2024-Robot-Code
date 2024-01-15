package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ShooterIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}
}
