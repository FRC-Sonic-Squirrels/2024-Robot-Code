package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class EndEffectorIOInputs {
    public double RPM = 0.0;
    public double voltage = 0.0;
    public double tempCelsius = 0.0;
    public boolean beamBreak = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setPercentOut(double percent) {}

  public default void setVoltage(double volts) {}

  public default void setRPM(double RPM) {}
}
