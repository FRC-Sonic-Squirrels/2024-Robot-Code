package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class IntakeIOInputs {
    public double velocityRPM = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double appliedVolts = 0.0;
    public boolean beamBreak = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
