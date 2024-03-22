package frc.robot.subsystems.intake;

public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  public static class Inputs {
    public double velocityRPM = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double appliedVolts = 0.0;
    public boolean beamBreak = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double revPerMin) {}

  public default void setClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {}
}
