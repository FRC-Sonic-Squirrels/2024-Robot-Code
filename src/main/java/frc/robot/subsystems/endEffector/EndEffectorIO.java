package frc.robot.subsystems.endEffector;

public interface EndEffectorIO {
  /** Contains all of the input data received from hardware. */
  public static class Inputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double intakeSideTOFDistanceInches = 0.0;
    public double shooterSideTOFDistanceInches = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double revPerMin) {}

  public default void markStartOfNoteIntaking() {}

  public default void markStartOfNoteDropping() {}

  public default void setClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {}
}
