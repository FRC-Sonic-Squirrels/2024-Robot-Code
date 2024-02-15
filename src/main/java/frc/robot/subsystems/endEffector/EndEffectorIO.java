package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class EndEffectorIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double intakeSideTOFDistanceInches = 0.0;
    public double shooterSideTOFDistanceInches = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
