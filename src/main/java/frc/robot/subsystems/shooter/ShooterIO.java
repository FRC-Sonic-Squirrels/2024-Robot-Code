package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ShooterIOInputs {
    public Rotation2d pivotPosition = Constants.zeroRotation2d;
    public double pivotVelocityRadsPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    public double[] launcherRPM = new double[] {};
    public double[] launcherAppliedVolts = new double[] {};
    public double[] launcherCurrentAmps = new double[] {};

    public double kickerAppliedVolts = 0.0;
    public double kickerCurrentAmps = 0.0;

    // launcher lead, launcher follower, pivot, kicker
    public double[] tempsCelcius = new double[] {};

    public double timeOfFlightDistance = 18.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  // PIVOT
  public default void setPivotPosition(Rotation2d rot) {}

  public default void setPivotVoltage(double volts) {}

  public default void resetPivotSensorPosition(Rotation2d position) {}

  public default void setPivotClosedLoopConstants(
      double kP,
      double kD,
      double kG,
      double maxProfiledVelocity,
      double maxProfiledAcceleration) {}

  // LAUNCHER
  public default void setLauncherVoltage(double volts) {}

  public default void setLauncherRPM(double rpm) {}

  public default void setLauncherClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {}

  // KICKER
  public default void setKickerVoltage(double volts) {}

  public default void setNeutralMode(NeutralModeValue value) {}

  public default void markStartOfNoteLoading() {}

  public default void markStartOfNoteShooting() {}
}
