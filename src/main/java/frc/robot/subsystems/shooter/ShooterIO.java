package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ShooterIOInputs {
    public Rotation2d pitch = new Rotation2d();
    public double pivotVoltage = 0.0;
    public double pivotTempCelsius = 0.0;

    public double RPM = 0.0;
    public double launcherLeadTempCelsius = 0.0;
    public double launcherFollowTempCelsius = 0.0;
    public double launcherVoltage = 0.0;

    public double kickerVoltage = 0.0;
    public double kickerRPM = 0.0;
    public double kickerTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  // PIVOT

  public default void setPivotVel(double radPerSec) {}

  public default void setPivotPosition(Rotation2d rot) {}

  public default void setPivotClosedLoopConstants(
      double kP,
      double kD,
      double kG,
      double maxProfiledVelocity,
      double maxProfiledAcceleration) {}

  public default void setPivotVoltage(double volts) {}

  // LAUNCHER

  public default void setLauncherVoltage(double volts) {}

  public default void setLauncherPercentOut(double percent) {}

  // KICKER

  public default void setKickerVoltage(double volts) {}
}
