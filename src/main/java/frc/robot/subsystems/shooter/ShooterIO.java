package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ShooterIOInputs {
    public Rotation2d pivotPosition = new Rotation2d();
    public double pivotVelocityRadsPerSec = 0.0;
    public double pivotAppliedVotls = 0.0;
    public double pivotCurrentAmps = 0.0;

    public double launcherRPM = 0.0;
    public double[] launcherAppliedVolts = new double[] {};
    public double[] launcherCurrentAmps = new double[] {};

    public double kickerAppliedVolts = 0.0;
    public double kickerCurrentAmps = 0.0;

    // launcher lead, launcher follower, pivot, kicker
    public double[] tempsCelcius = new double[] {};

    public boolean beamBreak = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  // PIVOT

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

  public default void setLauncherClosedLoopConstants(
      double kP, double kD, double maxProfiledVelocity, double maxProfiledAcceleration) {}

  // KICKER

  public default void setKickerVoltage(double volts) {}
}
