// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.PIDTargetMeasurement;
import frc.lib.team6328.LoggedTunableNumber;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static final String ROOT_TABLE = "Shooter";
  private static final LoggedTunableNumber kP = new LoggedTunableNumber(ROOT_TABLE + "/kP", 20.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber(ROOT_TABLE + "/kD", 1.0);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber(ROOT_TABLE + "/kG", 0.0);

  private static final LoggedTunableNumber closedLoopMaxVelocityConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/defaultClosedLoopMaxVelocityConstraint", 19.0);
  private static final LoggedTunableNumber closedLoopMaxAccelerationConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/defaultClosedLoopMaxAccelerationConstraint", 19.0);

  // Creates a new flat moving average filter
  // Average will be taken over the last 20 samples
  private LinearFilter pivotPidLatencyfilter = LinearFilter.movingAverage(20);

  private ArrayList<PIDTargetMeasurement> pivotTargetMeasurements =
      new ArrayList<PIDTargetMeasurement>();

  private Rotation2d currentTarget = new Rotation2d();

  private double pivotPidLatency = 0.0;

  /** Creates a new ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs(ROOT_TABLE, inputs);
    Logger.recordOutput(ROOT_TABLE + "/PitchDegrees", inputs.pitch.getDegrees());

    io.setPivotClosedLoopConstants(
        kP.get(),
        kD.get(),
        kG.get(),
        closedLoopMaxVelocityConstraint.get(),
        closedLoopMaxAccelerationConstraint.get());

    pivotTargetMeasurements.add(
        new PIDTargetMeasurement(
            Timer.getFPGATimestamp(),
            currentTarget,
            inputs.pitch.getRadians() <= currentTarget.getRadians()));
    for (int index = 0; index < pivotTargetMeasurements.size(); index++) {
      PIDTargetMeasurement measurement = pivotTargetMeasurements.get(index);
      if ((measurement.upDirection
              && inputs.pitch.getRadians() >= measurement.targetRot.getRadians())
          || (!measurement.upDirection
              && inputs.pitch.getRadians() <= measurement.targetRot.getRadians())) {
        pivotPidLatency =
            pivotPidLatencyfilter.calculate(Timer.getFPGATimestamp() - measurement.timestamp);
        pivotTargetMeasurements.remove(index);
      } else if (Timer.getFPGATimestamp() - measurement.timestamp >= 1.0) {
        pivotTargetMeasurements.remove(index);
      }
    }
    Logger.recordOutput(ROOT_TABLE + "/pivotPIDLatency", pivotPidLatency);
  }

  public void setPitchAngularVel(double radiansPerSecond) {
    io.setPivotVel(radiansPerSecond);
  }

  public Rotation2d getPitch() {
    return inputs.pitch;
  }

  public void setPercentOut(double percent) {
    io.setLauncherPercentOut(percent);
  }

  public void setPivotClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {
    io.setPivotClosedLoopConstants(kP, kD, kG, maxProfiledVelocity, maxProfiledAcceleration);
  }

  public void setPivotVoltage(double volts) {
    io.setPivotVoltage(volts);
  }

  public void setPivotPosition(Rotation2d rot) {
    io.setPivotPosition(rot);
    currentTarget = rot;
  }

  public void setLauncherVoltage(double volts) {
    io.setLauncherVoltage(volts);
  }

  public void setLauncherPercentOut(double percent) {
    io.setLauncherPercentOut(percent);
  }

  public double getRPM() {
    return inputs.RPM;
  }

  public double getPivotPIDLatency() {
    return pivotPidLatency;
  }
}
