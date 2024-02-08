// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.PIDTargetMeasurement;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final String ROOT_TABLE = "Shooter";
  private static final double MAX_VOLTAGE = 12;

  private static final LoggedTunableNumber pivotkP =
      new LoggedTunableNumber(ROOT_TABLE + "/pivotkP");
  private static final LoggedTunableNumber pivotkD =
      new LoggedTunableNumber(ROOT_TABLE + "/pivotkD");
  private static final LoggedTunableNumber pivotkG =
      new LoggedTunableNumber(ROOT_TABLE + "/pivotkG");
  private static final LoggedTunableNumber pivotClosedLoopMaxVelocityConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/pivotClosedLoopMaxVelocityConstraint");
  private static final LoggedTunableNumber pivotClosedLoopMaxAccelerationConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/pivotClosedLoopMaxAccelerationConstraint", 5.0);

  private static final LoggedTunableNumber launcherkP =
      new LoggedTunableNumber(ROOT_TABLE + "/launcherkP");
  private static final LoggedTunableNumber launcherkV =
      new LoggedTunableNumber(ROOT_TABLE + "/launcherkV");
  private static final LoggedTunableNumber launcherClosedLoopMaxAccelerationConstraint =
      new LoggedTunableNumber(ROOT_TABLE + "/launcherClosedLoopMaxAccelerationConstraint");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024) {
      pivotkP.initDefault(1.0);
      pivotkD.initDefault(0.0);
      pivotkG.initDefault(0.0);
      pivotClosedLoopMaxVelocityConstraint.initDefault(10.0);
      pivotClosedLoopMaxAccelerationConstraint.initDefault(10.0);

      launcherkP.initDefault(0.5);
      launcherkV.initDefault(0.13);
      launcherClosedLoopMaxAccelerationConstraint.initDefault(10.0);
    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {
      pivotkP.initDefault(1.0);
      pivotkD.initDefault(0.0);
      pivotkG.initDefault(0.0);
      pivotClosedLoopMaxVelocityConstraint.initDefault(10.0);
      pivotClosedLoopMaxAccelerationConstraint.initDefault(10.0);

      launcherkP.initDefault(0.5);
      launcherkV.initDefault(0.13);
      launcherClosedLoopMaxAccelerationConstraint.initDefault(10.0);
    }
  }

  private static final LoggedTunableNumber pivotToleranceDegrees =
      new LoggedTunableNumber(ROOT_TABLE + "/pivotToleranceDegrees", 0.5);

  private static final LoggedTunableNumber launcherToleranceRPM =
      new LoggedTunableNumber(ROOT_TABLE + "/launcherToleranceRPM", 20);

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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

    io.setLauncherClosedLoopConstants(
        launcherkP.get(), launcherkV.get(), launcherClosedLoopMaxAccelerationConstraint.get());

    io.setPivotClosedLoopConstants(
        pivotkP.get(),
        pivotkD.get(),
        pivotkG.get(),
        pivotClosedLoopMaxVelocityConstraint.get(),
        pivotClosedLoopMaxAccelerationConstraint.get());
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming("Shooter")) {
      io.updateInputs(inputs);

      Logger.processInputs(ROOT_TABLE, inputs);
      Logger.recordOutput(ROOT_TABLE + "/PitchDegrees", inputs.pivotPosition.getDegrees());

      pivotTargetMeasurements.add(
          new PIDTargetMeasurement(
              Timer.getFPGATimestamp(),
              currentTarget,
              inputs.pivotPosition.getRadians() <= currentTarget.getRadians()));
      for (int index = 0; index < pivotTargetMeasurements.size(); index++) {
        PIDTargetMeasurement measurement = pivotTargetMeasurements.get(index);
        if ((measurement.upDirection
                && inputs.pivotPosition.getRadians() >= measurement.targetRot.getRadians())
            || (!measurement.upDirection
                && inputs.pivotPosition.getRadians() <= measurement.targetRot.getRadians())) {
          pivotPidLatency =
              pivotPidLatencyfilter.calculate(Timer.getFPGATimestamp() - measurement.timestamp);
          pivotTargetMeasurements.remove(index);
        } else if (Timer.getFPGATimestamp() - measurement.timestamp >= 1.0) {
          pivotTargetMeasurements.remove(index);
        }
      }
      Logger.recordOutput(ROOT_TABLE + "/pivotPIDLatency", pivotPidLatency);

      var hc = hashCode();
      if (launcherkP.hasChanged(hc)
          || launcherkV.hasChanged(hc)
          || launcherClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        io.setLauncherClosedLoopConstants(
            launcherkP.get(), launcherkV.get(), launcherClosedLoopMaxAccelerationConstraint.get());
      }

      if (pivotkP.hasChanged(hc)
          || pivotkD.hasChanged(hc)
          || pivotkG.hasChanged(hc)
          || pivotClosedLoopMaxVelocityConstraint.hasChanged(hc)
          || pivotClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {

        io.setPivotClosedLoopConstants(
            pivotkP.get(),
            pivotkD.get(),
            pivotkG.get(),
            pivotClosedLoopMaxVelocityConstraint.get(),
            pivotClosedLoopMaxAccelerationConstraint.get());
      }
    }
  }

  public Rotation2d getPitch() {
    return inputs.pivotPosition;
  }

  public void setPercentOut(double percent) {
    io.setLauncherVoltage(percent * MAX_VOLTAGE);
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

  public void setLauncherRPM(double rpm) {
    io.setLauncherRPM(rpm);
  }

  public double getRPM() {
    return inputs.launcherRPM;
  }

  public boolean isAtShootingRPM() {
    return inputs.launcherRPM
        >= Constants.ShooterConstants.SHOOTING_RPM - launcherToleranceRPM.get();
  }

  public double getPivotPIDLatency() {
    return pivotPidLatency;
  }

  public boolean pivotIsAtTarget() {
    return pivotIsAtTarget(currentTarget);
  }

  public boolean pivotIsAtTarget(Rotation2d target) {
    return Math.abs(inputs.pivotPosition.getRadians() - target.getRadians())
        <= Units.degreesToRadians(pivotToleranceDegrees.get());
  }

  public void setKickerPercentOut(double percent) {
    io.setKickerVoltage(percent * Constants.MAX_VOLTAGE);
  }

  public double getKickerPercentOut() {
    return inputs.kickerAppliedVolts / Constants.MAX_VOLTAGE;
  }

  public boolean getBeamBreak() {
    return inputs.beamBreak;
  }
}
