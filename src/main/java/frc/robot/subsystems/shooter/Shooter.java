// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /** Creates a new ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/PitchDegrees", inputs.pitch.getDegrees());
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

  public void setRPM(double RPM) {
    io.setLauncherRPM(RPM);
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
  }

  public void setLauncherVoltage(double volts) {
    io.setLauncherVoltage(volts);
  }

  public void setLauncherPercentOut(double percent) {
    io.setLauncherPercentOut(percent);
  }

  public void setLauncherClosedLoopConstants(double kP, double kI, double kD) {
    io.setLauncherClosedLoopConstants(kP, kI, kD);
  }
}
