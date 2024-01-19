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
    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setPitchAngularVel(double radiansPerSecond) {}

  public Rotation2d getPitch() {
    return new Rotation2d();
  }

  public void setPercentOut(double percent) {}

  public void setRPM(double RPM) {}
}
