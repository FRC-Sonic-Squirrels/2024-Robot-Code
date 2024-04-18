// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.List;

public class SwerveModules {
  private final SwerveModule front_left;
  private final SwerveModule front_right;
  private final SwerveModule back_left;
  private final SwerveModule back_right;

  public SwerveModules(
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule backLeft,
      SwerveModule backRight) {
    front_left = frontLeft;
    front_right = frontRight;
    back_left = backLeft;
    back_right = backRight;
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      front_left.getState(), front_right.getState(), back_left.getState(), back_right.getState()
    };
  }

  public SwerveModuleState[] runSetpoints(
      SwerveModuleState[] setpointStates, double driveMotorMotionMagicAcceleration) {
    return new SwerveModuleState[] {
      front_left.runSetpoint(setpointStates[0], driveMotorMotionMagicAcceleration),
      front_right.runSetpoint(setpointStates[1], driveMotorMotionMagicAcceleration),
      back_left.runSetpoint(setpointStates[2], driveMotorMotionMagicAcceleration),
      back_right.runSetpoint(setpointStates[3], driveMotorMotionMagicAcceleration)
    };
  }

  public void runCharacterizationVolts(double volts) {
    front_left.runCharacterization(volts);
    front_right.runCharacterization(volts);
    back_left.runCharacterization(volts);
    back_right.runCharacterization(volts);
  }

  public double getCharacterizationVelocity() {
    double driveVelocityAverage =
        front_left.getCharacterizationVelocity()
            + front_right.getCharacterizationVelocity()
            + back_left.getCharacterizationVelocity()
            + back_right.getCharacterizationVelocity();

    return driveVelocityAverage / 4;
  }

  public double[] getCurrentDrawAmps() {
    double[] current = new double[8];
    front_left.getCurrentAmps(current, 0);
    front_right.getCurrentAmps(current, 2);
    back_left.getCurrentAmps(current, 4);
    back_right.getCurrentAmps(current, 6);
    return current;
  }

  public void registerSignalForOdometry(List<BaseStatusSignal> signals) {
    front_left.registerSignalForOdometry(signals);
    front_right.registerSignalForOdometry(signals);
    back_left.registerSignalForOdometry(signals);
    back_right.registerSignalForOdometry(signals);
  }

  public SwerveModulePosition[] updateOdometry() {
    return new SwerveModulePosition[] {
      front_left.updateOdometry(),
      front_right.updateOdometry(),
      back_left.updateOdometry(),
      back_right.updateOdometry()
    };
  }

  public void updateInputs() {
    front_left.updateInputs();
    front_right.updateInputs();
    back_left.updateInputs();
    back_right.updateInputs();
  }

  public void periodic() {
    front_left.periodic();
    front_right.periodic();
    back_left.periodic();
    back_right.periodic();
  }

  public void stop() {
    front_left.stop();
    front_right.stop();
    back_left.stop();
    back_right.stop();
  }

  public void reconfigureMotors() {
    front_left.reconfigureMotors();
    front_right.reconfigureMotors();
    back_left.reconfigureMotors();
    back_right.reconfigureMotors();
  }
}
