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

public class SwerveModules {
  public SwerveModule front_left;
  public SwerveModule front_right;
  public SwerveModule back_left;
  public SwerveModule back_right;

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
}
