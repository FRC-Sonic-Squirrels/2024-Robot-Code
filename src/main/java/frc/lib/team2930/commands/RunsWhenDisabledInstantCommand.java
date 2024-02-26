// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team2930.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

// Based on 2910 code
// https://github.com/FRCTeam2910/2023CompetitionRobot-Public/blob/7f38faa9afbd41e4372ae7e518dbff692a5abdeb/src/main/java/org/frcteam2910/c2023/util/DisabledInstantCommand.java#L12
public class RunsWhenDisabledInstantCommand extends InstantCommand {

  public RunsWhenDisabledInstantCommand(Runnable toRun, Subsystem... requirements) {
    super(toRun, requirements);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
