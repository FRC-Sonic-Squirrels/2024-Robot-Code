// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;

public class Intake extends SubsystemBase {
  public static final String ROOT_TABLE = "Intake";

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  private static final LoggerGroup logInputs = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_velocityRPM =
      logInputs.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal logInputs_currentAmps =
      logInputs.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_tempCelsius =
      logInputs.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_appliedVolts =
      logInputs.buildDecimal("AppliedVolts");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  private static final LoggedTunableNumber kS = group.build("kS");
  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kV = group.build("kV");
  private static final LoggedTunableNumber maxProfiledAcceleration =
      group.build("ClosedLoopMaxAccelerationConstraint");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_MAESTRO) {
      kP.initDefault(0.8);
      kV.initDefault(0.15);
      maxProfiledAcceleration.initDefault(300.0);
    } else if (Constants.RobotMode.isSimBot()) {

      kP.initDefault(0.0);
      kV.initDefault(0.0);
      maxProfiledAcceleration.initDefault(0.0);
    }
  }

  private final IntakeIO io;
  private final IntakeIO.Inputs inputs = new IntakeIO.Inputs();

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    io.setClosedLoopConstants(kP.get(), kV.get(), kS.get(), maxProfiledAcceleration.get());
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // This method will be called once per scheduler run
      io.updateInputs(inputs);
      logInputs_velocityRPM.info(inputs.velocityRPM);
      logInputs_currentAmps.info(inputs.currentAmps);
      logInputs_tempCelsius.info(inputs.tempCelsius);
      logInputs_appliedVolts.info(inputs.appliedVolts);

      var hc = hashCode();
      if (kS.hasChanged(hc)
          || kP.hasChanged(hc)
          || kV.hasChanged(hc)
          || maxProfiledAcceleration.hasChanged(hc)) {
        io.setClosedLoopConstants(kP.get(), kV.get(), kS.get(), maxProfiledAcceleration.get());
      }
    }
  }

  public double getCurrentDraw() {
    return inputs.currentAmps;
  }

  public void setPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE);
  }

  public double getRPM() {
    return inputs.velocityRPM;
  }

  public void setVelocity(double revPerMin) {
    io.setVelocity(revPerMin);
  }
}
