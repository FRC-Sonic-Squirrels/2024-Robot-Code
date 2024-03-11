// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final TunableNumberGroup group = new TunableNumberGroup("Intake");

  private static final LoggedTunableNumber kS = group.build("kS");
  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kV = group.build("kV");
  private static final LoggedTunableNumber ClosedLoopMaxAccelerationConstraint =
      group.build("ClosedLoopMaxAccelerationConstraint");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_MAESTRO) {
      kP.initDefault(0.8);
      kV.initDefault(0.15);
      ClosedLoopMaxAccelerationConstraint.initDefault(300.0);
    } else if (Constants.RobotMode.isSimBot()) {

      kP.initDefault(0.0);
      kV.initDefault(0.0);
      ClosedLoopMaxAccelerationConstraint.initDefault(0.0);
    }
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    io.setClosedLoopConstants(
        kP.get(), kV.get(), kS.get(), ClosedLoopMaxAccelerationConstraint.get());
  }

  @Override
  public void periodic() {
    try (var ignored = new ExecutionTiming("Intake")) {
      // This method will be called once per scheduler run
      io.updateInputs(inputs);
      Logger.processInputs("Intake", inputs);

      var hc = hashCode();
      if (kS.hasChanged(hc)
          || kP.hasChanged(hc)
          || kV.hasChanged(hc)
          || ClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        io.setClosedLoopConstants(
            kP.get(), kV.get(), kS.get(), ClosedLoopMaxAccelerationConstraint.get());
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

  public boolean getBeamBreak() {
    return inputs.beamBreak;
  }

  public void setVelocity(double revPerMin) {
    io.setVelocity(revPerMin);
  }
}
