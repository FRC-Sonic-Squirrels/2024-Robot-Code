package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.team2930.ControlMode;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getFalcon500Foc(2),
          Constants.ElevatorConstants.GEAR_RATIO,
          Constants.ElevatorConstants.CARRIAGE_MASS,
          Constants.ElevatorConstants.WHEEL_RADIUS,
          0.0,
          Constants.ElevatorConstants.MAX_HEIGHT,
          true,
          0.1);

  private double targetHeight = 0.0;

  private final ProfiledPIDController feedback =
      new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0));

  private double kG = 0.0;

  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  private double openLoopVolts = 0.0;

  private double appliedVolts = 0.0;

  public ElevatorIOSim() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (controlMode.equals(ControlMode.CLOSED_LOOP)) {
      appliedVolts = feedback.calculate(inputs.heightInches, targetHeight) + kG;
    } else {
      appliedVolts = openLoopVolts;
    }

    Logger.recordOutput("Elevator/controlMode", controlMode);

    inputs.voltage = appliedVolts;
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);
    inputs.heightInches = Units.metersToInches(sim.getPositionMeters());
  }

  @Override
  public void setVoltage(double volts) {
    openLoopVolts = volts;
    controlMode = ControlMode.OPEN_LOOP;
  }

  @Override
  public void setHeight(double heightInches) {
    targetHeight = heightInches;
    controlMode = ControlMode.CLOSED_LOOP;
  }

  @Override
  public void setPIDConstraints(double kP, double kD, double kG, Constraints constraints) {
    feedback.setPID(kP, 0.0, kD);
    this.kG = kG;
    feedback.setConstraints(constraints);
  }
}
