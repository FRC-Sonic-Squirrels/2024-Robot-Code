package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOSim implements IntakeIO {

  private TalonFX motor = new TalonFX(0);
  private TalonFXSimState motorSim = motor.getSimState();

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motorSim = motor.getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    inputs.currentAmps = motorSim.getSupplyCurrent();
  }
}
