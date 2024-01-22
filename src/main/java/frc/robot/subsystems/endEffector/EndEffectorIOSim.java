package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.RobotController;

public class EndEffectorIOSim implements EndEffectorIO {

  private TalonFX motor = new TalonFX(0);
  private TalonFXSimState motorSim = motor.getSimState();

  public EndEffectorIOSim() {}

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    motorSim = motor.getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
  }
}
