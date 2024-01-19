package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {

  private TalonFX intakeMotor = new TalonFX(0);
  private TalonFXSimState intakeMotorSim = intakeMotor.getSimState();

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorSim = intakeMotor.getSimState();
    intakeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    inputs.currentAmps = intakeMotorSim.getSupplyCurrent();
    inputs.velocityRPM = intakeMotor.get() * Constants.MotorConstants.KrakenConstants.MAX_RPM;
  }

  @Override
  public void setPercentOut(double percent) {
    intakeMotor.set(percent);
  }
}
