package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {

  private final TalonFX lead = new TalonFX(Constants.CanIDs.ELEVATOR_LEAD_CAN_ID);

  // FIXME: add FOC
  private final MotionMagicVoltage closedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(false);

  private double heightInches = 0.0;

  public ElevatorIOReal() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.heightInches = heightInches;
  }

  @Override
  public void setVoltage(double volts) {
    lead.setVoltage(volts);
  }

  @Override
  public void setHeight(double heightInches) {
    closedLoopControl.withPosition(heightInches);
    lead.setControl(closedLoopControl);
    this.heightInches = heightInches;
  }

  @Override
  public void setPIDConstraints(double kP, double kD, double kG, Constraints constraints) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    lead.getConfigurator().refresh(pidConfig);
    lead.getConfigurator().refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    mmConfig.MotionMagicCruiseVelocity = constraints.maxVelocity;
    mmConfig.MotionMagicAcceleration = constraints.maxAcceleration;

    lead.getConfigurator().apply(pidConfig);
    lead.getConfigurator().apply(mmConfig);
  }
}
