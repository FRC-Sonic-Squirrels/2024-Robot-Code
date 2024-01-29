package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {

  private final TalonFX lead = new TalonFX(Constants.CanIDs.ELEVATOR_LEAD_CAN_ID);
  private final TalonFX follow = new TalonFX(Constants.CanIDs.ELEVATOR_FOLLOW_CAN_ID);

  // FIXME: add FOC
  private final MotionMagicVoltage closedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(false);

  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private StatusSignal<Double> leadTemp;
  private StatusSignal<Double> followTemp;
  private StatusSignal<Double> position;

  public ElevatorIOReal() {
    leadTemp = lead.getDeviceTemp();
    followTemp = follow.getDeviceTemp();
    position = lead.getPosition();
    follow.setControl(
        new Follower(
            Constants.CanIDs.ELEVATOR_LEAD_CAN_ID, Constants.ElevatorConstants.FOLLOW_INVERTED));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(leadTemp, followTemp, position);
    inputs.heightInches = motorPositionToHeightInches(position.getValueAsDouble());
    inputs.leadTempCelsius = leadTemp.getValueAsDouble();
    inputs.followTempCelsius = followTemp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    lead.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setHeight(double heightInches) {
    closedLoopControl.withPosition(heightInches);
    lead.setControl(closedLoopControl);
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

  private double motorPositionToHeightInches(double position) {
    return position
        * Constants.ElevatorConstants.GEAR_RATIO
        / Constants.ElevatorConstants.WHEEL_RADIUS
        * 2.0
        * Math.PI;
  }
}
