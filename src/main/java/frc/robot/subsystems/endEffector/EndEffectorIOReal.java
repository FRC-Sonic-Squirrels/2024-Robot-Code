package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class EndEffectorIOReal implements EndEffectorIO {
  private TalonFX motor = new TalonFX(Constants.CanIDs.END_EFFECTOR_CAN_ID);

  // Initializes a DigitalInput on DIO 0
  DigitalInput beamBreak = new DigitalInput(Constants.DIOPorts.END_EFFECTOR_BEAM_BREAK);

  private StatusSignal<Double> velocityRPS;
  private StatusSignal<Double> deviceTemp;
  private StatusSignal<Double> appliedVolts;
  private StatusSignal<Double> currentAmps;

  private double voltage = 0.0;
  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  public EndEffectorIOReal() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

    currentLimitConfig.SupplyCurrentLimitEnable =
        Constants.EndEffectorConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
    currentLimitConfig.SupplyCurrentThreshold =
        Constants.EndEffectorConstants.SUPPLY_CURRENT_THRESHOLD;
    currentLimitConfig.SupplyCurrentLimit = Constants.EndEffectorConstants.SUPPLY_CURRENT_LIMIT;
    currentLimitConfig.SupplyTimeThreshold = Constants.EndEffectorConstants.SUPPLY_TIME_THRESHOLD;

    config.CurrentLimits = currentLimitConfig;
    config.MotorOutput.Inverted = Constants.EndEffectorConstants.INVERTED_VALUE;
    config.MotorOutput.NeutralMode = Constants.EndEffectorConstants.NEUTRAL_MODE_VALUE;
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVolts, velocityRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, deviceTemp);

    velocityRPS = motor.getVelocity();
    deviceTemp = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocityRPS, deviceTemp, appliedVolts, currentAmps);

    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempCelsius = deviceTemp.getValueAsDouble();
    inputs.beamBreak = beamBreak.get();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }
}
