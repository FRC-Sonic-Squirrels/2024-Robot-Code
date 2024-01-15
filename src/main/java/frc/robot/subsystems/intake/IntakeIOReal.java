package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
  private TalonFX motor = new TalonFX(Constants.CanIDs.INTAKE_CAN_ID);

  private StatusSignal<Double> currentAmps;
  private StatusSignal<Double> velocityRPS;
  private StatusSignal<Double> deviceTemp;

  public IntakeIOReal() {

    motor.setInverted(false);
    motor.setNeutralMode(NeutralModeValue.Brake);

    currentLimitConfig.SupplyCurrentLimitEnable =
        Constants.IntakeConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
    currentLimitConfig.SupplyCurrentThreshold = Constants.IntakeConstants.SUPPLY_CURRENT_THRESHOLD;
    currentLimitConfig.SupplyCurrentLimit = Constants.IntakeConstants.SUPPLY_CURRENT_LIMIT;
    currentLimitConfig.SupplyTimeThreshold = Constants.IntakeConstants.SUPPLY_TIME_THRESHOLD;

    config.CurrentLimits = currentLimitConfig;

    motor.getConfigurator().apply(config);

    currentAmps = motor.getStatorCurrent();
    velocityRPS = motor.getVelocity();
    deviceTemp = motor.getDeviceTemp();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(currentAmps, velocityRPS, deviceTemp);

    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.velocityRPM = velocityRPS.getValueAsDouble() / 60.0;
    inputs.deviceTemp = deviceTemp.getValueAsDouble();
  }

  @Override
  public void setPercentOut(double percent) {
    motor.set(percent);
  }
}
