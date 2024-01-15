package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class IntakeReal implements IntakeIO {
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
  private TalonFX motor = new TalonFX(Constants.CanIDs.INTAKE_CAN_ID);

  public IntakeReal() {

    motor.setInverted(false);
    motor.setNeutralMode(NeutralModeValue.Brake);

    currentLimitConfig.SupplyCurrentLimitEnable =
        Constants.IntakeConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
    currentLimitConfig.SupplyCurrentThreshold = Constants.IntakeConstants.SUPPLY_CURRENT_THRESHOLD;
    currentLimitConfig.SupplyCurrentLimit = Constants.IntakeConstants.SUPPLY_CURRENT_LIMIT;
    currentLimitConfig.SupplyTimeThreshold = Constants.IntakeConstants.SUPPLY_TIME_THRESHOLD;

    config.CurrentLimits = currentLimitConfig;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.velocityRPM = motor.getVelocity().getValueAsDouble() / 60.0;
    inputs.deviceTemp = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPercentOut(double percent) {
    motor.set(percent);
  }
}
