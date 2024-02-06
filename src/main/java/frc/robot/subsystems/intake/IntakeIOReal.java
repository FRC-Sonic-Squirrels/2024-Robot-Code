package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private TalonFX motor = new TalonFX(Constants.CanIDs.INTAKE_CAN_ID);

  private StatusSignal<Double> currentAmps;
  private StatusSignal<Double> deviceTemp;
  private StatusSignal<Double> appliedVolts;

  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  public IntakeIOReal() {

    motor.setInverted(false);
    motor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

    currentLimitConfig.SupplyCurrentThreshold = 40.0;
    currentLimitConfig.SupplyCurrentLimitEnable = true;

    config.CurrentLimits = currentLimitConfig;

    motor.getConfigurator().apply(config);

    currentAmps = motor.getStatorCurrent();
    deviceTemp = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(currentAmps, deviceTemp, appliedVolts);

    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempCelsius = deviceTemp.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }
}
