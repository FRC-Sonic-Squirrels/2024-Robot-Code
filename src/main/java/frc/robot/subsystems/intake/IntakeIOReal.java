package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  private TalonFX motor = new TalonFX(Constants.CanIDs.INTAKE_CAN_ID);

  // Initializes a DigitalInput on DIO 0
  DigitalInput beamBreak = new DigitalInput(Constants.DIOPorts.INTAKE_BEAM_BREAK);

  private StatusSignal<Double> currentAmps;
  private StatusSignal<Double> velocityRPS;
  private StatusSignal<Double> deviceTemp;

  private double voltage = 0.0;

  public IntakeIOReal() {

    motor.setInverted(false);
    motor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

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
    BaseStatusSignal.refreshAll(currentAmps, deviceTemp);

    motor.setControl(new VoltageOut(voltage));

    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempCelsius = deviceTemp.getValueAsDouble();
    inputs.beamBreak = beamBreak.get();
  }

  @Override
  public void setVoltage(double volts) {
    voltage = volts;
  }
}
