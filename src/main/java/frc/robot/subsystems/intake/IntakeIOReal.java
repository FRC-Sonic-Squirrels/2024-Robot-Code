package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private TalonFX motor = new TalonFX(Constants.CanIDs.INTAKE_CAN_ID);

  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> deviceTemp;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> velocityRPS;

  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  private final MotionMagicVelocityVoltage closedLoopControl =
      new MotionMagicVelocityVoltage(0, 0, true, 0, 0, false, false, false);

  private final BaseStatusSignal[] refreshSet;

  public IntakeIOReal() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

    currentLimitConfig.SupplyCurrentLimit = 40.0;
    currentLimitConfig.SupplyCurrentLimitEnable = true;

    config.CurrentLimits = currentLimitConfig;

    config.Feedback.SensorToMechanismRatio = IntakeConstants.GEARING;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.SupplyVoltageTimeConstant = 0.02;

    motor.getConfigurator().apply(config);

    currentAmps = motor.getStatorCurrent();
    deviceTemp = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    velocityRPS = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVolts, currentAmps, velocityRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(1, deviceTemp);

    motor.optimizeBusUtilization();
    refreshSet = new BaseStatusSignal[] {currentAmps, deviceTemp, appliedVolts, velocityRPS};
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(refreshSet);

    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempCelsius = deviceTemp.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.velocityRPM = velocityRPS.getValueAsDouble() * 60.0;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setVelocity(double revPerMin) {
    motor.setControl(closedLoopControl.withVelocity(revPerMin / 60.0));
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var config = motor.getConfigurator();

    config.refresh(pidConfig);
    config.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;
    pidConfig.kS = kS;

    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    config.apply(pidConfig);
    config.apply(mmConfig);
  }
}
