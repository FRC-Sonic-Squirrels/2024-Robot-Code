package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.ControlMode;

public class EndEffectorIOReal implements EndEffectorIO {
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
  private TalonFX motor = new TalonFX(Constants.CanIDs.END_EFFECTOR_CAN_ID);

  // Initializes a DigitalInput on DIO 0
  DigitalInput beamBreak = new DigitalInput(Constants.DIOPorts.END_EFFECTOR_BEAM_BREAK);

  private StatusSignal<Double> currentAmps;
  private StatusSignal<Double> velocityRPS;
  private StatusSignal<Double> deviceTemp;

  private double openLoopTargetRPM = 0.0;
  private double closedLoopVoltage = 0.0;
  private double controlEffort = 0.0;

  private PIDController controller = new PIDController(10.0, 0.0, 0.0);

  private ControlMode control = ControlMode.VELOCITY;

  public EndEffectorIOReal() {

    motor.setInverted(false);
    motor.setNeutralMode(NeutralModeValue.Brake);

    currentLimitConfig.SupplyCurrentLimitEnable =
        Constants.EndEffectorConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
    currentLimitConfig.SupplyCurrentThreshold =
        Constants.EndEffectorConstants.SUPPLY_CURRENT_THRESHOLD;
    currentLimitConfig.SupplyCurrentLimit = Constants.EndEffectorConstants.SUPPLY_CURRENT_LIMIT;
    currentLimitConfig.SupplyTimeThreshold = Constants.EndEffectorConstants.SUPPLY_TIME_THRESHOLD;

    config.CurrentLimits = currentLimitConfig;

    motor.getConfigurator().apply(config);

    currentAmps = motor.getStatorCurrent();
    velocityRPS = motor.getVelocity();
    deviceTemp = motor.getDeviceTemp();
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(currentAmps, velocityRPS, deviceTemp);

    if (control.equals(ControlMode.VELOCITY)) {
      controlEffort =
          controller.calculate(velocityRPS.getValueAsDouble() / 60.0, openLoopTargetRPM);
    } else {
      controlEffort = closedLoopVoltage;
    }

    motor.setControl(new VoltageOut(controlEffort));

    inputs.RPM = velocityRPS.getValueAsDouble() / 60.0;
    inputs.tempCelsius = deviceTemp.getValueAsDouble();
    inputs.beamBreak = beamBreak.get();
  }

  @Override
  public void setPercentOut(double percent) {
    motor.set(percent);
    control = ControlMode.VOLTAGE;
  }

  @Override
  public void setRPM(double RPM) {
    openLoopTargetRPM = RPM;
    control = ControlMode.VELOCITY;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoopVoltage = volts;
    control = ControlMode.VOLTAGE;
  }
}
