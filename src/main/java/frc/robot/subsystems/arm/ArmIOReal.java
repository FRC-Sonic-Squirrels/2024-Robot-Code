package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmIOReal implements ArmIO {
  private static final double RADS_TO_ROTATIONS = 0.0;

  private final StatusSignal<Double> appliedVols;
  private final StatusSignal<Double> positionRotations;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> tempCelsius;

  // FIXME:
  private final TalonFX motor;

  public ArmIOReal() {
    // FIXME
    motor = new TalonFX(0);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentThreshold = 0.0;
    config.CurrentLimits.SupplyTimeThreshold = 0.0;
    config.CurrentLimits.SupplyCurrentLimit = 0.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // closed loop configuration is handled by setClosedLoopConstants()

    appliedVols = motor.getMotorVoltage();
    positionRotations = motor.getPosition();
    currentAmps = motor.getStatorCurrent();
    tempCelsius = motor.getDeviceTemp();

    // BaseStatusSignal.setUpdateFrequencyForAll(100, )
    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVols, positionRotations);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, tempCelsius);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVols, positionRotations, currentAmps, tempCelsius);
    // could look into latency compensating this value
    inputs.armPositionRad = (positionRotations.getValueAsDouble() / RADS_TO_ROTATIONS);

    inputs.armAppliedVolts = appliedVols.getValueAsDouble();
    inputs.armCurrentAmps = currentAmps.getValueAsDouble();
    inputs.armTempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {
    var rotations = angle.getRadians() * RADS_TO_ROTATIONS;
    var control = new MotionMagicVoltage(rotations).withEnableFOC(true);
    // .withFeedForward(/*compensate for gravity here? */)

    motor.setControl(control);
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double maxProfiledVelocity, double maxProfiledAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    // this could be something to explore
    // pidConfig.GravityType
    pidConfig.kP = kP;
    pidConfig.kD = kD;

    MotionMagicConfigs mmConfig = new MotionMagicConfigs();
    mmConfig.MotionMagicCruiseVelocity = maxProfiledAcceleration;
    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    motor.getConfigurator().apply(pidConfig);
    motor.getConfigurator().apply(mmConfig);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts).withEnableFOC(true));
  }
}
