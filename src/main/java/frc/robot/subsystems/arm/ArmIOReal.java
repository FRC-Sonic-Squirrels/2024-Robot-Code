package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ArmIOReal implements ArmIO {
  private final StatusSignal<Double> appliedVols;
  private final StatusSignal<Double> positionRotations;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> tempCelsius;

  // FIXME: add FOC
  private final MotionMagicVoltage closedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(false);
  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private final TalonFX motor;

  public ArmIOReal() {
    motor = new TalonFX(Constants.CanIDs.ARM_CAN_ID, "CANivore");

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    //FIXME: find true current limits
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = Constants.ArmConstants.GEAR_RATIO;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // other closed loop configuration is handled by setClosedLoopConstants()

    motor.getConfigurator().apply(config);

    appliedVols = motor.getMotorVoltage();
    positionRotations = motor.getPosition();
    currentAmps = motor.getStatorCurrent();
    tempCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVols, positionRotations);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, tempCelsius);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(appliedVols, positionRotations, currentAmps, tempCelsius);
    // could look into latency compensating this value
    inputs.armPosition = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.armAppliedVolts = appliedVols.getValueAsDouble();
    inputs.armCurrentAmps = currentAmps.getValueAsDouble();
    inputs.armTempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {
    closedLoopControl.withPosition(angle.getRotations());
    motor.setControl(closedLoopControl);
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {

    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    motor.getConfigurator().refresh(pidConfig);
    motor.getConfigurator().refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    mmConfig.MotionMagicCruiseVelocity = maxProfiledVelocity;
    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    motor.getConfigurator().apply(pidConfig);
    motor.getConfigurator().apply(mmConfig);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void resetSensorPosition(Rotation2d angle) {
    motor.setPosition(angle.getRotations());
  }
}
