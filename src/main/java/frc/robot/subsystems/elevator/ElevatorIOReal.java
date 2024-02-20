package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

  private final TalonFX motor = new TalonFX(Constants.CanIDs.ELEVATOR_CAN_ID);

  private static final double inchesToMotorRot =
      Constants.ElevatorConstants.GEAR_RATIO
          / (Math.PI * Constants.ElevatorConstants.PULLEY_DIAMETER);
  // FIXME: add FOC
  private final MotionMagicVoltage closedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(false);
  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(false);

  private StatusSignal<Double> rotorPosition;
  private StatusSignal<Double> rotorVelocity;
  private StatusSignal<Double> appliedVolts;
  private StatusSignal<Double> currentAmps;
  private StatusSignal<Double> tempCelsius;

  public ElevatorIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // FIXME: maybe make this more aggressive?
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // config.Voltage.PeakForwardVoltage = 2.0;
    // config.Voltage.PeakReverseVoltage = -2.0;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.MAX_HEIGHT.in(Units.Inches) * inchesToMotorRot;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0 * inchesToMotorRot;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    motor.getConfigurator().apply(config);

    rotorPosition = motor.getRotorPosition();
    rotorVelocity = motor.getRotorVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    tempCelsius = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100, rotorPosition, rotorVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVolts, currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, tempCelsius);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rotorPosition, rotorVelocity, appliedVolts, currentAmps, tempCelsius);

    inputs.heightInches = rotorPosition.getValueAsDouble() / inchesToMotorRot;
    inputs.velocityInchesPerSecond = rotorVelocity.getValueAsDouble() / inchesToMotorRot;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    openLoopControl.withOutput(volts);
    motor.setControl(openLoopControl);
  }

  @Override
  public void setHeight(Measure<Distance> height) {
    closedLoopControl.withPosition(height.in(Units.Inches) * inchesToMotorRot);
    motor.setControl(closedLoopControl);
  }

  @Override
  public void setSensorPosition(Measure<Distance> position) {
    motor.setPosition(position.in(Units.Inches) * inchesToMotorRot);
  }

  @Override
  public void setPIDConstraints(double kP, double kD, double kG, Constraints constraints) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    motor.getConfigurator().refresh(pidConfig);
    motor.getConfigurator().refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    mmConfig.MotionMagicCruiseVelocity = constraints.maxVelocity;
    mmConfig.MotionMagicAcceleration = constraints.maxAcceleration;

    motor.getConfigurator().apply(pidConfig);
    motor.getConfigurator().apply(mmConfig);
  }

  @Override
  public void setNeutralMode(NeutralModeValue value) {
    motor.setNeutralMode(value);
  }
}
