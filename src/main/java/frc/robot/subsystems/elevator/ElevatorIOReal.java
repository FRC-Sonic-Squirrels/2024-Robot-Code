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
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {

  private final TalonFX motor = new TalonFX(Constants.CanIDs.ELEVATOR_CAN_ID, "CANivore");

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

    config.Voltage.PeakForwardVoltage = 2.0;
    config.Voltage.PeakReverseVoltage = -2.0;

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

    // closedLoopControl.EnableFOC = fal

    closedLoopControl.EnableFOC = false;

    Logger.recordOutput("Elevator/periodicMotorRot", closedLoopControl.Position);
  }

  @Override
  public void setVoltage(double volts) {
    openLoopControl.withOutput(volts);
    motor.setControl(openLoopControl);
  }

  @Override
  public void setHeight(double heightInches) {
    closedLoopControl.withPosition(heightInches * inchesToMotorRot);
    motor.setControl(closedLoopControl);

    Logger.recordOutput("Elevator/motorRotSetpoint", heightInches * inchesToMotorRot);

    System.out.println("in here");
  }

  @Override
  public void setSensorPositionInches(double positionInches) {
    motor.setPosition(positionInches * inchesToMotorRot);
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
}
