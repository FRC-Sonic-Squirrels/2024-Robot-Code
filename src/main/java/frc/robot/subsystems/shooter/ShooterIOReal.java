package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
  TalonFX lead = new TalonFX(Constants.CanIDs.SHOOTER_LEAD_CAN_ID);
  TalonFX follow = new TalonFX(Constants.CanIDs.SHOOTER_FOLLOW_CAN_ID);
  TalonFX pivot = new TalonFX(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID);

  public Rotation2d pitch = new Rotation2d();
  public double pivotVoltage = 0.0;
  public double pivotTempCelsius = 0.0;
  public double pivotVelocity = 0.0;

  public double RPM = 0.0;
  public StatusSignal<Double> launcherLeadTempCelsius;
  public StatusSignal<Double> launcherFollowTempCelsius;
  public double launcherVoltage = 0.0;

  private PIDController pivotController = new PIDController(0.01, 0, 0);

  Slot0Configs pidConfig = new Slot0Configs();
  final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

  public ShooterIOReal() {
    pidConfig.kP = 0;
    pidConfig.kD = 0;
    pidConfig.kG = 0;
    pidConfig.kI = 0;
    lead.getConfigurator().apply(pidConfig);
    follow.getConfigurator().apply(pidConfig);
    pivot.getConfigurator().apply(pidConfig);

    launcherLeadTempCelsius = lead.getDeviceTemp();
    launcherFollowTempCelsius = follow.getDeviceTemp();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // FIXME: add other variables
    BaseStatusSignal.refreshAll(launcherLeadTempCelsius, launcherFollowTempCelsius);
    // FIXME: get actual arm code
    inputs.pitch = new Rotation2d(/*arm.getAngle()*/ );
    inputs.launcherLeadTempCelsius = launcherLeadTempCelsius.getValueAsDouble();
    inputs.launcherFollowTempCelsius = launcherFollowTempCelsius.getValueAsDouble();
  }

  // PIVOT

  @Override
  public void setPivotVel(double radPerSec) {
    pivot.set(radPerSec);
  }

  @Override
  public void setPivotPosition(Rotation2d rot) {
    pitch = rot;
  }

  @Override
  public void setPivotClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {
    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();
    mmConfig.MotionMagicCruiseVelocity = maxProfiledVelocity;
    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotVoltage = volts;
    pivot.setVoltage(volts);
  }

  // LAUNCHER

  @Override
  public void setLauncherVoltage(double volts) {
    launcherVoltage = volts;
    lead.setVoltage(volts);
    follow.setVoltage(volts);
  }

  @Override
  public void setLauncherPercentOut(double percent) {
    lead.set(percent);
    follow.set(percent);
  }

  @Override
  public void setLauncherRPM(double rpm) {
    RPM = rpm;
    lead.setControl(request.withVelocity(rpm));
    follow.setControl(request.withVelocity(rpm));
  }

  @Override
  public void setLauncherClosedLoopConstants(double kP, double kI, double kD) {
    pivotController.setPID(kP, kI, kD);
  }
}
