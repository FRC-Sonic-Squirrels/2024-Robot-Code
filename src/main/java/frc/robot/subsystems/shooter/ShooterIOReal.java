package frc.robot.subsystems.shooter;

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
  private double RPM = 0.0;
  public double launcherLeadTempCelsius = 0.0;
  public double launcherFollowTempCelsius = 0.0;
  public double launcherVoltage = 0.0;

  public double pivotVoltage = 0.0;
  public double pivotTempCelsius = 0.0;

  private PIDController pivotController = new PIDController(0.01, 0, 0);

  Slot0Configs slot0Configs = new Slot0Configs();
  final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

  public ShooterIOReal() {
    slot0Configs.kP = 0;
    slot0Configs.kD = 0;
    slot0Configs.kG = 0;
    lead.getConfigurator().apply(slot0Configs);
    follow.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // FIXME: get actual arm code
    inputs.pitch = new Rotation2d(/*arm.getAngle()*/ );
  }

  // PIVOT

  @Override
  public void setPivotVel(double radPerSec) {}

  @Override
  public void setPivotPosition(Rotation2d rot) {}

  @Override
  public void setPivotClosedLoopConstants(
      double kP,
      double kD,
      double kG,
      double maxProfiledVelocity,
      double maxProfiledAcceleration) {}

  @Override
  public void setPivotVoltage(double volts) {}

  // LAUNCHER

  @Override
  public void setLauncherVoltage(double volts) {}

  @Override
  public void setLauncherPercentOut(double percent) {}

  @Override
  public void setLauncherRPM(double rpm) {
    RPM = rpm;
    lead.setControl(request.withVelocity(rpm));
    follow.setControl(request.withVelocity(rpm));
  }

  @Override
  public void setLauncherClosedLoopConstants(double kP, double kI, double kD) {}
}
