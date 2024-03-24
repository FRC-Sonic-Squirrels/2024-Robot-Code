package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
  private final double MOMENT_OF_INERTIA = 0.15;
  private final Measure<Distance> ARM_LENGTH = Units.Inches.of(14);

  SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          Constants.ArmConstants.GEAR_RATIO,
          MOMENT_OF_INERTIA,
          ARM_LENGTH.in(Units.Meters),
          Constants.ArmConstants.MIN_ARM_ANGLE.getRadians(),
          Constants.ArmConstants.MAX_ARM_ANGLE.getRadians(),
          false,
          0);

  private final ProfiledPIDController feedback;

  private double kG;

  private boolean closedLoop = false;
  private Rotation2d closedLoopTargetAngle = Constants.zeroRotation2d;
  private double openLoopVolts = 0.0;

  public ArmIOSim() {
    feedback = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  }

  @Override
  public void updateInputs(Inputs inputs) {
    var controlEffort = 0.0;

    if (closedLoop) {
      var fb =
          feedback.calculate(inputs.armPosition.getRadians(), closedLoopTargetAngle.getRadians());
      var ff = kG * Math.cos(armSim.getAngleRads());

      controlEffort = fb + ff;

      Arm.logSIM_FF_fG.info(ff);
      Arm.logSIM_error.info(feedback.getPositionError());

    } else {
      controlEffort = openLoopVolts;
    }

    controlEffort = MathUtil.clamp(controlEffort, -12, 12);

    armSim.setInputVoltage(controlEffort);
    armSim.update(Constants.kDefaultPeriod);

    inputs.armPosition = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.armAngleDegrees = inputs.armPosition.getDegrees();
    inputs.armAppliedVolts = controlEffort;
    inputs.armCurrentAmps = armSim.getCurrentDrawAmps();

    Arm.logSIM_error.info(feedback.getPositionError());
    Arm.logSIM_controlEffort.info(controlEffort);
  }

  @Override
  public void setVoltage(double volts) {
    openLoopVolts = volts;
    closedLoop = false;
  }

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {
    closedLoop = true;
    closedLoopTargetAngle = angle;
    openLoopVolts = 0.0;
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {

    this.kG = kG;
    feedback.setP(kP);
    feedback.setD(kD);
    feedback.setConstraints(new Constraints(maxProfiledVelocity, maxProfiledAcceleration));
  }

  @Override
  public void resetSensorPosition(Rotation2d angle) {
    armSim.setState(angle.getRadians(), 0.0);
  }
}
