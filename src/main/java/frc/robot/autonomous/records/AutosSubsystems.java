package frc.robot.autonomous.records;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;

public record AutosSubsystems(
    DrivetrainWrapper drivetrain,
    Elevator elevator,
    Arm arm,
    Intake intake,
    EndEffector endEffector,
    Shooter shooter,
    VisionGamepiece visionGamepiece,
    LED led) {}
