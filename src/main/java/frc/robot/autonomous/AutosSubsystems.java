package frc.robot.autonomous;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public record AutosSubsystems(
    DrivetrainWrapper drivetrain,
    Mechanism mech,
    EndEffector endEffector,
    LED led,
    Intake intake) {}
