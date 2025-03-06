package frc.robot.autonomous;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public record AutosSubsystems(
    DrivetrainWrapper drivetrain, Elevator elevator, Arm arm, EndEffector endEffector, LED led) {}
