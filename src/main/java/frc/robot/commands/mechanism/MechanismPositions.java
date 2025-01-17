package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotStates.ScoringLevel;

public class MechanismPositions {
    private static final TunableNumberGroup group = new TunableNumberGroup("MechanismPositions");

    public static final LoggedTunableNumber stowElevatorHeightInches =
        group.build("Stow/ElevatorHeightInches");

    public static final LoggedTunableNumber stowArmAngleDegrees =
        group.build("Stow/StowArmAngleDegrees");

    private static final LoggedTunableNumber reefL1ArmAngleDegrees =
        group.build("Reef/Score/L1/ArmAngleDegrees");

    private static final LoggedTunableNumber reefL2ArmAngleDegrees =
        group.build("Reef/Score/L2/ArmAngleDegrees");

    private static final LoggedTunableNumber reefL3ArmAngleDegrees =
        group.build("Reef/Score/L3/ArmAngleDegrees");

    private static final LoggedTunableNumber reefL4ArmAngleDegrees =
        group.build("Reef/Score/L4/ArmAngleDegrees");

    private static final LoggedTunableNumber reefL1ElevatorHeightInches =
        group.build(
            "reef/L1/ElevatorHeightInches"); 

    private static final LoggedTunableNumber reefL2ElevatorHeightInches =
        group.build(
            "reef/L2/ElevatorHeightInches");

    private static final LoggedTunableNumber reefL3ElevatorHeightInches =
        group.build(
            "reef/L3/ElevatorHeightInches");

    private static final LoggedTunableNumber reefL4ElevatorHeightInches =
        group.build(
            "reef/4/ElevatorHeightInches");

    private static final LoggedTunableNumber coralStationElevatorHeightInches =
        group.build("coralStation/ElevatorHeightInches");

    private static final LoggedTunableNumber coralStationArmAngleDegrees =
        group.build("coralStation/ArmAngleDegrees");

    static{
        if(Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO){
            stowElevatorHeightInches.initDefault(12);
            stowArmAngleDegrees.initDefault(0);
            reefL1ElevatorHeightInches.initDefault(20);
            reefL1ArmAngleDegrees.initDefault(43);
            reefL2ElevatorHeightInches.initDefault(16);
            reefL2ArmAngleDegrees.initDefault(0);
            reefL3ElevatorHeightInches.initDefault(20);
            reefL3ArmAngleDegrees.initDefault(0);
            reefL4ElevatorHeightInches.initDefault(26.2);
            reefL4ArmAngleDegrees.initDefault(120);
            coralStationElevatorHeightInches.initDefault(0);
            coralStationArmAngleDegrees.initDefault(0);
        } else {
            stowElevatorHeightInches.initDefault(11.6);
            stowArmAngleDegrees.initDefault(11.6);
            reefL1ElevatorHeightInches.initDefault(20); // just above where game manual has it set
            reefL1ArmAngleDegrees.initDefault(43);
            reefL2ElevatorHeightInches.initDefault(32); // just above where game manual has it set
            reefL2ArmAngleDegrees.initDefault(35);
            reefL3ElevatorHeightInches.initDefault(48); // just above where game manual has it set
            reefL3ArmAngleDegrees.initDefault(35);
            reefL4ElevatorHeightInches.initDefault(72); // 72 inches IS 6 feet, probably change this value
            reefL4ArmAngleDegrees.initDefault(90);
            coralStationElevatorHeightInches.initDefault(38);
            coralStationArmAngleDegrees.initDefault(55);
        }
        
    }

    public record MechanismPosition(Distance elevatorHeight, Rotation2d armAngle) {}

    public static MechanismPosition stowPosition() {
        return new MechanismPosition(
            Units.Inches.of(stowElevatorHeightInches.get()),
            Rotation2d.fromDegrees(reefL1ArmAngleDegrees.get()));
    }

    public static MechanismPosition reefPrepPosition(ScoringLevel scoringLevel) {
        switch (scoringLevel) {
            case L1:
                return new MechanismPosition(
                    Units.Inches.of(reefL1ElevatorHeightInches.get()),
                    Rotation2d.fromDegrees(reefL1ArmAngleDegrees.get()));
            case L2:
                return new MechanismPosition(
                    Units.Inches.of(reefL2ElevatorHeightInches.get()),
                    Rotation2d.fromDegrees(reefL2ArmAngleDegrees.get()));
            case L3:
                return new MechanismPosition(
                    Units.Inches.of(reefL3ElevatorHeightInches.get()),
                    Rotation2d.fromDegrees(reefL3ArmAngleDegrees.get()));
            case L4:
                return new MechanismPosition(
                    Units.Inches.of(reefL4ElevatorHeightInches.get()),
                    Rotation2d.fromDegrees(reefL4ArmAngleDegrees.get()));
            default:
                return new MechanismPosition(
                    Units.Inches.of(reefL1ElevatorHeightInches.get()),
                    Rotation2d.fromDegrees(reefL1ArmAngleDegrees.get()));
        }
    }

    public static MechanismPosition reefScorePosition(ScoringLevel scoringLevel) {
        switch (scoringLevel) {
            case L1:
                return new MechanismPosition(
                    Units.Inches.of(reefL1ElevatorHeightInches.get()),
                    Rotation2d.fromDegrees(reefL1ArmAngleDegrees.get()));
            case L2:
                return new MechanismPosition(
                    Units.Inches.of(reefL2ElevatorHeightInches.get()),
                    Rotation2d.fromDegrees(reefL2ArmAngleDegrees.get()));
            case L3:
                return new MechanismPosition(
                    Units.Inches.of(reefL3ElevatorHeightInches.get()),
                    Rotation2d.fromDegrees(reefL3ArmAngleDegrees.get()));
            case L4:
                return new MechanismPosition(
                    Units.Inches.of(reefL4ElevatorHeightInches.get()),
                    Rotation2d.fromDegrees(reefL4ArmAngleDegrees.get()));
            default:
                return new MechanismPosition(
                    Units.Inches.of(reefL1ElevatorHeightInches.get()),
                    Rotation2d.fromDegrees(reefL1ArmAngleDegrees.get()));
        }
    }

    public static MechanismPosition coralStationPosition() {
        return new MechanismPosition(
            Units.Inches.of(coralStationElevatorHeightInches.get()),
            Rotation2d.fromDegrees(coralStationArmAngleDegrees.get()));
    }
}
