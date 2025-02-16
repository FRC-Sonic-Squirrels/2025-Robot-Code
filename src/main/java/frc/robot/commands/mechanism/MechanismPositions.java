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
      group.build("Reef/L1/ArmAngleDegrees");

  private static final LoggedTunableNumber reefL2ArmAngleDegrees =
      group.build("Reef/L2/Scoring/ArmAngleDegrees");

  private static final LoggedTunableNumber reefL3ArmAngleDegrees =
      group.build("Reef/L3/Scoring/ArmAngleDegrees");

  private static final LoggedTunableNumber reefL4ArmAngleDegrees =
      group.build("Reef/L4/ArmAngleDegrees");

  private static final LoggedTunableNumber reefL1ElevatorHeightInches =
      group.build("Reef/L1/ElevatorHeightInches");

  private static final LoggedTunableNumber reefL2ElevatorHeightInches =
      group.build("Reef/L2/Scoring/ElevatorHeightInches");

  private static final LoggedTunableNumber reefL3ElevatorHeightInches =
      group.build("Reef/L3/Scoring/ElevatorHeightInches");

  private static final LoggedTunableNumber reefL4ElevatorHeightInches =
      group.build("Reef/L4/ElevatorHeightInches");

  private static final LoggedTunableNumber coralStationElevatorHeightInches =
      group.build("CoralStation/ElevatorHeightInches");

  private static final LoggedTunableNumber coralStationArmAngleDegrees =
      group.build("CoralStation/ArmAngleDegrees");

  private static final LoggedTunableNumber algaeClearingLow1ElevatorHeightInches =
      group.build("Reef/AlgaeClearing/Low/Step1/ElevatorHeightInches");

  private static final LoggedTunableNumber algaeClearingLow1ArmAngleDegrees =
      group.build("Reef/AlgaeClearing/Low/Step1/ArmAngleDegrees");

  private static final LoggedTunableNumber algaeClearingLow2ElevatorHeightInches =
      group.build("Reef/AlgaeClearing/Low/Step2/ElevatorHeightInches");

  private static final LoggedTunableNumber algaeClearingLow2ArmAngleDegrees =
      group.build("Reef/AlgaeClearing/Low/Step2/ArmAngleDegrees");

  private static final LoggedTunableNumber algaeClearingHigh1ElevatorHeightInches =
      group.build("Reef/AlgaeClearing/High/Step1/ElevatorHeightInches");

  private static final LoggedTunableNumber algaeClearingHigh1ArmAngleDegrees =
      group.build("Reef/AlgaeClearing/High/Step1/ArmAngleDegrees");

  private static final LoggedTunableNumber algaeClearingHigh2ElevatorHeightInches =
      group.build("Reef/AlgaeClearing/High/Step2/ElevatorHeightInches");

  private static final LoggedTunableNumber algaeClearingHigh2ArmAngleDegrees =
      group.build("Reef/AlgaeClearing/High/Step2/ArmAngleDegrees");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
      stowElevatorHeightInches.initDefault(2);
      stowArmAngleDegrees.initDefault(80);
      reefL1ElevatorHeightInches.initDefault(16);
      reefL1ArmAngleDegrees.initDefault(43);
      reefL2ElevatorHeightInches.initDefault(16);
      reefL2ArmAngleDegrees.initDefault(0);
      reefL3ElevatorHeightInches.initDefault(20);
      reefL3ArmAngleDegrees.initDefault(0);
      reefL4ElevatorHeightInches.initDefault(26.2);
      reefL4ArmAngleDegrees.initDefault(120);
      coralStationElevatorHeightInches.initDefault(25.451);
      coralStationArmAngleDegrees.initDefault(-22.324);
      algaeClearingLow1ElevatorHeightInches.initDefault(17.34);
      algaeClearingLow1ArmAngleDegrees.initDefault(0);
      algaeClearingLow2ElevatorHeightInches.initDefault(21.95);
      algaeClearingLow2ArmAngleDegrees.initDefault(32);
      algaeClearingHigh1ElevatorHeightInches.initDefault(24.03);
      algaeClearingHigh1ArmAngleDegrees.initDefault(0);
      algaeClearingHigh2ElevatorHeightInches.initDefault(26.2);
      algaeClearingHigh2ArmAngleDegrees.initDefault(32);
    } else {
      stowElevatorHeightInches.initDefault(11.6);
      stowArmAngleDegrees.initDefault(11.6);
      reefL1ElevatorHeightInches.initDefault(1);
      reefL1ArmAngleDegrees.initDefault(140);
      reefL2ElevatorHeightInches.initDefault(15);
      reefL2ArmAngleDegrees.initDefault(150);
      reefL3ElevatorHeightInches.initDefault(30);
      reefL3ArmAngleDegrees.initDefault(150);
      reefL4ElevatorHeightInches.initDefault(55);
      reefL4ArmAngleDegrees.initDefault(150);
      coralStationElevatorHeightInches.initDefault(31);
      coralStationArmAngleDegrees.initDefault(-40);
      algaeClearingLow1ElevatorHeightInches.initDefault(10);
      algaeClearingLow1ArmAngleDegrees.initDefault(150);
      algaeClearingLow2ElevatorHeightInches.initDefault(25);
      algaeClearingLow2ArmAngleDegrees.initDefault(150);
      algaeClearingHigh1ElevatorHeightInches.initDefault(35);
      algaeClearingHigh1ArmAngleDegrees.initDefault(150);
      algaeClearingHigh2ElevatorHeightInches.initDefault(50);
      algaeClearingHigh2ArmAngleDegrees.initDefault(150);
    }
  }

  public record MechanismPosition(Distance elevatorHeight, Rotation2d armAngle) {}

  public static MechanismPosition stowPosition() {
    return new MechanismPosition(
        Units.Inches.of(stowElevatorHeightInches.get()),
        Rotation2d.fromDegrees(reefL1ArmAngleDegrees.get()));
  }

  public static MechanismPosition reefPosition(ScoringLevel scoringLevel) {
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

  public static MechanismPosition clearAlgaeLow1Position() {
    return new MechanismPosition(
        Units.Inches.of(algaeClearingLow1ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(algaeClearingLow1ArmAngleDegrees.get()));
  }

  public static MechanismPosition clearAlgaeLow2Position() {
    return new MechanismPosition(
        Units.Inches.of(algaeClearingLow2ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(algaeClearingLow2ArmAngleDegrees.get()));
  }

  public static MechanismPosition clearAlgaeHigh1Position() {
    return new MechanismPosition(
        Units.Inches.of(algaeClearingHigh1ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(algaeClearingHigh1ArmAngleDegrees.get()));
  }

  public static MechanismPosition clearAlgaeHigh2Position() {
    return new MechanismPosition(
        Units.Inches.of(algaeClearingHigh2ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(algaeClearingHigh2ArmAngleDegrees.get()));
  }
}
