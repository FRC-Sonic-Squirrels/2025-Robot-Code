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

  public static final LoggedTunableNumber scorePrepElevatorHeightInches =
      group.build("Reef/ScorePrep/ElevatorHeightInches");

  public static final LoggedTunableNumber scorePrepArmAngleDegrees =
      group.build("Reef/ScorePrep/StowArmAngleDegrees");

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

  private static final LoggedTunableNumber intermediatePoseLowElevatorHeightInches =
      group.build("IntermediatePoses/LowForward/ElevatorHeightInches");

  private static final LoggedTunableNumber intermediatePoseLowArmAngleDegrees =
      group.build("IntermediatePoses/LowForward/ArmAngleDegrees");

  private static final LoggedTunableNumber intermediatePoseHighElevatorHeightInches =
      group.build("IntermediatePoses/High/ElevatorHeightInches");

  private static final LoggedTunableNumber intermediatePoseHighArmAngleDegrees =
      group.build("IntermediatePoses/High/ArmAngleDegrees");

  private static final LoggedTunableNumber intermediatePoseLowBackElevatorHeightInches =
      group.build("IntermediatePoses/LowBack/ElevatorHeightInches");

  private static final LoggedTunableNumber intermediatePoseLowBackArmAngleDegrees =
      group.build("IntermediatePoses/LowBack/ArmAngleDegrees");

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
      reefL2ElevatorHeightInches.initDefault(8.5);
      reefL2ArmAngleDegrees.initDefault(132.71);
      reefL3ElevatorHeightInches.initDefault(26.41);
      reefL3ArmAngleDegrees.initDefault(138.25);
      reefL4ElevatorHeightInches.initDefault(58.4);
      reefL4ArmAngleDegrees.initDefault(148);
      coralStationElevatorHeightInches.initDefault(33);
      coralStationArmAngleDegrees.initDefault(-40);
      algaeClearingLow1ElevatorHeightInches.initDefault(0);
      algaeClearingLow1ArmAngleDegrees.initDefault(149.99);
      algaeClearingLow2ElevatorHeightInches.initDefault(25);
      algaeClearingLow2ArmAngleDegrees.initDefault(150);
      algaeClearingHigh1ElevatorHeightInches.initDefault(27.2);
      algaeClearingHigh1ArmAngleDegrees.initDefault(153);
      algaeClearingHigh2ElevatorHeightInches.initDefault(50);
      algaeClearingHigh2ArmAngleDegrees.initDefault(150);
      scorePrepElevatorHeightInches.initDefault(26);
      scorePrepArmAngleDegrees.initDefault(90);
      intermediatePoseHighElevatorHeightInches.initDefault(26);
      intermediatePoseHighArmAngleDegrees.initDefault(70);
      intermediatePoseLowElevatorHeightInches.initDefault(0);
      intermediatePoseLowArmAngleDegrees.initDefault(70);
      intermediatePoseLowBackElevatorHeightInches.initDefault(0);
      intermediatePoseLowBackArmAngleDegrees.initDefault(140);
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

  public static MechanismPosition scorePrepPosition() {
    return new MechanismPosition(
        Units.Inches.of(scorePrepElevatorHeightInches.get()),
        Rotation2d.fromDegrees(scorePrepArmAngleDegrees.get()));
  }

  public static MechanismPosition intermediateLowPosition() {
    return new MechanismPosition(
        Units.Inches.of(intermediatePoseLowElevatorHeightInches.get()),
        Rotation2d.fromDegrees(intermediatePoseLowArmAngleDegrees.get()));
  }

  public static MechanismPosition intermediateLowBackPosition() {
    return new MechanismPosition(
        Units.Inches.of(intermediatePoseLowBackElevatorHeightInches.get()),
        Rotation2d.fromDegrees(intermediatePoseLowBackArmAngleDegrees.get()));
  }

  public static MechanismPosition intermediateHighPosition() {
    return new MechanismPosition(
        Units.Inches.of(intermediatePoseHighElevatorHeightInches.get()),
        Rotation2d.fromDegrees(intermediatePoseHighArmAngleDegrees.get()));
  }
}
