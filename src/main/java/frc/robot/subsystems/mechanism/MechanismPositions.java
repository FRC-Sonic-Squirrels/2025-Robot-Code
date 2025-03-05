package frc.robot.subsystems.mechanism;

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
      group.build("Reef/ScorePrep/ElevatorHeightInches");

  public static final LoggedTunableNumber stowArmAngleDegrees =
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

  private static final LoggedTunableNumber reefPrepArmOffsetDegrees =
      group.build("Reef/ReefPrepArmAngleDegrees");

  private static final LoggedTunableNumber coralStationElevatorHeightInches =
      group.build("CoralStation/ElevatorHeightInches");

  private static final LoggedTunableNumber coralStationArmAngleDegrees =
      group.build("CoralStation/ArmAngleDegrees");

  private static final LoggedTunableNumber algaeClearingLowElevatorHeightInches =
      group.build("Reef/AlgaeClearing/Low/ElevatorHeightInches");

  private static final LoggedTunableNumber algaeClearingLowArmAngleDegrees =
      group.build("Reef/AlgaeClearing/Low/ArmAngleDegrees");

  private static final LoggedTunableNumber algaeClearingHighElevatorHeightInches =
      group.build("Reef/AlgaeClearing/High/ElevatorHeightInches");

  private static final LoggedTunableNumber algaeClearingHighArmAngleDegrees =
      group.build("Reef/AlgaeClearing/High/ArmAngleDegrees");

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

  private static final LoggedTunableNumber passOffElevatorHeightInches =
      group.build("PassOff/ElevatorHeightInches");

  private static final LoggedTunableNumber passOffArmAngleDegrees =
      group.build("PassOff/ArmAngleDegrees");

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
      algaeClearingLowElevatorHeightInches.initDefault(17.34);
      algaeClearingLowArmAngleDegrees.initDefault(0);
      algaeClearingHighElevatorHeightInches.initDefault(24.03);
      algaeClearingHighArmAngleDegrees.initDefault(0);
    } else {
      reefL1ElevatorHeightInches.initDefault(1);
      reefL1ArmAngleDegrees.initDefault(140);
      reefL2ElevatorHeightInches.initDefault(8.5);
      reefL2ArmAngleDegrees.initDefault(132.71);
      reefL3ElevatorHeightInches.initDefault(26.94);
      reefL3ArmAngleDegrees.initDefault(138.69);
      reefL4ElevatorHeightInches.initDefault(54.8);
      reefL4ArmAngleDegrees.initDefault(151);
      coralStationElevatorHeightInches.initDefault(33);
      coralStationArmAngleDegrees.initDefault(-40);
      algaeClearingLowElevatorHeightInches.initDefault(8);
      algaeClearingLowArmAngleDegrees.initDefault(166);
      algaeClearingHighElevatorHeightInches.initDefault(27.2);
      algaeClearingHighArmAngleDegrees.initDefault(153);
      stowElevatorHeightInches.initDefault(0);
      stowArmAngleDegrees.initDefault(90);
      intermediatePoseHighElevatorHeightInches.initDefault(26);
      intermediatePoseHighArmAngleDegrees.initDefault(70);
      intermediatePoseLowElevatorHeightInches.initDefault(0);
      intermediatePoseLowArmAngleDegrees.initDefault(70);
      intermediatePoseLowBackElevatorHeightInches.initDefault(0);
      intermediatePoseLowBackArmAngleDegrees.initDefault(130);
      reefPrepArmOffsetDegrees.initDefault(20);
      passOffArmAngleDegrees.initDefault(9.756);
      passOffElevatorHeightInches.initDefault(11.922);
    }
  }

  public record MechanismPosition(Distance elevatorHeight, Rotation2d armAngle) {}

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
            Rotation2d.fromDegrees(reefL3ArmAngleDegrees.get() - reefPrepArmOffsetDegrees.get()));
      case L4:
        return new MechanismPosition(
            Units.Inches.of(reefL4ElevatorHeightInches.get()),
            Rotation2d.fromDegrees(reefL4ArmAngleDegrees.get() - reefPrepArmOffsetDegrees.get()));
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

  public static MechanismPosition intakeToEndEffectorPassOffPosition() {
    return new MechanismPosition(
        Units.Inches.of(passOffElevatorHeightInches.get()),
        Rotation2d.fromDegrees(passOffArmAngleDegrees.get()));
  }

  public static MechanismPosition clearAlgaeHighPosition() {
    return new MechanismPosition(
        Units.Inches.of(algaeClearingHighElevatorHeightInches.get()),
        Rotation2d.fromDegrees(algaeClearingHighArmAngleDegrees.get()));
  }

  public static MechanismPosition stowPosition() {
    return new MechanismPosition(
        Units.Inches.of(stowElevatorHeightInches.get()),
        Rotation2d.fromDegrees(stowArmAngleDegrees.get()));
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
