package frc.robot.subsystems.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotStates;
import frc.robot.RobotStates.ScoringLevel;

public class MechanismPositions {
  private static final TunableNumberGroup group = new TunableNumberGroup("MechanismPositions");

  public static final LoggedTunableNumber lowStowElevatorHeightInches =
      group.build("Reef/ScorePrep/LowStowElevatorHeightInches");

  public static final LoggedTunableNumber highStowElevatorHeightInches =
      group.build("Reef/ScorePrep/HighStowElevatorHeightInches");

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
      group.build("PassOff/Pass/ElevatorHeightInches");

  private static final LoggedTunableNumber passOffArmAngleDegrees =
      group.build("PassOff/Pass/ArmAngleDegrees");

  private static final LoggedTunableNumber prepForPassOffElevatorHeightInches =
      group.build("PassOff/Prep/ElevatorHeightInches");

  private static final LoggedTunableNumber prepForPassOffArmAngleDegrees =
      group.build("PassOff/Prep/ArmAngleDegrees");

  private static final LoggedTunableNumber grabAlgaeHighElevatorHeightIntahces =
      group.build("grabAlgae/High/ElevatorHeightInches");

  private static final LoggedTunableNumber grabAlgaeHighArmAngleDegrees =
      group.build("grabAlgae/High/ArmAngleDegrees");

  private static final LoggedTunableNumber holdAlgaeElevatorHeightInches =
      group.build("holdAlgae/ElevatorHeightInches");

  private static final LoggedTunableNumber holdAlgaeArmAngleDegrees =
      group.build("holdAlgae/ArmAngleDegrees");

  private static final LoggedTunableNumber avoidIntakeElevatorHeightInches =
      group.build("avoidIntake/ElevatorHeightInches");

  private static final LoggedTunableNumber avoidIntakeArmAngleDegrees =
      group.build("avoidIntake/ArmAngleDegrees");

  private static final LoggedTunableNumber defaultElevatorHeightInches =
      group.build("default/ElevatorHeightInches");

  private static final LoggedTunableNumber defaultArmAngleDegrees =
      group.build("default/ArmAngleDegrees");

  private static final LoggedTunableNumber autoPrepElevatorHeightInches =
      group.build("autoPrep/ElevatorHeightInches");

  private static final LoggedTunableNumber autoPrepArmAngleDegrees =
      group.build("autoPrep/ArmAngleDegrees");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
      lowStowElevatorHeightInches.initDefault(2);
      highStowElevatorHeightInches.initDefault(2);
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
      reefL2ElevatorHeightInches.initDefault(10);
      reefL2ArmAngleDegrees.initDefault(132.71);
      reefL3ElevatorHeightInches.initDefault(27.6);
      reefL3ArmAngleDegrees.initDefault(138.69);
      reefL4ElevatorHeightInches.initDefault(56.5);
      reefL4ArmAngleDegrees.initDefault(151);
      coralStationElevatorHeightInches.initDefault(33);
      coralStationArmAngleDegrees.initDefault(-40);
      algaeClearingLowElevatorHeightInches.initDefault(10);
      algaeClearingLowArmAngleDegrees.initDefault(166);
      algaeClearingHighElevatorHeightInches.initDefault(27.2);
      algaeClearingHighArmAngleDegrees.initDefault(153);
      lowStowElevatorHeightInches.initDefault(0);
      highStowElevatorHeightInches.initDefault(26);
      stowArmAngleDegrees.initDefault(90);
      intermediatePoseHighElevatorHeightInches.initDefault(26);
      intermediatePoseHighArmAngleDegrees.initDefault(70);
      intermediatePoseLowElevatorHeightInches.initDefault(0);
      intermediatePoseLowArmAngleDegrees.initDefault(70);
      intermediatePoseLowBackElevatorHeightInches.initDefault(0);
      intermediatePoseLowBackArmAngleDegrees.initDefault(130);
      reefPrepArmOffsetDegrees.initDefault(20);
      passOffArmAngleDegrees.initDefault(9.756);
      passOffElevatorHeightInches.initDefault(15);
      prepForPassOffArmAngleDegrees.initDefault(9.756);
      prepForPassOffElevatorHeightInches.initDefault(15);
      grabAlgaeHighArmAngleDegrees.initDefault(151);
      grabAlgaeHighElevatorHeightIntahces.initDefault(48);
      holdAlgaeArmAngleDegrees.initDefault(90);
      holdAlgaeElevatorHeightInches.initDefault(26);
      avoidIntakeElevatorHeightInches.initDefault(18);
      avoidIntakeArmAngleDegrees.initDefault(45);
      defaultElevatorHeightInches.initDefault(4);
      defaultArmAngleDegrees.initDefault(70);
      autoPrepElevatorHeightInches.initDefault(0);
      autoPrepArmAngleDegrees.initDefault(90);
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

  public static MechanismPosition climbPosition() {
    return new MechanismPosition(
        Units.Inches.of(reefL3ElevatorHeightInches.get()),
        Rotation2d.fromDegrees(reefL3ArmAngleDegrees.get()));
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

  public static MechanismPosition clearAlgaeLowPosition() {
    return new MechanismPosition(
        Units.Inches.of(algaeClearingLowElevatorHeightInches.get()),
        Rotation2d.fromDegrees(algaeClearingLowArmAngleDegrees.get()));
  }

  public static MechanismPosition clearAlgaeHighPosition() {
    return new MechanismPosition(
        Units.Inches.of(algaeClearingHighElevatorHeightInches.get()),
        Rotation2d.fromDegrees(algaeClearingHighArmAngleDegrees.get()));
  }

  public static MechanismPosition stowPosition(RobotStates states) {
    return states.scoringLevel != ScoringLevel.L4
        ? reefPosition(states.scoringLevel)
        : new MechanismPosition(
            Units.Inches.of(highStowElevatorHeightInches.get()),
            Rotation2d.fromDegrees(stowArmAngleDegrees.get()));
  }

  public static MechanismPosition prepForPassoffPosition() {
    return new MechanismPosition(
        Units.Inches.of(prepForPassOffElevatorHeightInches.get()),
        Rotation2d.fromDegrees(prepForPassOffArmAngleDegrees.get()));
  }

  public static MechanismPosition passoffPosition() {
    return new MechanismPosition(
        Units.Inches.of(passOffElevatorHeightInches.get()),
        Rotation2d.fromDegrees(passOffArmAngleDegrees.get()));
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

  public static MechanismPosition passOffAlgaePosition() {
    return new MechanismPosition(
        Units.Inches.of(grabAlgaeHighElevatorHeightIntahces.get()),
        Rotation2d.fromDegrees(grabAlgaeHighArmAngleDegrees.get()));
  }

  public static MechanismPosition holdAlgaePosition() {
    return new MechanismPosition(
        Units.Inches.of(holdAlgaeElevatorHeightInches.get()),
        Rotation2d.fromDegrees(holdAlgaeArmAngleDegrees.get()));
  }

  public static MechanismPosition avoidIntakePosition() {
    return new MechanismPosition(
        Units.Inches.of(avoidIntakeElevatorHeightInches.get()),
        Rotation2d.fromDegrees(avoidIntakeArmAngleDegrees.get()));
  }

  public static MechanismPosition defaultPosition() {
    return new MechanismPosition(
        Units.Inches.of(defaultElevatorHeightInches.get()),
        Rotation2d.fromDegrees(defaultArmAngleDegrees.get()));
  }

  public static MechanismPosition autoPrepPosition() {
    return new MechanismPosition(
        Units.Inches.of(autoPrepElevatorHeightInches.get()),
        Rotation2d.fromDegrees(autoPrepArmAngleDegrees.get()));
  }
}
