package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.RobotStates.ScoringLevel;

public class MechanismPositions {
  private static final TunableNumberGroup group = new TunableNumberGroup("MechanismPositions");

  public static final LoggedTunableNumber stowElevatorHeightInches =
      group.build("robot/ElevatorHeightInches", 11.6); // 17.0

  private static final LoggedTunableNumber reefArmAngleDegrees =
      group.build("reef/Score/ArmAngleDegrees", 43); // 35.0

  private static final LoggedTunableNumber reefL2L3ArmAngleDegrees =
      group.build("reef/Score/L2L3ArmAngleDegrees", 35);

  private static final LoggedTunableNumber reefL4ArmAngleDegrees =
      group.build("reef/Score/L4ArmAngleDegrees", 90);

  private static final LoggedTunableNumber reefL1ElevatorHeightInches =
      group.build(
          "reef/Level1/ElevatorHeightInches", 20); // just above where game manual has it set

  private static final LoggedTunableNumber reefL2ElevatorHeightInches =
      group.build(
          "reef/Level2/ElevatorHeightInches", 32); // just above where game manual has it set

  private static final LoggedTunableNumber reefL3ElevatorHeightInches =
      group.build(
          "reef/Level3/ElevatorHeightInches", 48); // just above where game manual has it set

  private static final LoggedTunableNumber reefL4ElevatorHeightInches =
      group.build(
          "reef/Level4/ElevatorHeightInches",
          72); // 72 inches IS 6 feet, probably change this value

  private static final LoggedTunableNumber coralStationElevatorHeightInches =
      group.build("coralStation/ElevatorHeightInches", 38);

  private static final LoggedTunableNumber coralStationArmAngleDegrees =
      group.build("coralStation/ArmAngleDegrees", 55);

  private static final LoggedTunableNumber climbPrepArmAngleDegrees =
      group.build("climb/Prep/ArmAngleDegrees", Constants.ArmConstants.MAX_ARM_ANGLE.getDegrees());

  public record MechanismPosition(Distance elevatorHeight, Rotation2d armAngle) {}

  public static MechanismPosition stowPosition() {
    return new MechanismPosition(
        Units.Inches.of(stowElevatorHeightInches.get()),
        Rotation2d.fromDegrees(reefArmAngleDegrees.get()));
  }

  public static MechanismPosition reefPosition(ScoringLevel scoringLevel) {
    switch (scoringLevel) {
        case L1:
            return new MechanismPosition(
                Units.Inches.of(reefL1ElevatorHeightInches.get()),
                Rotation2d.fromDegrees(reefArmAngleDegrees.get()));
        case L2:
            return new MechanismPosition(
                Units.Inches.of(reefL2ElevatorHeightInches.get()),
                Rotation2d.fromDegrees(reefL2L3ArmAngleDegrees.get()));
        case L3:
            return new MechanismPosition(
                Units.Inches.of(reefL3ElevatorHeightInches.get()),
                Rotation2d.fromDegrees(reefL2L3ArmAngleDegrees.get()));
        case L4:
            return new MechanismPosition(
                Units.Inches.of(reefL4ElevatorHeightInches.get()),
                Rotation2d.fromDegrees(reefL4ArmAngleDegrees.get()));
        default:
            return new MechanismPosition(
                Units.Inches.of(reefL1ElevatorHeightInches.get()),
                Rotation2d.fromDegrees(reefArmAngleDegrees.get()));
    }
  }

  public static MechanismPosition coralStationPosition() {
    return new MechanismPosition(
        Units.Inches.of(coralStationElevatorHeightInches.get()),
        Rotation2d.fromDegrees(coralStationArmAngleDegrees.get()));
  }
}
