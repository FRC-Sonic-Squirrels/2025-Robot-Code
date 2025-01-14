package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;

public class MechanismPositions {
  private static final TunableNumberGroup group = new TunableNumberGroup("MechanismPositions");

  public static final LoggedTunableNumber reefElevatorHeightInches =
      group.build("amp/Score/ElevatorHeightInches", 11.6); // 17.0

  private static final LoggedTunableNumber reefArmAngleDegrees =
      group.build("amp/Score/ArmAngleDegrees", 43); // 35.0

  private static final LoggedTunableNumber climbPrepElevatorHeightInches =
      group.build(
          "climb/Prep/ElevatorHeightInches",
          Constants.ElevatorConstants.MAX_LEGAL_HEIGHT.in(Units.Inches));

  private static final LoggedTunableNumber climbPrepArmAngleDegrees =
      group.build("climb/Prep/ArmAngleDegrees", Constants.ArmConstants.MAX_ARM_ANGLE.getDegrees());

  public record MechanismPosition(Distance elevatorHeight, Rotation2d armAngle) {}

  public static MechanismPosition reefPosition() {
    return new MechanismPosition(
        Units.Inches.of(reefElevatorHeightInches.get()),
        Rotation2d.fromDegrees(reefArmAngleDegrees.get()));
  }

  public static MechanismPosition climbPrepPosition() {
    return new MechanismPosition(
        Units.Inches.of(climbPrepElevatorHeightInches.get()),
        Rotation2d.fromDegrees(climbPrepArmAngleDegrees.get()));
  }
}
