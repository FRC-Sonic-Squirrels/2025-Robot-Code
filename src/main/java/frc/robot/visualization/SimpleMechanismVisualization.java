package frc.robot.visualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SimpleMechanismVisualization { // TODO: vizualize mech on other side of robot
  private static final LoggerGroup logGroup = LoggerGroup.build("Mechanism");
  private static final LoggerEntry.Mechanism logMech = logGroup.buildMechanism2d("SimpleMechanism");

  static LoggedMechanism2d mechanism2d =
      new LoggedMechanism2d(
          Units.Inches.of(32.0).in(Units.Meters), Units.Inches.of(60.0).in(Units.Meters));

  static LoggedMechanismRoot2d mechRoot =
      mechanism2d.getRoot(
          "mechRoot", Units.Inches.of(9.75).in(Units.Meters), Units.Inches.of(4).in(Units.Meters));

  static LoggedMechanismLigament2d elevatorLigament =
      mechRoot.append(new LoggedMechanismLigament2d("mech", 0.001, 90));

  static LoggedMechanismLigament2d armLigament =
      elevatorLigament.append(
          new LoggedMechanismLigament2d("mech", ArmConstants.ARM_LENGTH.in(Units.Meters), 90));

  static {
    elevatorLigament.setColor(new Color8Bit(0, 0, 255));
    elevatorLigament.setLineWeight(15.0);
    armLigament.setLineWeight(15.0);
  }

  public static void updateVisualization(Distance elevatorHeight, Rotation2d armAngle) {
    if (elevatorHeight.in(Units.Inch) != 0.0)
      elevatorLigament.setLength(elevatorHeight.in(Units.Meters));
    armLigament.setAngle(armAngle.unaryMinus().plus(Rotation2d.k180deg));
  }

  public static void logMechanism() {
    logMech.info(mechanism2d);
  }
}
