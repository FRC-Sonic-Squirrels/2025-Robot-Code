package frc.robot.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;

public class MechanismVisualization {
  private static final LoggerGroup logGroup = LoggerGroup.build("Visualization");
  private static final LoggerEntry.StructArray<Pose3d> logMechanism =
      logGroup.buildStructArray(Pose3d.class, "Mechanism");
  private static final LoggerEntry.Struct<Pose2d> logTestPose =
      logGroup.buildStruct(Pose2d.class, "TestPose");

  private static final TunableNumberGroup tunableGroup = new TunableNumberGroup("Visualization");
  private static final LoggedTunableNumber tunableArmAngleDeg = tunableGroup.build("ArmAngleDeg", 0);
  private static final LoggedTunableNumber tunableElevatorHeightInches = tunableGroup.build("ElevatorHeightInches", 0);
  private static final LoggedTunableNumber tunableIntakeAngleDeg = tunableGroup.build("IntakeAngleDeg", 0);
  private static final LoggedTunableNumber tunableClimberAngleDeg = tunableGroup.build("ClimberAngleDeg", 0);

  private static Pose3d mechFirstStage = Constants.zeroPose3d;
  private static Pose3d mechSecondStage = Constants.zeroPose3d;
  private static Pose3d arm = Constants.zeroPose3d;
  private static Pose3d intake = Constants.zeroPose3d;
  private static Pose3d climber = Constants.zeroPose3d;

  public static void updateVisualization(double elevatorHeightInches, double armAngleDeg, double intakeAngleDeg, double climberAngleDeg) {
    // TODO: input positions as variables, add to pose3ds
    double elevatorHeightMeters = Units.Inches.of(elevatorHeightInches).in(Units.Meter);
    mechFirstStage = new Pose3d(0.0, 0.0, Math.max(0.0, elevatorHeightMeters - 0.691), new Rotation3d());
    mechSecondStage = new Pose3d(0.0, 0.0, elevatorHeightMeters, new Rotation3d());
    arm = new Pose3d(-0.0314, 0.0, 0.352 + elevatorHeightMeters, new Rotation3d(0, Math.toRadians(89-armAngleDeg), 0));
    intake = new Pose3d(0.342, 0.0, 0.178, new Rotation3d(0, Math.toRadians(intakeAngleDeg + 43), 0));
    climber = new Pose3d(0.0, -0.3303, 0.1235, new Rotation3d(Math.toRadians(climberAngleDeg), 0, 0));  
  }

  public static void logMechanism() {
    logMechanism.info(new Pose3d[] {mechFirstStage, mechSecondStage, arm, intake, climber});
    logTestPose.info(Constants.zeroPose2d);
  }
}
