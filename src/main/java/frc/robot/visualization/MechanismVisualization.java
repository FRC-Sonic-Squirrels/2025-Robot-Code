package frc.robot.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.RobotStates;

public class MechanismVisualization {
  private static final LoggerGroup logGroup = LoggerGroup.build("Visualization");
  private static final LoggerEntry.StructArray<Pose3d> logMechanism =
      logGroup.buildStructArray(Pose3d.class, "Mechanism");
  private static final LoggerEntry.Struct<Pose3d> logCoralInEndEffector =
      logGroup.buildStruct(Pose3d.class, "CoralInEndEffector");
  private static final LoggerEntry.Struct<Pose2d> logTestPose =
      logGroup.buildStruct(Pose2d.class, "TestPose");
  private static final LoggerEntry.Struct<Pose3d> logArmCameraView =
      logGroup.buildStruct(Pose3d.class, "ArmCameraView");

  // Use these tunable numbers for testing different heights and angles manually
  private static final TunableNumberGroup tunableGroup = new TunableNumberGroup("Visualization");
  private static final LoggedTunableNumber tunableArmAngleDeg =
      tunableGroup.build("ArmAngleDeg", 0);
  private static final LoggedTunableNumber tunableElevatorHeightInches =
      tunableGroup.build("ElevatorHeightInches", 0);
  private static final LoggedTunableNumber tunableIntakeAngleDeg =
      tunableGroup.build("IntakeAngleDeg", 0);
  private static final LoggedTunableNumber tunableClimberAngleDeg =
      tunableGroup.build("ClimberAngleDeg", 0);

  private static final LoggedTunableNumber tunableX = tunableGroup.build("TunableX", 0);
  private static final LoggedTunableNumber tunableY = tunableGroup.build("TunableY", 0);
  private static final LoggedTunableNumber tunableZ = tunableGroup.build("TunableZ", 0);
  private static final LoggedTunableNumber tunableYaw = tunableGroup.build("TunableYaw", 0);
  private static final LoggedTunableNumber tunablePitch = tunableGroup.build("TunablePitch", 0);
  private static final LoggedTunableNumber tunableRoll = tunableGroup.build("TunableRoll", 0);

  private static Pose3d mechFirstStage = Constants.zeroPose3d;
  private static Pose3d mechSecondStage = Constants.zeroPose3d;
  private static Pose3d arm = Constants.zeroPose3d;
  private static Pose3d intake = Constants.zeroPose3d;
  private static Pose3d climber = Constants.zeroPose3d;

  private static Pose3d coralInEndEffector = Constants.zeroPose3d;

  private static Pose3d camViewFromArm = Constants.zeroPose3d;

  public static void updateVisualization(
      Pose2d robotPose,
      Distance elevatorHeight,
      Rotation2d armAngle,
      Rotation2d intakeAngle,
      Rotation2d climberAngle) {
    // TODO: input positions as variables, add to pose3ds
    double elevatorHeightMeters = elevatorHeight.in(Units.Meter);
    mechFirstStage =
        new Pose3d(0.0, 0.0, Math.max(0.0, elevatorHeightMeters - 0.691), new Rotation3d());
    mechSecondStage = new Pose3d(0.0, 0.0, elevatorHeightMeters, new Rotation3d());
    arm =
        new Pose3d(
            -0.0314,
            0.0,
            0.352 + elevatorHeightMeters,
            new Rotation3d(0, Math.toRadians(89 - armAngle.getDegrees()), 0));
    intake =
        new Pose3d(
            0.342,
            0.0,
            0.178,
            new Rotation3d(0, Math.toRadians(133 - intakeAngle.getDegrees()), 0));
    climber = new Pose3d(0.0, -0.3303, 0.1235, new Rotation3d(climberAngle.getRadians(), 0, 0));

    Pose3d rotatedArmPose = arm.rotateBy(new Rotation3d(robotPose.getRotation()));

    coralInEndEffector =
        new Pose3d(
                robotPose.getX() + rotatedArmPose.getX(),
                robotPose.getY() + rotatedArmPose.getY(),
                RobotStates.coralInEndEffector ? rotatedArmPose.getZ() : -100,
                rotatedArmPose.getRotation())
            .transformBy(new Transform3d(0, 0, 0.46, new Rotation3d(0, Math.toRadians(5), 0)));

    camViewFromArm =
        new Pose3d(
                robotPose.getX() + rotatedArmPose.getX(),
                robotPose.getY() + rotatedArmPose.getY(),
                rotatedArmPose.getZ(),
                rotatedArmPose.getRotation())
            .transformBy(
                new Transform3d(
                    tunableX.get(),
                    0,
                    tunableZ.get(),
                    new Rotation3d(0, Math.toRadians(tunablePitch.get()), 0)));
  }

  public static void logMechanism() {
    logMechanism.info(new Pose3d[] {mechFirstStage, mechSecondStage, arm, intake, climber});
    logTestPose.info(Constants.zeroPose2d);
    logCoralInEndEffector.info(coralInEndEffector);
    logArmCameraView.info(camViewFromArm); // TODO: make this work
  }
}
