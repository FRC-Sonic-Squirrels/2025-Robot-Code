package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.autonomous.records.ScoringLocation.ReefSide;
import java.util.ArrayList;
import java.util.List;

public class FieldStates {
  private static boolean[][] scoredLocations =
      new boolean[ScoringLevel.values().length][ReefSide.values().length];

  public static boolean isScoringLocationFilled(ScoringLocation location) {
    return scoredLocations[location.level().ordinal()][location.side().ordinal()];
  }

  public static void setScoringLocationFilled(ScoringLocation location) {
    scoredLocations[location.level().ordinal()][location.side().ordinal()] = true;
  }

  public static boolean scoringLevelFilled(ScoringLevel level) {
    for (ReefSide side : ReefSide.values()) {
      if (!isScoringLocationFilled(new ScoringLocation(side, level))) return false;
    }
    return true;
  }

  private static final TunableNumberGroup tunableGroup = new TunableNumberGroup("FieldStates");
  private static final LoggedTunableNumber tunableX = tunableGroup.build("TunableX", 3.758);
  private static final LoggedTunableNumber tunableY = tunableGroup.build("TunableY", 4.19);
  private static final LoggedTunableNumber tunableZ = tunableGroup.build("TunableZ", 1.108);
  private static final LoggedTunableNumber tunableYaw = tunableGroup.build("TunableYaw", 0);
  private static final LoggedTunableNumber tunablePitch = tunableGroup.build("TunablePitch", 35);
  private static final LoggedTunableNumber tunableRoll = tunableGroup.build("TunableRoll", 0);

  private static final LoggerGroup logGroup = LoggerGroup.build("FieldStates");
  private static final LoggerEntry.Struct<Pose3d> testPose =
      logGroup.buildStruct(Pose3d.class, "TestPose");
  private static final LoggerEntry.StructArray<Pose3d> coralPoses =
      logGroup.buildStructArray(Pose3d.class, "CoralPoses");

  private static final Pose3d[] blueAPose3ds =
      new Pose3d[] {
        new Pose3d(3.725, 4.19, 0.485, new Rotation3d(0, 0, Math.toRadians(90))),
        new Pose3d(3.786, 4.19, 0.705, new Rotation3d(0, Math.toRadians(35), 0)),
        new Pose3d(3.786, 4.19, 1.1075, new Rotation3d(0, Math.toRadians(35), 0)),
        new Pose3d(3.69, 4.19, 1.74, new Rotation3d(0, Math.toRadians(85), 0))
      };
  private static final Pose3d[] blueBPose3ds =
      new Pose3d[] {
        new Pose3d(3.725, 3.86, 0.485, new Rotation3d(0, 0, Math.toRadians(90))),
        new Pose3d(3.786, 3.86, 0.705, new Rotation3d(0, Math.toRadians(35), 0)),
        new Pose3d(3.786, 3.86, 1.1075, new Rotation3d(0, Math.toRadians(35), 0)),
        new Pose3d(3.69, 3.86, 1.74, new Rotation3d(0, Math.toRadians(85), 0))
      };

  public static void logGamepieceVisualization() {
    List<Pose3d> poses = new ArrayList<>();
    int scoredGamepieces = 0;
    for (int columb = 0; columb < scoredLocations.length; columb++) {
      for (int row = 0; row < scoredLocations[0].length; row++) {
        if (scoredLocations[columb][row]) {
          Pose3d referencePose = row % 2 == 1 ? blueAPose3ds[columb] : blueBPose3ds[columb];

          poses.add(
              GeometryUtil.rotatePose3dAroundTranslation2d(
                  referencePose,
                  Constants.FieldConstants.BLUE_REEF_CENTER_POSE,
                  Rotation2d.fromDegrees(60 * (row / 2))));

          scoredGamepieces++;
        }
      }
    }

    testPose.info(
        new Pose3d(
            tunableX.get(),
            tunableY.get(),
            tunableZ.get(),
            new Rotation3d(
                Math.toRadians(tunableRoll.get()),
                Math.toRadians(tunablePitch.get()),
                Math.toRadians(tunableYaw.get()))));
    coralPoses.info(poses.toArray(new Pose3d[scoredGamepieces]));
  }
}
