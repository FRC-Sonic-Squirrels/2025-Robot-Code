package frc.robot;

import frc.robot.RobotStates.ScoringLevel;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.autonomous.records.ScoringLocation.ReefSide;

public class FieldStates {
  private static boolean[][] scoredLocations =
      new boolean[ScoringLevel.values().length][ReefSide.values().length];

  public static boolean isScoringLocationFilled(ScoringLocation location) {
    return false;
  }

  public static void setScoringLocationFilled(ScoringLocation location) {}
}
