package frc.robot;

import frc.robot.autonomous.records.ScoringLocation.ReefSide;

public class RobotStates {
  public static boolean clearingAlgae = false;

  public static ScoringLevel scoringLevel = ScoringLevel.L4;

  public static boolean algaeInRobot = false;

  // TODO: update these coral values
  public static boolean coralInRobot = true;

  public static boolean coralInEndEffector = true;

  public static boolean coralInEndEffectorScoringSide = true;

  public static boolean coralInEndEffectorNonScoringSide = true;

  public static boolean coralInIntake = false;

  public static ReefSide targetReefSide = ReefSide.CA;

  public enum ScoringLevel {
    L1,
    L2,
    L3,
    L4
  }
}
