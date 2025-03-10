package frc.lib.team2930;

import frc.robot.RobotStates;
import frc.robot.autonomous.AutoStateMachine;
import frc.robot.autonomous.records.AutoDescriptor;
import frc.robot.autonomous.records.CoralStationLocation;
import frc.robot.autonomous.records.ScoringLocation;
import org.junit.jupiter.api.Test;

public class ChoreoPathsTest {
  @Test()
  public void testAllPaths() {
    for (AutoDescriptor.StartingLocation startLocation : AutoDescriptor.StartingLocation.values()) {
      for (ScoringLocation.ReefSide reefSide : ScoringLocation.ReefSide.values()) {
        ScoringLocation scoringLocation =
            new ScoringLocation(reefSide, RobotStates.ScoringLevel.L4);
        AutoStateMachine.locationsToPath(startLocation, scoringLocation);
      }
    }

    for (CoralStationLocation coralStationLocation : CoralStationLocation.values()) {
      for (ScoringLocation.ReefSide reefSide : ScoringLocation.ReefSide.values()) {
        ScoringLocation scoringLocation =
            new ScoringLocation(reefSide, RobotStates.ScoringLevel.L4);
        AutoStateMachine.locationsToPath(coralStationLocation, scoringLocation);

        AutoStateMachine.locationsToPath(scoringLocation, coralStationLocation);
      }
    }
  }
}
