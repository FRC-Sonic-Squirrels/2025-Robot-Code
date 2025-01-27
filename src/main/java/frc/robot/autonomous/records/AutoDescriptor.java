package frc.robot.autonomous.records;

import java.util.List;

public record AutoDescriptor(
    List<ScoringLocation> scoringLocations,
    List<CoralStationLocation> coralStationLocations,
    StartingLocation startingLocation) {
  public enum StartingLocation {
    S1
  }
}
