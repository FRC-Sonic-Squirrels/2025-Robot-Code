package frc.robot.autonomous.records;

public record CoralStationLocation(CoralStation station, CoralStationSide side) {
  public enum CoralStation {
    LEFT,
    RIGHT
  }

  public enum CoralStationSide {
    LEFT,
    RIGHT
  }
}
