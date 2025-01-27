package frc.robot.autonomous.records;

import frc.robot.RobotStates.ScoringLevel;

public record ScoringLocation(ReefSide side, ScoringLevel level) {
  public enum ReefSide {
    CA,
    CB,
    CC,
    CD,
    CE,
    CF,
    CG,
    CH,
    CI,
    CJ,
    CK,
    CL
  }
}
