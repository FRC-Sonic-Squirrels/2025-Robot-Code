package frc.robot.autonomous.records;

import frc.robot.RobotStates.ScoringLevel;
import frc.robot.commands.ScoreCoral.ScoringDirection;
import frc.robot.commands.ScoreCoral.ScoringSide;

public record ScoringLocation(ScoringSide side, ScoringDirection direction, ScoringLevel level) {}
