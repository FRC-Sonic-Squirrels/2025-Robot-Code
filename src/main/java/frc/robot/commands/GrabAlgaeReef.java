package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.StateMachine;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ScoringSideWithPose;
import frc.robot.RobotStates;
import frc.robot.RobotStates.EndEffectorDesiredAction;
import frc.robot.RobotStates.MechState;
import java.util.function.Supplier;

public class GrabAlgaeReef extends StateMachine {

  private final RobotStates states;
  private final Supplier<Pose2d> robotPose;

  public GrabAlgaeReef(RobotStates states, Supplier<Pose2d> robotPose) {
    super("GrabAlgaeReef");

    this.states = states;
    this.robotPose = robotPose;

    setInitialState(stateWithName("GrabAlgae", () -> grabAlgae()));
    setInterruptedState(stateWithName("End", () -> end()));
  }

  private StateHandler grabAlgae() {
    states.coralExpectedInEndEffector = false;
    states.mechState =
        closerToLowAlgae()
            ? MechState.GrabAlgaeFromReefLow
            : MechState
                .GrabAlgaeFromReefHigh; // Bring mechanism to different positions based on which
    // algae is closer
    states.endEffectorDesiredAction = EndEffectorDesiredAction.HoldAlgae;
    return null;
  }

  private StateHandler end() {
    states.mechState = MechState.HoldAlgaePosition;
    return setDone();
  }

  private boolean closerToLowAlgae() {

    ScoringSideWithPose bestSide = null;

    for (ScoringSideWithPose side : Constants.FieldConstants.SCORING_SIDES(Units.Inches.of(0))) {
      if (bestSide == null
          || GeometryUtil.getDist(side.pose(), robotPose.get())
              < GeometryUtil.getDist(bestSide.pose(), robotPose.get())) bestSide = side;
    }

    return bestSide.side().hasAlgaeAtStartOnL2;
  }
}
