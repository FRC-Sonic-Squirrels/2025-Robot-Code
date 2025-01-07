package frc.robot.autonomous;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;

public record ChoreoTrajectoryWithName(String name, Trajectory<SwerveSample> states) {
  public static ChoreoTrajectoryWithName getTrajectory(String name) {
    if (name == null) return null;
    // TODO: add throw message?
    // TODO: check whether this cast is okay:
    return new ChoreoTrajectoryWithName(
        name, (Trajectory<SwerveSample>) Choreo.loadTrajectory(name).orElseThrow());
  }

  public static String getName(ChoreoTrajectoryWithName trajWithName) {
    return trajWithName == null ? "NULL" : trajWithName.name();
  }

  public ChoreoTrajectoryWithName rescale(double speedScaling) {
    return new ChoreoTrajectoryWithName(name, ChoreoHelper.rescale(states, speedScaling));
  }

  public Pose2d getInitialPose(boolean flipForAlliance) {
    // TODO: add throw message?
    return states.getInitialPose(flipForAlliance).orElseThrow();
  }

  public Pose2d getFinalPose(boolean flipForAlliance) {
    // TODO: add throw message?
    return states.getFinalPose(flipForAlliance).orElseThrow();
  }
}
