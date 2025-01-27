package frc.robot.autonomous.records;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomous.helpers.ChoreoHelper;

public record ChoreoTrajectoryWithName(String name, Trajectory<SwerveSample> states) {
  public static ChoreoTrajectoryWithName getTrajectory(String name) {
    if (name == null) return null;
    return new ChoreoTrajectoryWithName(
        name, (Trajectory<SwerveSample>) Choreo.loadTrajectory(name).orElseThrow());
  }

  public static String getName(ChoreoTrajectoryWithName trajWithName) {
    return trajWithName == null ? "NULL" : trajWithName.name();
  }

  public ChoreoTrajectoryWithName rescale(double speedScaling) {
    return new ChoreoTrajectoryWithName(name, ChoreoHelper.rescale(states, speedScaling));
  }

  public ChoreoTrajectoryWithName flipOnAlliance(boolean flip) {
    return flip ? new ChoreoTrajectoryWithName(name, ChoreoHelper.flipOnAlliance(states)) : this;
  }

  public Pose2d getInitialPose(boolean flipForAlliance) {
    return states.getInitialPose(flipForAlliance).orElseThrow();
  }

  public Pose2d getFinalPose(boolean flipForAlliance) {
    return states.getFinalPose(flipForAlliance).orElseThrow();
  }
}
