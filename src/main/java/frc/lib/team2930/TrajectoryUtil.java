package frc.lib.team2930;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtil {
  public static Trajectory<SwerveSample> pathPlannerToChoreo(
      PathPlannerTrajectory pathPlannerTrajectory) {
    List<PathPlannerTrajectoryState> states = pathPlannerTrajectory.getStates();
    List<SwerveSample> swerveSamples = new ArrayList<>();
    for (PathPlannerTrajectoryState state : states) {
      Pose2d pose = state.pose;
      swerveSamples.add(
          new SwerveSample(
              state.timeSeconds,
              pose.getX(),
              pose.getY(),
              pose.getRotation().getRadians(),
              state.fieldSpeeds.vxMetersPerSecond,
              state.fieldSpeeds.vyMetersPerSecond,
              state.fieldSpeeds.omegaRadiansPerSecond,
              0,
              0,
              0,
              null,
              null));
    }
    return new Trajectory<>(null, swerveSamples, null, null);
  }
}
