package frc.robot.autonomous.records;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.team2930.AllianceFlipUtil;
import frc.robot.Constants;
import frc.robot.autonomous.helpers.ChoreoHelper;
import java.util.ArrayList;
import java.util.Optional;

public record ChoreoTrajectoryWithName(String name, Trajectory<SwerveSample> states) {
  public static ChoreoTrajectoryWithName getTrajectory(String name) {
    if (name == null) return null;
    Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);
    if (traj.isEmpty()) return null;
    return new ChoreoTrajectoryWithName(name, traj.orElseThrow());
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

  public ChoreoTrajectoryWithName flipForAlliance() {
    if (Constants.isRedAlliance()) {
      ArrayList<SwerveSample> newSamples = new ArrayList<>();
      for (SwerveSample sample : this.states.samples()) {
        newSamples.add(flipSample(sample));
      }
      Trajectory<SwerveSample> newPath =
          new Trajectory<>(name, newSamples, new ArrayList<>(), new ArrayList<>());
      return new ChoreoTrajectoryWithName(name, newPath);
    } else {
      return this;
    }
  }

  public SwerveSample flipSample(SwerveSample sample) {
    Pose2d oldPoint = sample.getPose();
    Pose2d newPoint = AllianceFlipUtil.rotatePose2DAroundCenterPoint(oldPoint);
    double[] newModuleForcesX = new double[sample.moduleForcesX().length];
    double[] newModuleForcesY = new double[sample.moduleForcesY().length];
    for (int i = 0; i < 4; i++) {
      newModuleForcesX[i] = -sample.moduleForcesX()[i];
      newModuleForcesY[i] = -sample.moduleForcesY()[i];
    }
    return new SwerveSample(
        sample.t,
        newPoint.getX(),
        newPoint.getY(),
        newPoint.getRotation().getRadians(),
        -sample.vx,
        -sample.vy,
        sample.omega,
        -sample.ax,
        -sample.ay,
        sample.alpha,
        sample.moduleForcesX(),
        null);
  }

  public Pose2d getInitialPose(boolean flipForAlliance) {
    return states.getInitialPose(flipForAlliance).orElseThrow();
  }

  public Pose2d getFinalPose(boolean flipForAlliance) {
    return states.getFinalPose(flipForAlliance).orElseThrow();
  }
}
