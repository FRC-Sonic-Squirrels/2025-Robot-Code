package frc.lib.team2930;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.configs.RobotConfig;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtil {

  private static TunableNumberGroup tunableGroup = new TunableNumberGroup("TrajectoryUtil");
  private static LoggedTunableNumber reefCirclingDistMeters =
      tunableGroup.build(
          "ReefCirclingDistMeters",
          Constants.FieldConstants.REEF_DIAGONAL_WIDTH
                  .plus(Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getMeasureY())
                  .div(2)
                  .in(Units.Meters)
              + 0.1);
  private static LoggedTunableNumber headingToleranceNearElements =
      tunableGroup.build("HeadingToleranceNearElements", 40.0);

  private static LoggerGroup logGroup = LoggerGroup.build("TrajectoryUtil");
  private static LoggerEntry.Struct<Pose2d> log_intermediatePose =
      logGroup.buildStruct(Pose2d.class, "IntermediatePose");
  private static LoggerEntry.Struct<Pose2d> log_intersectingPose =
      logGroup.buildStruct(Pose2d.class, "IntersectingPose");
  private static LoggerEntry.StructArray<Pose2d> log_unAttemptedPath =
      logGroup.buildStructArray(Pose2d.class, "UnAttemptedPath");

  private static boolean debugRotationClamping = false;
  private static boolean debugRerouting = false;
  private static boolean debugGenerateStartAndEndRotations = false;
  private static boolean debugIsRobotNextToReef = false;

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

  public static Trajectory<SwerveSample> generateTrajectory(
      Pose2d currentPose,
      Pose2d targetPose,
      Translation2d vel,
      RobotConfig config,
      ChassisSpeeds initSpeedsRobotRel) {

    if (GeometryUtil.getDist(currentPose, targetPose) < 0.0002) {
      return new Trajectory<SwerveSample>(
          "EMPTY", new ArrayList<>(), new ArrayList<>(), new ArrayList<>());
    }

    var rotations = generateStartAndEndRotations(targetPose, currentPose, currentPose, vel, config);

    return generatePath(
        rotations.getFirst(),
        rotations.getSecond(),
        currentPose,
        targetPose,
        vel,
        config,
        initSpeedsRobotRel);
  }

  private static boolean trajIntersectsWithReef(Trajectory<SwerveSample> traj) {
    for (SwerveSample sample : traj.samples()) {
      Pose2d pose = sample.getPose();
      double distToStart = GeometryUtil.getDist(pose, traj.getInitialPose(false).get());
      double distToEnd = GeometryUtil.getDist(pose, traj.getFinalPose(false).get());
      if (poseIntersectsWithReef(pose) && distToStart > 0.3 && distToEnd > 0.3) {
        log_intersectingPose.info(pose);
        return true;
      }
    }
    log_intersectingPose.info(Pose2d.kZero);
    return false;
  }

  private static boolean poseIntersectsWithReef(Pose2d pose) {
    if (!isRobotNextToReef(pose)) return false;

    for (Translation2d corner : getRobotCorners(pose)) {
      if (translationIntersectsWithReef(corner)) return true;
    }

    return false;
  }

  private static boolean translationIntersectsWithReef(Translation2d translation) {
    double reefx = Constants.FieldConstants.REEF_CENTER_POSE().getX();
    double reefy = Constants.FieldConstants.REEF_CENTER_POSE().getY();
    double translationx = translation.getX();
    double translationy = translation.getY();
    double reefRad = Constants.FieldConstants.REEF_DIAGONAL_WIDTH.div(2.0).in(Units.Meter);
    double angledSlope = Math.atan(Math.toRadians(30));
    double xOffset = translationx - reefx;
    // All sides of hexagon
    boolean check1 = translationx > reefx - reefRad;
    boolean check2 = translationx < reefx + reefRad;
    boolean check3 = translationy > reefy - reefRad + Math.abs(angledSlope * xOffset);
    boolean check4 = translationy < reefy + reefRad - Math.abs(angledSlope * xOffset);
    return check1 && check2 && check3 && check4;
  }

  private static Translation2d[] getRobotCorners(Pose2d pose) {
    Translation2d[] corners = new Translation2d[4];

    double xRad = Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getX() / 2.0;
    double yRad = Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getY() / 2.0;

    Translation2d[] robotRelativeCorners =
        new Translation2d[] {
          new Translation2d(xRad, yRad),
          new Translation2d(-xRad, yRad),
          new Translation2d(xRad, -yRad),
          new Translation2d(-xRad, -yRad)
        };

    for (int i = 0; i < corners.length; i++) {
      corners[i] =
          pose.getTranslation()
              .plus(
                  robotRelativeCorners[i].rotateBy(
                      pose.getRotation())); // TODO: change to corners of bumpers
    }
    return corners;
  }

  private static List<Trajectory<SwerveSample>> getPaths(
      Trajectory<SwerveSample> path, double[] cutTimes) {
    List<Trajectory<SwerveSample>> paths = new ArrayList<>();
    List<SwerveSample> samples = path.samples();

    int start = 0;

    for (int i = 0; i < cutTimes.length; i++) {
      int end = findIndexOfTime(path, cutTimes[i]);
      paths.add(getSubpath(samples, start, end));
      start = end;
    }

    paths.add(getSubpath(samples, start, samples.size()));

    return paths;
  }

  private static Trajectory<SwerveSample> getSubpath(
      List<SwerveSample> samples, int start, int end) {
    List<SwerveSample> sublist = samples.subList(start, end);
    return new Trajectory<SwerveSample>("", sublist, null, null);
  }

  private static int findIndexOfTime(Trajectory<SwerveSample> path, double time) {
    SwerveSample sample = path.sampleAt(time, false).get();
    List<SwerveSample> samples = path.samples();
    for (int i = 0; i < samples.size(); i++) if (samples.get(i).equals(sample)) return i;
    return -1;
  }

  private static Trajectory<SwerveSample> rerouteTrajectory(
      Trajectory<SwerveSample> traj,
      Pose2d start,
      Pose2d end,
      RobotConfig config,
      Translation2d vel) {

    if (!trajIntersectsWithReef(traj)) {
      log_intermediatePose.info(Pose2d.kZero);
      log_unAttemptedPath.info(new Pose2d[0]);
      return traj;
    }

    if (debugRerouting) {
      System.out.println("START REROUTING DEBUG -------------------");
    }

    log_unAttemptedPath.info(traj.getPoses());

    Translation2d reefCenter = Constants.FieldConstants.REEF_CENTER_POSE();
    Rotation2d reefCenterToStartHeading =
        GeometryUtil.getHeading(reefCenter, start.getTranslation());
    Rotation2d reefCenterToEndHeading = GeometryUtil.getHeading(reefCenter, end.getTranslation());
    Rotation2d halfWayAngle = midRotation(reefCenterToStartHeading, reefCenterToEndHeading);
    Translation2d midTranslation =
        reefCenter.plus(new Translation2d(reefCirclingDistMeters.get(), halfWayAngle));

    double reefCenterToStartHeadingDeg = reefCenterToStartHeading.getDegrees();
    double reefCenterToEndHeadingDeg = reefCenterToEndHeading.getDegrees();
    double halfWayAngleDeg = halfWayAngle.getDegrees();

    double reefCenterToStartHeadingDegOptimized =
        GeometryUtil.optimizeRotationInDegrees(reefCenterToStartHeading.getDegrees());
    double halfWayAngleDegOptimized =
        GeometryUtil.optimizeRotationInDegrees(halfWayAngle.getDegrees());

    if (debugRerouting) {
      System.out.println(
          "Headings: reefCenterToStartHeading: "
              + reefCenterToStartHeadingDeg
              + " reefCenterToEndHeading: "
              + reefCenterToEndHeadingDeg
              + " halfWayAngle: "
              + halfWayAngleDeg);

      System.out.println(
          "HeadingsOptimized: reefCenterToStartHeading: "
              + reefCenterToStartHeadingDegOptimized
              + " halfWayAngle: "
              + halfWayAngleDegOptimized);
    }

    boolean flipRotation =
        (Math.abs(reefCenterToStartHeadingDegOptimized - 180)
                    + Math.abs(halfWayAngleDegOptimized + 180)
                < Math.abs(reefCenterToStartHeadingDegOptimized - halfWayAngleDegOptimized))
            || (Math.abs(reefCenterToStartHeadingDegOptimized - 180)
                    + Math.abs(halfWayAngleDegOptimized + 180)
                < Math.abs(reefCenterToStartHeadingDegOptimized - halfWayAngleDegOptimized));

    Pose2d midWaypoint =
        new Pose2d(
            midTranslation,
            halfWayAngle.plus(
                logicalXOR(
                        reefCenterToStartHeadingDegOptimized < halfWayAngleDegOptimized,
                        flipRotation)
                    ? Rotation2d.kCCW_90deg
                    : Rotation2d.kCW_90deg));
    log_intermediatePose.info(midWaypoint);

    Pair<Rotation2d, Rotation2d> newRotations =
        generateStartAndEndRotations(midWaypoint, midWaypoint, start, vel, config);

    start = new Pose2d(start.getTranslation(), newRotations.getFirst());
    end = new Pose2d(end.getTranslation(), newRotations.getSecond());

    if (debugRerouting) {
      System.out.println("END REROUTING DEBUG -------------------");
    }

    return generateSimplePath(
        PathPlannerPath.waypointsFromPoses(start, midWaypoint, end),
        end.getRotation(),
        start.getRotation(),
        config,
        new ChassisSpeeds());
  }

  private static Rotation2d midRotation(Rotation2d a, Rotation2d b) {
    double aDeg = a.getDegrees();
    double bDeg = b.getDegrees();

    double higher = Math.max(aDeg, bDeg);
    double lower = Math.min(aDeg, bDeg);

    double wrappedDist = 360 - higher + lower;
    double regDist = higher - lower;

    if (regDist < wrappedDist) {
      return a.plus(b).div(2);
    } else {
      return Rotation2d.fromDegrees(wrappedDist / 2.0 + higher);
    }
  }

  private static Trajectory<SwerveSample> generatePath(
      Rotation2d startDirection,
      Rotation2d endDirection,
      Pose2d initPose,
      Pose2d targetPose,
      Translation2d vel,
      RobotConfig config,
      ChassisSpeeds initSpeedsRobotRel) {

    Pose2d startPoint = new Pose2d(initPose.getTranslation(), startDirection);

    Pose2d endPoint = new Pose2d(targetPose.getTranslation(), endDirection);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPoint, endPoint);

    Trajectory<SwerveSample> traj =
        generateSimplePath(
            waypoints,
            targetPose.getRotation(),
            initPose.getRotation(),
            config,
            initSpeedsRobotRel);
    if (traj != null) {
      traj = rerouteTrajectory(traj, startPoint, endPoint, config, vel);
    }

    return traj;
  }

  private static Trajectory<SwerveSample> generateSimplePath(
      List<Waypoint> waypoints,
      Rotation2d targetRot,
      Rotation2d initRot,
      RobotConfig config,
      ChassisSpeeds initSpeedsRobotRel) {
    PathConstraints constraints =
        new PathConstraints(
            config.getPathingMaxSpeedMPS().get(),
            config.getPathingMaxAccelerationMPSPS().get(),
            config.getPathingMaxAngularVelocityRadPerSecond().get(),
            config.getPathingMaxAngularAccelerationRadPerSecondSquared().get());

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0, // TODO: consider having this not be 0? Slam into reef/coral station a little
                // bit?
                targetRot) // Goal end state. You can set a holonomic rotation here. If
            // using a
            // differential drivetrain, the rotation will have no effect.
            );

    path.preventFlipping = true;

    if (path.numPoints() < 2) {
      return null;
    }

    PathPlannerTrajectory traj =
        path.generateTrajectory(initSpeedsRobotRel, initRot, config.pathPlannerConfig());

    return TrajectoryUtil.pathPlannerToChoreo(traj);
  }

  private static double getRobotDistToReef(Pose2d pose) {
    return GeometryUtil.getDist(pose.getTranslation(), Constants.FieldConstants.REEF_CENTER_POSE());
  }

  private static boolean isRobotNextToReef(Pose2d pose) {
    if (debugIsRobotNextToReef) {
      System.out.println("START IS ROBOT NEXT TO REEF DEBUG ----------------");
    }

    double robotDistToReef = getRobotDistToReef(pose);
    double minAllowedDistanceAway =
        Constants.FieldConstants.REEF_DIAGONAL_WIDTH.div(2.0).in(Units.Meters)
            + (Math.max(
                    Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getX(),
                    Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getY())
                / 2.0);

    if (debugIsRobotNextToReef) {

      System.out.println(
          "Distances: RobotDistToReef: "
              + robotDistToReef
              + " MinAllowedDistanceAway: "
              + minAllowedDistanceAway);

      System.out.println("END IS ROBOT NEXT TO REEF DEBUG ----------------");
    }

    return robotDistToReef < minAllowedDistanceAway;
  }

  private static double clampRotation(double rot, double topBound, double bottomBound) {
    if (debugRotationClamping) {
      System.out.println("START CLAMPING DEBUG -----------------");
      System.out.println(
          "Inputs: rot: " + rot + " topBound: " + topBound + " bottomBound: " + bottomBound);
    }

    rot = GeometryUtil.optimizeRotationInDegrees(rot);
    topBound = GeometryUtil.optimizeRotationInDegrees(topBound);
    bottomBound = GeometryUtil.optimizeRotationInDegrees(bottomBound);

    if (debugRotationClamping) {
      System.out.println(
          "Optimized: rot: " + rot + " topBound: " + topBound + " bottomBound: " + bottomBound);
    }

    boolean inside = bottomBound < topBound;

    if (debugRotationClamping) {
      System.out.println("Inside? " + inside);
    }

    if (inside) {
      if (!(rot < topBound && rot > bottomBound)) {
        double minTopDistance = -1;
        double minBottomDistance = -1;
        for (int i = 0; i < 3; i++) {
          double testTopDistance = Math.abs(topBound + 360 * (1 - i) - rot);
          double testBottomDistance = Math.abs(bottomBound + 360 * (1 - i) - rot);

          if (debugRotationClamping) {
            System.out.println("testTopDistance" + i + ": " + testTopDistance);
            System.out.println("testBottomDistance" + i + ": " + testBottomDistance);
          }

          if (minTopDistance == -1 || Math.abs(testTopDistance - rot) < minTopDistance) {

            minTopDistance = testTopDistance;
          }

          if (minBottomDistance == -1 || Math.abs(testBottomDistance - rot) < minBottomDistance) {

            minBottomDistance = testBottomDistance;
          }
        }

        if (debugRotationClamping) {
          System.out.println("minTopDistance: " + minTopDistance);
          System.out.println("minBottomDistance: " + minBottomDistance);
        }

        rot = minTopDistance < minBottomDistance ? topBound : bottomBound;
      }
    } else {
      if (rot < bottomBound && rot > topBound) {
        if (Math.abs(rot - bottomBound) < Math.abs(rot - topBound)) {
          rot = bottomBound;
        } else {
          rot = topBound;
        }
      }
    }

    if (debugRotationClamping) {
      System.out.println("output: " + rot);

      System.out.println("END CLAMPING DEBUG -----------------");
    }

    return rot;
  }

  private static double approachRotation(double targetRot, double currentRot, double percent) {
    targetRot = GeometryUtil.optimizeRotationInDegrees(targetRot);

    currentRot = GeometryUtil.optimizeRotationInDegrees(currentRot);

    boolean outsideUp = currentRot > targetRot;

    double outsideDist =
        (outsideUp
            ? (Math.abs(currentRot - 180) + Math.abs(targetRot + 180))
            : -(Math.abs(targetRot - 180) + Math.abs(currentRot + 180)));

    boolean inside = Math.abs(targetRot - currentRot) < Math.abs(outsideDist);

    double output;

    double dif = inside ? (targetRot - currentRot) : outsideDist;

    if (inside) {
      output = dif * percent + currentRot;
    } else {
      output = dif * percent + currentRot;
    }

    output = GeometryUtil.optimizeRotationInDegrees(output);

    return output;
  }

  private static Pair<Rotation2d, Rotation2d> generateStartAndEndRotations(
      Pose2d target,
      Pose2d endRotationReferencePose,
      Pose2d startPose,
      Translation2d vel,
      RobotConfig config) {

    if (debugGenerateStartAndEndRotations) {
      System.out.println("START DEBUG GENERATE START AND END ROTATIONS ---------------------");
    }

    double startBottomRotationBound = -180;
    double startTopRotationBound = 180;

    double endBottomRotationBound = -180;
    double endTopRotationBound = 180;

    boolean currentPoseNextToReef = isRobotNextToReef(startPose);
    boolean targetPoseNextToReef = isRobotNextToReef(target);

    if (debugGenerateStartAndEndRotations) {
      System.out.println("CurrentPoseNextToReef: " + currentPoseNextToReef);
      System.out.println("TargetPoseNextToReef: " + targetPoseNextToReef);
    }

    if (currentPoseNextToReef) {
      double headingAwayFromReef =
          GeometryUtil.getHeading(
                  Constants.FieldConstants.REEF_CENTER_POSE(), startPose.getTranslation())
              .getDegrees();
      startBottomRotationBound = headingAwayFromReef - headingToleranceNearElements.get();
      startTopRotationBound = headingAwayFromReef + headingToleranceNearElements.get();
    }

    if (targetPoseNextToReef) {
      double headingTowardReef =
          GeometryUtil.getHeading(
                  target.getTranslation(), Constants.FieldConstants.REEF_CENTER_POSE())
              .getDegrees();

      endBottomRotationBound = headingTowardReef - headingToleranceNearElements.get();
      endTopRotationBound = headingTowardReef + headingToleranceNearElements.get();
    }

    Rotation2d headingToTarget =
        GeometryUtil.getHeading(startPose.getTranslation(), target.getTranslation());

    Rotation2d endHeadingToTarget =
        GeometryUtil.getHeading(endRotationReferencePose.getTranslation(), target.getTranslation());

    double velHeading = GeometryUtil.getHeading(Translation2d.kZero, vel).getDegrees();

    double optimalStartAngle =
        approachRotation(
            velHeading,
            headingToTarget.getDegrees(),
            vel.getNorm() / config.getRobotMaxLinearVelocity());

    double clampedStartAngle =
        clampRotation(optimalStartAngle, startTopRotationBound, startBottomRotationBound);
    double clampedEndAngle =
        clampRotation(endHeadingToTarget.getDegrees(), endTopRotationBound, endBottomRotationBound);

    if (debugGenerateStartAndEndRotations) {
      System.out.println("END DEBUG GENERATE START AND END ROTATIONS ---------------------");
    }

    return new Pair<Rotation2d, Rotation2d>(
        Rotation2d.fromDegrees(clampedStartAngle), Rotation2d.fromDegrees(clampedEndAngle));
  }

  public static boolean logicalXOR(boolean x, boolean y) {
    return ((x || y) && !(x && y));
  }
}
