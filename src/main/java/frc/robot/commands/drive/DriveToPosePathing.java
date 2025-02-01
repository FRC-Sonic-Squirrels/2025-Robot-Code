// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TrajectoryUtil;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.helpers.ChoreoHelper.ChassisSpeedsWithPathEnd;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPosePathing extends Command {

  private final DrivetrainWrapper wrapper;
  private final RobotConfig config;
  private final Supplier<Pose2d> currentPose;
  private final Supplier<Pose2d> targetPose;

  private ChoreoHelper helper;
  private boolean pathFinished = false;

  private TunableNumberGroup tunableGroup = new TunableNumberGroup("DriveToPosePathing");
  private LoggedTunableNumber minVelToUseVelForHeading =
      tunableGroup.build("MinVelToUseVelForHeading", 1.0);
  private LoggedTunableNumber reefCirclingDistMeters =
      tunableGroup.build(
          "ReefCirclingDistMeters",
          Constants.FieldConstants.REEF_DIAGONAL_WIDTH
                  .plus(Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getMeasureY())
                  .div(2)
                  .in(Units.Meters)
              + 0.1);
  private LoggedTunableNumber solverIterations = tunableGroup.build("SolverIterations", 6.0);
  private LoggedTunableNumber solverAggresivenessFactor =
      tunableGroup.build("SolverAggresivenessFactor", 5.0);
  private LoggedTunableNumber headingToleranceNearElements =
      tunableGroup.build("HeadingToleranceNearElements", 80.0);

  private LoggerGroup logGroup = LoggerGroup.build("DriveToPosePathing");
  private LoggerEntry.Struct<Pose2d> log_intermediatePose =
      logGroup.buildStruct(Pose2d.class, "IntermediatePose");
  private LoggerEntry.Struct<Pose2d> log_intersectingPose =
      logGroup.buildStruct(Pose2d.class, "IntersectingPose");
  private LoggerEntry.Struct<Pose2d> log_targetChassisSpeeds =
      logGroup.buildStruct(Pose2d.class, "TargetChassisSpeeds");
  private LoggerEntry.DecimalArray log_iterationTimes =
      logGroup.buildDecimalArray("IterationTimes");

  /** Creates a new DriveToPosePathing. */
  public DriveToPosePathing(
      DrivetrainWrapper wrapper,
      RobotConfig config,
      Supplier<Pose2d> currentPose,
      Supplier<Pose2d> targetPose) {
    this.wrapper = wrapper;
    this.config = config;
    this.currentPose = currentPose;
    this.targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double startBottomRotationBound = -180;
    double startTopRotationBound = 180;

    double endBottomRotationBound = -180;
    double endTopRotationBound = 180;

    if (isRobotNextToReef(currentPose.get())) {
      double headingAwayFromReef =
          GeometryUtil.getHeading(
                  Constants.FieldConstants.REEF_CENTER_POSE(), currentPose.get().getTranslation())
              .getDegrees();
      startBottomRotationBound = headingAwayFromReef - headingToleranceNearElements.get();
      startTopRotationBound = headingAwayFromReef + headingToleranceNearElements.get();
    }

    if (isRobotNextToReef(targetPose.get())) {
      double headingAwayFromReef =
          GeometryUtil.getHeading(
                  Constants.FieldConstants.REEF_CENTER_POSE(), targetPose.get().getTranslation())
              .getDegrees();
      endBottomRotationBound = headingAwayFromReef - headingToleranceNearElements.get();
      endTopRotationBound = headingAwayFromReef + headingToleranceNearElements.get();
    }

    Pair<Rotation2d, Rotation2d> bestHeadings =
        optimizeHeading(
            startBottomRotationBound,
            startTopRotationBound,
            endBottomRotationBound,
            endTopRotationBound);

    Trajectory<SwerveSample> traj = generatePath(bestHeadings.getFirst(), bestHeadings.getSecond());

    helper =
        new ChoreoHelper(
            Timer.getFPGATimestamp(),
            traj.getInitialPose(false).orElseThrow(),
            new ChoreoTrajectoryWithName("DriveToPose", traj),
            config.getDriveBaseRadius() / 2,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController(),
            false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeedsWithPathEnd result =
        helper.calculateChassisSpeeds(currentPose.get(), Timer.getFPGATimestamp());
    log_targetChassisSpeeds.info(
        new Pose2d(
            result.chassisSpeeds().vxMetersPerSecond,
            result.chassisSpeeds().vyMetersPerSecond,
            Rotation2d.fromRadians(result.chassisSpeeds().omegaRadiansPerSecond)));
    wrapper.setVelocityOverride(result.chassisSpeeds());
    pathFinished = result.atEndOfPath();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrapper.resetVelocityOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathFinished;
  }

  private boolean trajIntersectsWithReef(Trajectory<SwerveSample> traj) {
    for (SwerveSample sample : traj.samples()) {
      Pose2d pose = sample.getPose();
      double distToStart = GeometryUtil.getDist(pose, traj.getInitialPose(false).get());
      double distToEnd = GeometryUtil.getDist(pose, traj.getFinalPose(false).get());
      if (poseIntersectsWithReef(pose) && distToStart > 0.1 && distToEnd > 0.1) {
        log_intersectingPose.info(pose);
        return true;
      }
    }
    log_intersectingPose.info(Pose2d.kZero);
    return false;
  }

  private boolean poseIntersectsWithReef(Pose2d pose) {
    if (!isRobotNextToReef(pose)) return false;

    for (Translation2d corner : getRobotCorners(pose)) {
      if (translationIntersectsWithReef(corner)) return true;
    }

    return false;
  }

  private boolean translationIntersectsWithReef(Translation2d translation) {
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

  private Translation2d[] getRobotCorners(Pose2d pose) {
    Translation2d[] corners = new Translation2d[4];
    Translation2d[] robotRelativeCorners = config.getModuleTranslations();
    for (int i = 0; i < corners.length; i++) {
      corners[i] =
          pose.getTranslation()
              .plus(
                  robotRelativeCorners[i].rotateBy(
                      pose.getRotation())); // TODO: change to corners of bumpers
    }
    return corners;
  }

  private List<Trajectory<SwerveSample>> getPaths(
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

  private Trajectory<SwerveSample> getSubpath(List<SwerveSample> samples, int start, int end) {
    List<SwerveSample> sublist = samples.subList(start, end);
    return new Trajectory<SwerveSample>("", sublist, null, null);
  }

  private int findIndexOfTime(Trajectory<SwerveSample> path, double time) {
    SwerveSample sample = path.sampleAt(time, false).get();
    List<SwerveSample> samples = path.samples();
    for (int i = 0; i < samples.size(); i++) if (samples.get(i).equals(sample)) return i;
    return -1;
  }

  private Trajectory<SwerveSample> rerouteTrajectory(
      Trajectory<SwerveSample> traj, Pose2d start, Pose2d end) {
    if (!trajIntersectsWithReef(traj)) {
      log_intermediatePose.info(Pose2d.kZero);
      return traj;
    }
    Translation2d reefCenter = Constants.FieldConstants.REEF_CENTER_POSE();
    Rotation2d heading1 = GeometryUtil.getHeading(reefCenter, start.getTranslation());
    Rotation2d heading2 = GeometryUtil.getHeading(reefCenter, end.getTranslation());
    Rotation2d halfWayAngle = midRotation(heading1, heading2);
    Translation2d midTranslation =
        reefCenter.plus(new Translation2d(reefCirclingDistMeters.get(), halfWayAngle));
    Pose2d midWaypoint =
        new Pose2d(
            midTranslation,
            GeometryUtil.getHeading(reefCenter, midTranslation)
                .plus(
                    heading1.getRadians() < halfWayAngle.getRadians()
                        ? Rotation2d.kCCW_90deg
                        : Rotation2d.kCW_90deg));
    log_intermediatePose.info(midWaypoint);
    return generateSimplePath(PathPlannerPath.waypointsFromPoses(start, midWaypoint, end));
  }

  private Rotation2d midRotation(Rotation2d a, Rotation2d b) {
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

  private Trajectory<SwerveSample> generatePath(
      Rotation2d startDirection, Rotation2d endDirection) {
    Pose2d initPose = currentPose.get();

    Translation2d vel = wrapper.getFieldRelativeVelocities().getTranslation();

    Pose2d startPoint = new Pose2d(initPose.getTranslation(), startDirection);

    //   GeometryUtil.getDist(
    //     initPose.getTranslation(), Constants.FieldConstants.REEF_CENTER_POSE())
    // < Constants.FieldConstants.REEF_DIAGONAL_WIDTH
    //     .plus(Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getMeasureY())
    //     .div(2)
    //     .plus(Units.Inches.of(2))
    //     .in(Units.Meter)

    Pose2d endPoint = new Pose2d(targetPose.get().getTranslation(), endDirection);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPoint, endPoint);

    Trajectory<SwerveSample> traj =
        rerouteTrajectory(generateSimplePath(waypoints), startPoint, endPoint);

    return traj;
  }

  private Trajectory<SwerveSample> generateSimplePath(List<Waypoint> waypoints) {
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
                0.0,
                targetPose
                    .get()
                    .getRotation()) // Goal end state. You can set a holonomic rotation here. If
            // using a
            // differential drivetrain, the rotation will have no effect.
            );

    path.preventFlipping = true;

    PathPlannerTrajectory traj =
        path.generateTrajectory(
            wrapper.getCurrentRobotRelativeChassisSpeeds(),
            wrapper.getRotationGyroOnly(),
            config.pathPlannerConfig());

    return TrajectoryUtil.pathPlannerToChoreo(traj);
  }

  private Pair<Rotation2d, Rotation2d> optimizeHeading(
      double startRotationBottomBoundDeg,
      double startRotationTopBoundDeg,
      double endRotationBottomBoundDeg,
      double
          endRotationTopBoundDeg) { // TODO: we could potentially add wrapping to this when bounds

    startRotationBottomBoundDeg =
        GeometryUtil.optimizeRotationInDegrees(startRotationBottomBoundDeg);
    startRotationTopBoundDeg = GeometryUtil.optimizeRotationInDegrees(startRotationTopBoundDeg);
    endRotationBottomBoundDeg = GeometryUtil.optimizeRotationInDegrees(endRotationBottomBoundDeg);
    endRotationTopBoundDeg = GeometryUtil.optimizeRotationInDegrees(endRotationTopBoundDeg);

    double bestStartRot =
        midRotation(
                Rotation2d.fromDegrees(startRotationBottomBoundDeg),
                Rotation2d.fromDegrees(startRotationTopBoundDeg))
            .getDegrees();
    double bestEndRot =
        midRotation(
                Rotation2d.fromDegrees(startRotationBottomBoundDeg),
                Rotation2d.fromDegrees(startRotationTopBoundDeg))
            .getDegrees();

    double prevGuessTime = generatePath(Rotation2d.kZero, Rotation2d.kZero).getTotalTime();

    double startTimeOffset = 0.5;
    double endTimeOffset = 0.5;

    boolean startRotInsideRange = startRotationBottomBoundDeg < bestStartRot;
    boolean endRotInsideRange = endRotationBottomBoundDeg < bestEndRot;

    List<Double> times = new ArrayList<>();

    for (int i = 0; i < solverIterations.get(); i++) {

      times.add(prevGuessTime);

      double startRotTime =
          generatePath(
                  Rotation2d.fromDegrees(
                      bestStartRot + startTimeOffset * solverAggresivenessFactor.get()),
                  Rotation2d.fromDegrees(bestEndRot))
              .getTotalTime();
      double endRotTime =
          generatePath(
                  Rotation2d.fromDegrees(
                      bestStartRot + endTimeOffset * solverAggresivenessFactor.get()),
                  Rotation2d.fromDegrees(bestEndRot))
              .getTotalTime();

      startTimeOffset = prevGuessTime - startRotTime;
      endTimeOffset = prevGuessTime - endRotTime;

      bestStartRot += startTimeOffset * solverAggresivenessFactor.get();

      bestEndRot += endTimeOffset * solverAggresivenessFactor.get();

      bestStartRot = GeometryUtil.optimizeRotationInDegrees(bestStartRot);

      bestEndRot = GeometryUtil.optimizeRotationInDegrees(bestEndRot);

      if (startRotInsideRange) {
        bestStartRot =
            MathUtil.clamp(bestStartRot, startRotationBottomBoundDeg, startRotationTopBoundDeg);
      } else {
        if (bestStartRot < startRotationBottomBoundDeg) bestStartRot = startRotationBottomBoundDeg;
        if (bestStartRot > startRotationTopBoundDeg) bestStartRot = startRotationTopBoundDeg;
      }

      if (endRotInsideRange) {
        bestEndRot = MathUtil.clamp(bestEndRot, endRotationBottomBoundDeg, endRotationTopBoundDeg);
      } else {
        if (bestEndRot < endRotationBottomBoundDeg) bestEndRot = endRotationBottomBoundDeg;
        if (bestEndRot > endRotationTopBoundDeg) bestEndRot = endRotationTopBoundDeg;
      }

      prevGuessTime =
          generatePath(Rotation2d.fromDegrees(bestStartRot), Rotation2d.fromDegrees(bestEndRot))
              .getTotalTime();
    }

    times.add(prevGuessTime);

    double[] doublePrimitives =
        Arrays.stream(times.toArray(new Double[times.size()]))
            .mapToDouble(Double::doubleValue)
            .toArray();

    log_iterationTimes.info(doublePrimitives);

    return new Pair<Rotation2d, Rotation2d>(
        Rotation2d.fromDegrees(bestStartRot), Rotation2d.fromDegrees(bestEndRot));
  }

  private double getRobotDistToReef(Pose2d pose) {
    return GeometryUtil.getDist(pose.getTranslation(), Constants.FieldConstants.REEF_CENTER_POSE());
  }

  private boolean isRobotNextToReef(Pose2d pose) {
    return getRobotDistToReef(pose)
        > Constants.FieldConstants.REEF_DIAGONAL_WIDTH.div(2.0).in(Units.Meters)
            + (Math.max(
                    Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getX(),
                    Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getY())
                / 2.0);
  }
}
