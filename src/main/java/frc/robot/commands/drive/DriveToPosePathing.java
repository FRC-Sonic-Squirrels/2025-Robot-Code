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
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.helpers.ChoreoHelper.ChassisSpeedsWithPathEnd;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.ArrayList;
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

  private static TunableNumberGroup tunableGroup = new TunableNumberGroup("DriveToPosePathing");
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
  private static LoggedTunableNumber stallMaxVelocity =
      tunableGroup.build("Stall/MaxVelocity", 0.4);
  private static LoggedTunableNumber stallMinDistance =
      tunableGroup.build("Stall/MinDistance", 0.05);
  private static LoggedTunableNumber stallDebounce = tunableGroup.build("Stall/Debounce", 0.15);

  private static LoggerGroup logGroup = LoggerGroup.build("DriveToPosePathing");
  private static LoggerGroup pathGenLogGroup = logGroup.subgroup("PathGeneration");
  private static LoggerGroup pathFollowingLogGroup = logGroup.subgroup("PathFollowing");
  private static LoggerEntry.Struct<Pose2d> log_intermediatePose =
      pathGenLogGroup.buildStruct(Pose2d.class, "IntermediatePose");
  private static LoggerEntry.Struct<Pose2d> log_intersectingPose =
      pathGenLogGroup.buildStruct(Pose2d.class, "IntersectingPose");
  private static LoggerEntry.StructArray<Pose2d> log_unAttemptedPath =
      pathGenLogGroup.buildStructArray(Pose2d.class, "UnAttemptedPath");
  private static LoggerEntry.Struct<Pose2d> log_targetChassisSpeeds =
      pathFollowingLogGroup.buildStruct(Pose2d.class, "TargetChassisSpeeds");
  private static LoggerEntry.Decimal log_distToTarget =
      pathFollowingLogGroup.buildDecimal("DistToTarget");
  private static LoggerEntry.Decimal log_velToTarget =
      pathFollowingLogGroup.buildDecimal("VelToTarget");
  private static LoggerEntry.Bool log_pathStalling = pathFollowingLogGroup.buildBoolean("Stalling");

  private boolean debugRotationClamping = false;
  private boolean debugRerouting = false;
  private boolean debugGenerateStartAndEndRotations = false;
  private boolean debugIsRobotNextToReef = false;

  private double finalOffsetErrorLimit = Double.NaN;
  private double finalTargetErrorLimit = Double.NaN;
  private double finalHeadingErrorLimit = Double.NaN;
  private double finalErrorMaxWait = 2;

  private final ScoringLevel level;

  // private double distToTarget;
  // private Double prevdistToTarget = Double.NaN;
  // private LinearFilter velocityToTarget = LinearFilter.movingAverage(8);
  // private double prevTime;
  // private Trigger stalled =
  //     new Trigger(
  //             () ->
  //                 (velocityToTarget.lastValue() < stallMaxVelocity.get()
  //                     && distToTarget > stallMinDistance.get()))
  //         .debounce(stallDebounce.get());

  /** Creates a new DriveToPosePathing. */
  public DriveToPosePathing(
      DrivetrainWrapper wrapper,
      RobotConfig config,
      Supplier<Pose2d> currentPose,
      Supplier<Pose2d> targetPose,
      ScoringLevel level) {
    this.wrapper = wrapper;
    this.config = config;
    this.currentPose = currentPose;
    this.targetPose = targetPose;
    this.level = level;
  }

  public DriveToPosePathing setFinalOffsetError(double maxError) {
    this.finalOffsetErrorLimit = maxError;
    return this;
  }

  public DriveToPosePathing setFinalTargetError(double maxError) {
    this.finalTargetErrorLimit = maxError;
    return this;
  }

  public DriveToPosePathing setFinalHeadingError(double maxError) {
    this.finalHeadingErrorLimit = maxError;
    return this;
  }

  public DriveToPosePathing setFinalErrorMaxWait(double maxWait) {
    this.finalErrorMaxWait = maxWait;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPose = wrapper.getCoralStationPoseEstimatorPose(true);
    Pose2d targetPose = this.targetPose.get();
    if (GeometryUtil.getDist(robotPose, targetPose) < 0.0002) {
      this.cancel();
      return;
    }

    initializeNoCheck();
  }

  public void initializeNoCheck() {
    Pose2d targetPose = this.targetPose.get();
    var rotations = generateStartAndEndRotations(targetPose, currentPose.get());

    var traj = generatePath(rotations.getFirst(), rotations.getSecond());
    if (traj == null) {
      this.cancel();
      return;
    }

    helper =
        new ChoreoHelper(
            Timer.getFPGATimestamp(),
            traj.getInitialPose(false).orElseThrow(),
            new ChoreoTrajectoryWithName("DriveToPose", traj),
            config.getDriveBaseRadius() / 2,
            config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(),
            config.getAutoThetaPidController());

    helper.setFinalErrorMaxWait(finalErrorMaxWait);
    helper.setFinalOffsetError(finalOffsetErrorLimit);
    helper.setFinalTargetError(finalTargetErrorLimit);
    helper.setFinalHeadingError(finalHeadingErrorLimit);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (helper == null) {
      pathFinished = true;
      return;
    }

    // distToTarget = GeometryUtil.getDist(currentPose.get(), targetPose.get());
    // log_distToTarget.info(distToTarget);
    // double time = RobotController.getFPGATime() / 1000000.0;

    // System.out.println("TIME DIF: " + (time - prevTime));
    // if (!prevdistToTarget.isNaN())
    //   velocityToTarget.calculate(Math.abs((distToTarget - prevdistToTarget) / (time -
    // prevTime)));

    // log_velToTarget.info(velocityToTarget.lastValue());
    log_pathStalling.info(pathStalling());

    ChassisSpeedsWithPathEnd result =
        helper.calculateChassisSpeeds(currentPose.get(), Timer.getFPGATimestamp());
    log_targetChassisSpeeds.info(
        new Pose2d(
            result.chassisSpeeds().vxMetersPerSecond,
            result.chassisSpeeds().vyMetersPerSecond,
            Rotation2d.fromRadians(result.chassisSpeeds().omegaRadiansPerSecond)));
    wrapper.setVelocityOverride(result.chassisSpeeds());
    pathFinished = result.atEndOfPath();

    // prevdistToTarget = distToTarget;
    // prevTime = time;
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

  public boolean pathStalling() {
    return false;
    // stalled.getAsBoolean();
  }

  private boolean trajIntersectsWithReef(Trajectory<SwerveSample> traj) {
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
        generateStartAndEndRotations(midWaypoint, midWaypoint);

    start = new Pose2d(start.getTranslation(), newRotations.getFirst());
    end = new Pose2d(end.getTranslation(), newRotations.getSecond());

    if (debugRerouting) {
      System.out.println("END REROUTING DEBUG -------------------");
    }

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

    Pose2d endPoint = new Pose2d(targetPose.get().getTranslation(), endDirection);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPoint, endPoint);

    Trajectory<SwerveSample> traj = generateSimplePath(waypoints);
    if (traj != null) {
      traj = rerouteTrajectory(traj, startPoint, endPoint);
    }

    return traj;
  }

  private Trajectory<SwerveSample> generateSimplePath(List<Waypoint> waypoints) {
    PathConstraints constraints =
        new PathConstraints(
            config.getPathingMaxSpeedMPS().get(),
            level == ScoringLevel.L4 ? config.getPathingMaxAccelerationMPSPS().get() : 2.0,
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
                targetPose
                    .get()
                    .getRotation()) // Goal end state. You can set a holonomic rotation here. If
            // using a
            // differential drivetrain, the rotation will have no effect.
            );

    path.preventFlipping = true;

    if (path.numPoints() < 2) {
      return null;
    }

    PathPlannerTrajectory traj =
        path.generateTrajectory(
            wrapper.getCurrentRobotRelativeChassisSpeeds(),
            wrapper.getRotationGyroOnly(),
            config.pathPlannerConfig());

    return TrajectoryUtil.pathPlannerToChoreo(traj);
  }

  private double getRobotDistToReef(Pose2d pose) {
    return GeometryUtil.getDist(pose.getTranslation(), Constants.FieldConstants.REEF_CENTER_POSE());
  }

  private boolean isRobotNextToReef(Pose2d pose) {
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

  private double clampRotation(double rot, double topBound, double bottomBound) {
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

  private double approachRotation(double targetRot, double currentRot, double percent) {
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

  private Pair<Rotation2d, Rotation2d> generateStartAndEndRotations(
      Pose2d target, Pose2d endRotationReferencePose) {

    if (debugGenerateStartAndEndRotations) {
      System.out.println("START DEBUG GENERATE START AND END ROTATIONS ---------------------");
    }

    double startBottomRotationBound = -180;
    double startTopRotationBound = 180;

    double endBottomRotationBound = -180;
    double endTopRotationBound = 180;

    boolean currentPoseNextToReef = isRobotNextToReef(currentPose.get());
    boolean targetPoseNextToReef = isRobotNextToReef(targetPose.get());

    if (debugGenerateStartAndEndRotations) {
      System.out.println("CurrentPoseNextToReef: " + currentPoseNextToReef);
      System.out.println("TargetPoseNextToReef: " + targetPoseNextToReef);
    }

    if (currentPoseNextToReef) {
      double headingAwayFromReef =
          GeometryUtil.getHeading(
                  Constants.FieldConstants.REEF_CENTER_POSE(), currentPose.get().getTranslation())
              .getDegrees();
      startBottomRotationBound = headingAwayFromReef - headingToleranceNearElements.get();
      startTopRotationBound = headingAwayFromReef + headingToleranceNearElements.get();
    }

    if (targetPoseNextToReef) {
      double headingTowardReef =
          GeometryUtil.getHeading(
                  targetPose.get().getTranslation(), Constants.FieldConstants.REEF_CENTER_POSE())
              .getDegrees();

      endBottomRotationBound = headingTowardReef - headingToleranceNearElements.get();
      endTopRotationBound = headingTowardReef + headingToleranceNearElements.get();
    }

    Rotation2d headingToTarget =
        GeometryUtil.getHeading(currentPose.get().getTranslation(), target.getTranslation());

    Rotation2d endHeadingToTarget =
        GeometryUtil.getHeading(
            endRotationReferencePose.getTranslation(), targetPose.get().getTranslation());

    double velHeading =
        GeometryUtil.getHeading(
                Translation2d.kZero, wrapper.getFieldRelativeVelocities().getTranslation())
            .getDegrees();

    double optimalStartAngle =
        approachRotation(
            velHeading,
            headingToTarget.getDegrees(),
            wrapper.getFieldRelativeVelocities().getTranslation().getNorm()
                / config.getRobotMaxLinearVelocity());

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
