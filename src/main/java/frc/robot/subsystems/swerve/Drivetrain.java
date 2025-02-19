// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.team6328.PoseEstimator;
import frc.lib.team6328.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase {
  public static final String ROOT_TABLE = "Drivetrain";

  // Execution timing

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);
  private static final ExecutionTiming timing_wait = new ExecutionTiming(ROOT_TABLE + "/wait");
  private static final ExecutionTiming timing_process =
      new ExecutionTiming(ROOT_TABLE + "/odometry");
  private static final ExecutionTiming timing_vision = new ExecutionTiming(ROOT_TABLE + "/vision");
  private static final ExecutionTiming timing_pose = new ExecutionTiming(ROOT_TABLE + "/pose");

  private static final LoggerGroup logGroupDrive = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal logGyro_canivoreBusUtilization =
      logGroupDrive.buildDecimal("canivoreBusUtilization");

  // Logging

  // Gyro logging

  private static final LoggerGroup logGyro = logGroupDrive.subgroup("Gyro");

  private static final LoggerEntry.Text logGyro_selectedGyro = logGyro.buildString("selectedGyro");
  private static final LoggerEntry.Bool logGyro_usingSecondGyro =
      logGyro.buildBoolean("usingSecondGyro");
  private static final LoggerEntry.Decimal logGyro_yawPosition =
      logGyro.buildDecimal("YawPosition");
  private static final LoggerEntry.Decimal logGyro_yawVelocityRadPerSec =
      logGyro.buildDecimal("YawVelocityRadPerSec");
  private static final LoggerEntry.Decimal logGyro_xAcceleration =
      logGyro.buildDecimal("XAcceleration");
  private static final LoggerEntry.Decimal logGyro_yAcceleration =
      logGyro.buildDecimal("YAcceleration");
  private static final LoggerEntry.Decimal logGyro_zAcceleration =
      logGyro.buildDecimal("ZAcceleration");
  private static final LoggerEntry.Decimal logGyro_Acceleration =
      logGyro.buildDecimal("linearAcceleration");

  // Swerve logging

  private static final LoggerGroup logGroupSwerveStates = LoggerGroup.build("SwerveStates");
  private static final LoggerEntry.StructArray<SwerveModuleState> logSwerveStatesSetpoints =
      logGroupSwerveStates.buildStructArray(SwerveModuleState.class, "Setpoints");
  private static final LoggerEntry.StructArray<SwerveModuleState>
      logSwerveStatesSetpointsOptimized =
          logGroupSwerveStates.buildStructArray(SwerveModuleState.class, "SetpointsOptimized");
  private static final LoggerEntry.StructArray<SwerveModuleState> logSwerveStatesMeasured =
      logGroupSwerveStates.buildStructArray(SwerveModuleState.class, "Measured");

  // Odometry logging

  private static final LoggerGroup logGroupOdometryThread = LoggerGroup.build("OdometryThread");
  private static final LoggerEntry.EnumValue<StatusCode> logOdometryStatus =
      logGroupOdometryThread.buildEnum("status", SwerveModule.ODOMETRY_FREQUENCY);
  private static final LoggerEntry.Decimal logOdometryTimestampSpread =
      logGroupOdometryThread.buildDecimal("timestampSpread", SwerveModule.ODOMETRY_FREQUENCY);
  private static final LoggerEntry.Struct<Twist2d> logOdometryTwist =
      logGroupOdometryThread.buildStruct(Twist2d.class, "twist", SwerveModule.ODOMETRY_FREQUENCY);
  private static final LoggerEntry.Text logOdometryCrash =
      logGroupOdometryThread.buildString("crash");

  // Drivetrain logging

  private static final LoggerGroup logGroupDrivetrain = LoggerGroup.build("Drivetrain");
  private static final LoggerEntry.Decimal logDrivetrain_leftoverVelocity =
      logGroupDrivetrain.buildDecimal("leftoverVelocity");

  private static final LoggerEntry.Decimal logDrivetrain_speedsX =
      logGroupDrivetrain.buildDecimal("speedsX");
  private static final LoggerEntry.Decimal logDrivetrain_speedsY =
      logGroupDrivetrain.buildDecimal("speedsY");
  private static final LoggerEntry.Decimal logDrivetrain_linearSpeed =
      logGroupDrivetrain.buildDecimal("linearSpeed");
  private static final LoggerEntry.Decimal logDrivetrain_linearSpeedMax =
      logGroupDrivetrain.buildDecimal("linearSpeedMax");
  private static final LoggerEntry.Decimal logDrivetrain_speedsRot =
      logGroupDrivetrain.buildDecimal("speedsRot");

  // Robot position logging
  private static final LoggerGroup logGroupLocalization = LoggerGroup.build("Localization");

  private static final LoggerEntry.Struct<Pose2d> logLocalization_RobotPosition_RAW_ODOMETRY =
      logGroupLocalization.buildStruct(Pose2d.class, "RobotPosition_RAW_ODOMETRY");

  private static final LoggerEntry.Struct<Pose2d> logLocalization_ReefRobotPosition =
      logGroupLocalization.buildStruct(Pose2d.class, "ReefRobotPosition");
  private static final LoggerEntry.Struct<Pose2d> logLocalization_CoralStationRobotPosition =
      logGroupLocalization.buildStruct(Pose2d.class, "CoralStationRobotPosition");

  private static final LoggerEntry.Decimal logTimeSinceVision =
      logGroupLocalization.buildDecimal("timeSinceVision");
  private static final LoggerEntry.Integer logRejectionCutoff =
      logGroupLocalization.buildInteger("rejectionCutoff");
  private static final LoggerEntry.Integer logRejectionInvalidTags =
      logGroupLocalization.buildInteger("rejectionInvalidTags");
  private static final LoggerEntry.Integer logRejectionNoDriveData =
      logGroupLocalization.buildInteger("rejectionNoDriveData");

  private static final LoggerGroup logGroupRobot = LoggerGroup.build("Robot");
  private static final LoggerEntry.Struct<Pose2d> log_FieldRelativeVel =
      logGroupRobot.buildStruct(Pose2d.class, "FieldRelativeVel");
  private static final LoggerEntry.Decimal log_FieldRelativeAcceleration =
      logGroupRobot.buildDecimal("FieldRelativeAcceleration");

  public static final AutoLock odometryLock = new AutoLock("odometry", 100);

  // Tunable numbers

  public static final TunableNumberGroup group = new TunableNumberGroup("Drivetrain");
  public static final LoggedTunableNumber driveMotorAccelerationAuto =
      group.build("SwerveDRIVEAccelerationAuto", 1000);
  public static final LoggedTunableNumber driveMotorAccelerationTele =
      group.build("SwerveDRIVEAccelerationTele", 1000);

  private boolean switchGyro;
  private Boolean useSecondGyro;

  private final RobotConfig config;
  private final Supplier<Boolean> isAutonomous;
  private final boolean isCANFD;

  private final GyroIO gyroIO;
  private final GyroIO gyroIO2;
  private GyroIO selectedGyroIO;
  private final GyroIO.Inputs gyroInputs = new GyroIO.Inputs(logGyro);
  private final GyroIO.Inputs gyroInputs2 = new GyroIO.Inputs(logGyro);
  private GyroIO.Inputs selectedGyroInputs;
  private final SwerveModules modules;

  private final SwerveDriveKinematics kinematics;
  private Pose2d rawOdometryPose = Constants.zeroPose2d;

  private final PoseEstimator reefPoseEstimator;
  private final PoseEstimator coralStationPoseEstimator;

  private final Field2d reefField2d = new Field2d();
  private final Field2d coralField2d = new Field2d();
  private final Field2d rawOdometryField2d = new Field2d();

  private Pose2d prevVel = Constants.zeroPose2d;

  public Drivetrain(
      RobotConfig config,
      GyroIO gyroIO,
      GyroIO gyroIO2,
      SwerveModules swerveModules,
      Supplier<Boolean> isAutonomous) {
    this.config = config;
    this.modules = swerveModules;
    this.gyroIO = gyroIO;
    this.gyroIO2 = gyroIO2;
    this.isAutonomous = isAutonomous;

    String canBusName = config.getCANBusName();
    if (canBusName != null) {
      isCANFD = com.ctre.phoenix6.CANBus.isNetworkFD(canBusName);
    } else {
      isCANFD = false;
    }

    kinematics = config.getSwerveDriveKinematics();

    int[] reefTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    int[] coralStationTags = {1, 2, 12, 13};

    reefPoseEstimator = new PoseEstimator(0.6, 0.6, 0.3, reefTags); // TODO: refine these numbers?
    coralStationPoseEstimator =
        new PoseEstimator(0.6, 0.6, 0.3, coralStationTags); // TODO: refine these numbers?

    var thread = new Thread(this::runOdometry);
    thread.setName("PhoenixOdometryThread");
    thread.setDaemon(true);
    thread.start();
  }

  public void periodic() {
    try (var ignored = timing.start()) {
      GyroIO gyroIO = selectedGyroIO;
      GyroIO.Inputs inputs = selectedGyroInputs;
      if (gyroIO != null && inputs != null) {
        gyroIO.updateInputs(inputs);
        logGyro_yawPosition.info(inputs.yawPosition);
        logGyro_yawVelocityRadPerSec.info(inputs.yawVelocityRadPerSec);
        logGyro_xAcceleration.info(inputs.xAcceleration);
        logGyro_yAcceleration.info(inputs.yAcceleration);
        logGyro_zAcceleration.info(inputs.zAcceleration);
        logGyro_Acceleration.info(Math.hypot(inputs.xAcceleration, inputs.yAcceleration));
      }

      modules.updateInputs();
      modules.periodic();

      // Stop moving when disabled
      if (DriverStation.isDisabled()) {
        modules.stop();
      }
      // Log empty setpoint states when disabled
      if (DriverStation.isDisabled()) {
        logSwerveStatesSetpoints.info(new SwerveModuleState[] {});
        logSwerveStatesSetpointsOptimized.info(new SwerveModuleState[] {});
      }

      logSwerveStatesMeasured.info(getModuleStates());

      log_FieldRelativeVel.info(getFieldRelativeVelocities());
      log_FieldRelativeAcceleration.info(
          getFieldRelativeAccelerationMagnitude(prevVel.getTranslation().getNorm()));

      prevVel = getFieldRelativeVelocities();

      var reefPoseEstimatorPose = getReefPoseEstimatorPose();
      var coralPoseEstimatorPose = getReefPoseEstimatorPose();
      var visionStaleness = getReefVisionStaleness();

      logLocalization_ReefRobotPosition.info(reefPoseEstimatorPose);
      logLocalization_CoralStationRobotPosition.info(coralPoseEstimatorPose);

      reefField2d.setRobotPose(reefPoseEstimatorPose);
      coralField2d.setRobotPose(coralPoseEstimatorPose);
      SmartDashboard.putData("Localization/reefField2d", reefField2d);
      SmartDashboard.putData("Localization/coralField2d", coralField2d);

      rawOdometryField2d.setRobotPose(rawOdometryPose);
      logLocalization_RobotPosition_RAW_ODOMETRY.info(rawOdometryPose);
      SmartDashboard.putData("Localization/rawOdometryField2d", rawOdometryField2d);

      logTimeSinceVision.info(visionStaleness);
      logRejectionCutoff.info(reefPoseEstimator.rejectionCutoff);
      logRejectionInvalidTags.info(reefPoseEstimator.rejectionInvalidTags);
      logRejectionNoDriveData.info(reefPoseEstimator.rejectionNoDriveData);

      if (useSecondGyro != null) {
        logGyro_usingSecondGyro.info(useSecondGyro);
      }
      if (Robot.isReal()) {
        logGyro_canivoreBusUtilization.info(
            CANBus.getStatus(config.getCANBusName()).BusUtilization);
      }
    }
  }

  private void runOdometry() {
    while (true) {
      gyroIO.updateInputs(gyroInputs);
      gyroIO2.updateInputs(gyroInputs2);

      var gyro1Healthy = true;
      // gyroInputs.wasUpdatedRecently(0.1); //TODO: fix gyro healing
      var gyro2Healthy = gyroInputs2.wasUpdatedRecently(0.1);

      // Switch to the selected gyro, if healthy
      if (useSecondGyro != null && gyro1Healthy && gyro2Healthy) {
        if (useSecondGyro) {
          gyro1Healthy = false;
        } else {
          gyro2Healthy = false;
        }
      }

      if (gyro1Healthy) {
        selectedGyroIO = gyroIO;
        selectedGyroInputs = gyroInputs;
        logGyro_selectedGyro.info("Primary");
      } else if (gyro2Healthy) {
        selectedGyroIO = gyroIO2;
        selectedGyroInputs = gyroInputs2;
        logGyro_selectedGyro.info("Secondary");
      } else {
        selectedGyroIO = null;
        selectedGyroInputs = null;
        logGyro_selectedGyro.info("Disabled");
      }

      switchGyro = false;
      runOdometryInner();
    }
  }

  private void runOdometryInner() {
    List<BaseStatusSignal> signals = new ArrayList<>();

    if (selectedGyroIO != null) {
      selectedGyroIO.registerSignalForOdometry(signals);
    }

    modules.registerSignalForOdometry(signals);

    var signalsArray = signals.toArray(new BaseStatusSignal[0]);
    Rotation2d lastGyroRotation = null;
    StatusCode lastStatusCode = null;
    int loopWithoutAnyGyros = SwerveModule.ODOMETRY_FREQUENCY;

    while (!switchGyro) {
      if (selectedGyroInputs != null) {
        if (!selectedGyroInputs.wasUpdatedRecently(0.1))
          // Gyro not reporting, exit.
          break;
      } else {
        if (loopWithoutAnyGyros-- <= 0) {
          // Try checking for gyro again.
          break;
        }
      }

      try {
        double timestamp;

        // Wait for updates from all signals
        if (isCANFD) {
          StatusCode statusCode;

          try (var ignored = timing_wait.start()) {
            statusCode =
                BaseStatusSignal.waitForAll(2.0 / SwerveModule.ODOMETRY_FREQUENCY, signalsArray);
          }

          if (statusCode != lastStatusCode) {
            logOdometryStatus.info(statusCode);
            lastStatusCode = statusCode;
          }

          if (statusCode != StatusCode.OK) {
            continue;
          }

          double timeSum = 0.0;
          double timeMin = Double.MAX_VALUE;
          double timeMax = 0.0;

          for (BaseStatusSignal s : signals) {
            var time = s.getTimestamp().getTime();
            timeSum += time;
            timeMin = Math.min(timeMin, time);
            timeMax = Math.max(timeMax, time);
          }
          timestamp = timeSum / signalsArray.length;
          var timestampSpread =
              Math.max(Math.abs(timeMax - timestamp), Math.abs(timeMin - timestamp));

          logOdometryTimestampSpread.info(timestampSpread);
        } else {
          // "waitForAll" does not support blocking on multiple
          // signals with a bus that is not CAN FD, regardless
          // of Pro licensing. No reasoning for this behavior
          // is provided by the documentation.
          Thread.sleep((long) (1000.0 / SwerveModule.ODOMETRY_FREQUENCY));
          BaseStatusSignal.refreshAll(signalsArray);

          timestamp = Utils.getCurrentTimeSeconds();
        }

        var wheelDeltas = modules.updateOdometry();

        // The twist represents the motion of the robot since the last
        // sample in x, y, and theta based only on the modules, without
        // the gyro. The gyro is always disconnected in simulation.
        var twist = kinematics.toTwist2d(wheelDeltas);
        logOdometryTwist.info(twist);

        if (selectedGyroIO != null) {
          var gyroRotation = selectedGyroIO.updateOdometry(selectedGyroInputs);
          if (gyroRotation != null) {
            if (lastGyroRotation != null) {
              var dtheta = gyroRotation.minus(lastGyroRotation).getRadians();
              twist = new Twist2d(twist.dx, twist.dy, dtheta);
            }

            lastGyroRotation = gyroRotation;
          }
        }

        try (var ignored2 = odometryLock.lock()) // Prevents odometry updates while reading data
        {
          try (var ignored3 = timing_process.start()) {
            // Apply the twist (change since last sample) to the current pose
            rawOdometryPose = rawOdometryPose.exp(twist);

            reefPoseEstimator.addDriveData(timestamp, twist);
            coralStationPoseEstimator.addDriveData(timestamp, twist);
          }
        }
      } catch (Exception e) {
        logOdometryCrash.info(e.toString());
      }
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds, boolean prioritizeRotation) {
    // new Double().compareTo(null)
    if (prioritizeRotation
        && speeds.vxMetersPerSecond > 0.001
        && speeds.vyMetersPerSecond > 0.001) {
      // Calculate module setpoints

      SwerveModuleState[] justRotationSetpointStates =
          kinematics.toSwerveModuleStates(
              new ChassisSpeeds(0.0, 0.0, speeds.omegaRadiansPerSecond));

      SwerveModuleState[] calculatedSetpointStates = kinematics.toSwerveModuleStates(speeds);

      double realMaxSpeed = 0;
      for (SwerveModuleState moduleState : calculatedSetpointStates) {
        realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
      }
      if (realMaxSpeed > config.getRobotMaxLinearVelocity()) {
        /*

        linear cannot exceed the leftover amount after rotation
        Vmax - abs(Vr) = hypot(Vx, Vy)

        preserve xy ratio
        Vyprev      Vy
        -------- = ----
        Vxprev      Vx

        solve system of equations for Vx and Vy

        Vx = Vxprev / hypot(Vxprev, Vyprev) * abs(max(0.0, Vmax - abs(Vr)))

        Vy = Vyprev * Vx / Vxprev

        */

        double rotationSpeedMetersPerSecond = justRotationSetpointStates[0].speedMetersPerSecond;
        double vxMetersPerSecond =
            (speeds.vxMetersPerSecond
                    / Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
                * Math.abs(
                    Math.max(
                        0.0,
                        config.getRobotMaxLinearVelocity()
                            - Math.abs(rotationSpeedMetersPerSecond)));

        double vyMetersPerSecond =
            speeds.vyMetersPerSecond * vxMetersPerSecond / speeds.vxMetersPerSecond;

        speeds =
            new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, speeds.omegaRadiansPerSecond);

        logDrivetrain_leftoverVelocity.info(
            config.getRobotMaxLinearVelocity() - rotationSpeedMetersPerSecond);
      }
    }

    logDrivetrain_speedsX.info(speeds.vxMetersPerSecond);
    logDrivetrain_speedsY.info(speeds.vyMetersPerSecond);
    logDrivetrain_linearSpeed.info(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    logDrivetrain_linearSpeedMax.info(config.getRobotMaxLinearVelocity());
    logDrivetrain_speedsRot.info(speeds.omegaRadiansPerSecond);

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.kDefaultPeriod);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, config.getRobotMaxLinearVelocity());

    // Send setpoints to modules
    // The module returns the optimized state, useful for logging

    double driveMotorAcceleration;
    if (isAutonomous.get()) {
      driveMotorAcceleration = driveMotorAccelerationAuto.get();
    } else {
      driveMotorAcceleration = driveMotorAccelerationTele.get();
    }

    var optimizedSetpointStates = modules.runSetpoints(setpointStates, driveMotorAcceleration);

    // Log setpoint states
    logSwerveStatesSetpoints.info(setpointStates);
    logSwerveStatesSetpointsOptimized.info(optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds(), false);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      // each translation forms a triangle from the center of the robot with the appropriate angle
      // for X stance
      headings[i] = config.getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    modules.runCharacterizationVolts(volts);
  }

  /** Returns the average drive velocity in radians/sec. */
  public AngularVelocity getCharacterizationVelocity() {
    return modules.getCharacterizationVelocity();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  private SwerveModuleState[] getModuleStates() {
    return modules.getModuleStates();
  }

  public Angle[] getModuleDriveRotations() {
    return modules.getRawModuleDrivePosition();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getFieldRelativeVelocities() {
    Translation2d translation =
        new Translation2d(
                getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond)
            .rotateBy(getRawOdometryPose().getRotation());
    return new Pose2d(translation, new Rotation2d(getChassisSpeeds().omegaRadiansPerSecond));
  }

  public Pose2d getRobotCentricVelocities() {
    Translation2d translation =
        new Translation2d(
            getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
    return new Pose2d(translation, new Rotation2d(getChassisSpeeds().omegaRadiansPerSecond));
  }

  public double getFieldRelativeAccelerationMagnitude(double prevVelMagnitude) {
    Pose2d currentVel = getFieldRelativeVelocities();
    return (currentVel.getTranslation().getNorm() - prevVelMagnitude) / 0.02;
  }

  public void addVisionEstimate(List<TimestampedVisionUpdate> visionData) {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      try (var ignored2 = timing_vision.start()) {
        reefPoseEstimator.addVisionData(visionData);
        coralStationPoseEstimator.addVisionData(visionData);
      }
    }
  }

  /**
   * WARNING - THIS IS THE RAW *ODOMETRY* POSE, THIS DOES NOT ACCOUNT FOR VISION DATA & SHOULD
   * EXCLUSIVELY BE USED FOR LOGGING AND ANALYSIS OR NON ABSOLUTE POSITION USAGE
   */
  public Pose2d getRawOdometryPose() {
    return rawOdometryPose;
  }

  public Pose2d getReefPoseEstimatorPose() {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      try (var ignored2 = timing_pose.start()) {
        return reefPoseEstimator.getLatestPose();
      }
    }
  }

  public Pose2d getCoralStationPoseEstimatorPose() {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      try (var ignored2 = timing_pose.start()) {
        return coralStationPoseEstimator.getLatestPose();
      }
    }
  }

  public double getReefVisionStaleness() {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      try (var ignored2 = timing_pose.start()) {
        return reefPoseEstimator.getVisionStaleness();
      }
    }
  }

  public double getCoralStationVisionStaleness() {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      try (var ignored2 = timing_pose.start()) {
        return coralStationPoseEstimator.getVisionStaleness();
      }
    }
  }

  public Pose2d getReefPoseEstimatorPoseAtTimestamp(double timestamp) {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      return reefPoseEstimator.getPoseAtTime(timestamp);
    }
  }

  public Pose2d getCoralStationPoseEstimatorPoseAtTimestamp(double timestamp) {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      return coralStationPoseEstimator.getPoseAtTime(timestamp);
    }
  }

  public Rotation2d getRotationGyroOnly() {
    return rawOdometryPose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    try (var ignored = odometryLock.lock()) // Prevents odometry updates while reading data
    {
      this.reefPoseEstimator.resetPose(pose, Utils.getCurrentTimeSeconds() + 0.2);
      this.coralStationPoseEstimator.resetPose(pose, Utils.getCurrentTimeSeconds() + 0.2);

      this.rawOdometryPose = pose;
    }
  }

  public void setCoralStationPoseToReefPose() {
    this.coralStationPoseEstimator.resetPose(
        reefPoseEstimator.getLatestPose(), Utils.getCurrentTimeSeconds() + 0.2);
  }

  public void setRawOdometryPose(Pose2d pose) {
    this.rawOdometryPose = pose;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return config.getRobotMaxLinearVelocity();
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return config.getRobotMaxAngularVelocity();
  }

  public boolean isGyroConnected() {
    return selectedGyroInputs != null && selectedGyroInputs.connected;
  }

  public void chooseWhichGyro(boolean useSecondGyro) {
    this.useSecondGyro = useSecondGyro;
    this.switchGyro = true;
  }

  public void setBrakeMode(boolean brake) {
    modules.setBrakeMode(brake);
  }
}
