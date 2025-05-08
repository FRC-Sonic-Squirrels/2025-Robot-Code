package frc.robot.configs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.ModuleConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.constants.SwerveModuleConstants;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.swerve.SwerveModules;
import frc.robot.subsystems.vision.VisionModuleConfiguration;

public class RobotConfig2025 extends RobotConfig {

  private static final boolean PHOENIX_PRO_LICENSE = true;

  // ------------ SWERVE ---------------------
  private static final Distance WHEEL_RADIUS = Units.Inches.of(1.957);

  // ------ SWERVE MODULE CONFIGURATIONS: CANID + OFFSET + INVERTS --------------
  // 0
  private static final IndividualSwerveModuleConfig FRONT_LEFT_MODULE_CONFIG =
      new IndividualSwerveModuleConfig(
          1,
          11,
          21,
          Rotation2d.fromDegrees(67.939),
          InvertedValue.CounterClockwise_Positive,
          InvertedValue.Clockwise_Positive);
  // 1
  private static final IndividualSwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG =
      new IndividualSwerveModuleConfig(
          2,
          12,
          22,
          Rotation2d.fromDegrees(-56.514),
          InvertedValue.CounterClockwise_Positive,
          InvertedValue.Clockwise_Positive);
  // 2
  private static final IndividualSwerveModuleConfig BACK_LEFT_MODULE_CONFIG =
      new IndividualSwerveModuleConfig(
          3,
          13,
          23,
          Rotation2d.fromDegrees(130.43),
          InvertedValue.CounterClockwise_Positive,
          InvertedValue.Clockwise_Positive);
  // 3
  private static final IndividualSwerveModuleConfig BACK_RIGHT_MODULE_CONFIG =
      new IndividualSwerveModuleConfig(
          4,
          14,
          24,
          Rotation2d.fromDegrees(-84.199),
          InvertedValue.CounterClockwise_Positive,
          InvertedValue.Clockwise_Positive);

  // -------- SWERVE CURRENT LIMITS ---------
  private static final CurrentLimitsConfigs DRIVE_TALON_CURRENT_LIMIT_CONFIGS =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(30)
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimit(100)
          .withStatorCurrentLimitEnable(true);

  private static final CurrentLimitsConfigs STEER_TALON_CURRENT_LIMIT_CONFIGS =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(25)
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimit(100)
          .withStatorCurrentLimitEnable(true);

  // --------- SWERVE GEAR RATIO ---------
  public static final double SWERVE_DRIVE_GEAR_RATIO =
      SwerveModuleConstants.MK4n.LEVEL_2_GEARING_DRIVE_GEAR_RATIO_PLUS_SPEED_KIT;
  public static final double SWERVE_STEER_GEAR_RATIO =
      SwerveModuleConstants.MK4n.GEARING_TURN_GEAR_RATIO;

  // ---------- SWERVE STEERING MOTOR PID CONSTANTS -----------
  // FIXE: RN copied from Mechanical advantage (6328) 2023 codebase. Should learn to tune
  // ourselves
  private static final LoggedTunableNumber ANGLE_KP = group.build("ANGLE_KP", 50.0);
  private static final LoggedTunableNumber ANGLE_KD = group.build("ANGLE_KD", 2.0);

  // ---------- SWERVE DRIVE MOTOR PID + KS + KV + KA CONSTANTS -------------
  private static final LoggedTunableNumber DRIVE_KP = group.build("DRIVE_KP", 2.5);
  private static final LoggedTunableNumber DRIVE_KD = group.build("DRIVE_KD", 0.0);

  private static final LoggedTunableNumber DRIVE_KS = group.build("DRIVE_KS", 0.0);
  private static final LoggedTunableNumber DRIVE_KV = group.build("DRIVE_KV", 0.8);
  private static final LoggedTunableNumber DRIVE_KA = group.build("DRIVE_KA", 0.0);

  // -------- GYRO CAN ID ---------
  private static final int GYRO_CAN_ID = 5;

  // -------- GYRO OFFSETS --------

  private static final double GYRO_MOUNTING_PITCH = 0.0;

  private static final double GYRO_MOUNTING_ROLL = 0.0;

  private static final double GYRO_MOUNTING_YAW = 0.0;

  // -------- CAN BUS NAME -----------
  private static final String CAN_BUS_NAME = "CANivore";

  // -------- ROBOT DIMENSIONS -----------
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
  // front to back
  private static final Distance TRACK_WIDTH_X = Units.Inches.of(24.75);
  // left to right
  private static final Distance TRACK_WIDTH_Y = Units.Inches.of(22.750005);

  // ------- ROBOT MAX SPEED --------
  private static final double MAX_VELOCITY_METERS_PER_SECOND = 4.78;
  private static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05; // unused currently

  // ------- AUTONOMOUS CONSTANTS -------
  private static final LoggedTunableNumber AUTO_MAX_SPEED_METERS_PER_SECOND =
      group.build("AUTO_MAX_SPEED", 5.0);
  private static final LoggedTunableNumber AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED =
      group.build("AUTO_MAX_ACCEL", 1.0);
  private static final LoggedTunableNumber AUTO_MAX_ANGULAR_VEL_RADIANS_PER_SECOND =
      group.build("AUTO_MAX_ANGULAR_VEL_RAD_PER_SECOND", Math.PI * 4);
  private static final LoggedTunableNumber
      AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED =
          group.build("AUTO_MAX_ANGULAR_ACCEL_RAD_PER_SECOND_SQUARED", Math.PI * 8);

  private static final LoggedTunableNumber AUTO_TRANSLATION_KP =
      group.build("AUTO_TRANSLATION_KP", 5.0);
  private static final LoggedTunableNumber AUTO_TRANSLATION_KI =
      group.build("AUTO_TRANSLATION_KI", 0.0);
  private static final LoggedTunableNumber AUTO_TRANSLATION_KD =
      group.build("AUTO_TRANSLATION_KD", 0.0);
  private static final LoggedTunableNumber AUTO_THETA_KP = group.build("AUTO_THETA_KP", 2.4);
  private static final LoggedTunableNumber AUTO_THETA_KI = group.build("AUTO_THETA_KI", 0.0);
  private static final LoggedTunableNumber AUTO_THETA_KD = group.build("AUTO_THETA_KD", 0.0);

  private final com.pathplanner.lib.config.RobotConfig PATH_PLANNER_CONFIG =
      new com.pathplanner.lib.config.RobotConfig(
          Units.Kilogram.of(145),
          Units.KilogramSquareMeters.of(4.422493401),
          new ModuleConfig(
              WHEEL_RADIUS,
              Units.MetersPerSecond.of(MAX_VELOCITY_METERS_PER_SECOND),
              0.65,
              DCMotor.getKrakenX60Foc(1),
              SwerveModuleConstants.MK4I.LEVEL_2_GEARING_DRIVE_GEAR_RATIO,
              Units.Amps.of(DRIVE_TALON_CURRENT_LIMIT_CONFIGS.SupplyCurrentLimit),
              1),
          getModuleTranslations());

  // ---- VISION  -------
  public static final Transform3d REEF_SIDE_LEFT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-8.763).in(Units.Meters),
              Units.Inches.of(8.472).in(Units.Meters),
              Units.Inches.of(12.125).in(Units.Meters)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(208.7)));

  public static final Transform3d REEF_SIDE_RIGHT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-8.763).in(Units.Meters),
              Units.Inches.of(-8.472).in(Units.Meters),
              Units.Inches.of(12.125).in(Units.Meters)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(151.85)));

  public static final Transform3d CORAL_STATION_SIDE_LEFT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-4.563).in(Units.Meters),
              Units.Inches.of(9.063).in(Units.Meters),
              Units.Inches.of(39.750).in(Units.Meters)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(-20.0), Math.toRadians(25.0)));

  public static final Transform3d CORAL_STATION_SIDE_RIGHT =
      new Transform3d(
          new Translation3d(
              Units.Inches.of(-4.558).in(Units.Meters),
              Units.Inches.of(-9.063).in(Units.Meters),
              Units.Inches.of(39.750).in(Units.Meters)),
          new Rotation3d(Math.toRadians(0.0), Math.toRadians(-20.0), Math.toRadians(-25.0)));

  public static final String OBJECT_DETECTION_CAMERA_NAME = "0_Object_Detection_ELP";
  public static final String REEF_SIDE_LEFT_CAMERA_NAME = "1_Reef_Left";
  public static final String REEF_SIDE_RIGHT_CAMERA_NAME = "2_Reef_Right";
  public static final String CORAL_STATION_SIDE_LEFT_CAMERA_NAME = "1_Coral_Left";
  public static final String CORAL_STATION_SIDE_RIGHT_CAMERA_NAME = "2_Coral_Right";

  public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2025ReefscapeWelded;

  /*
   *
   *
   * FUNCTIONS RETURNING CONSTANTS:
   *
   * YOU MUST CHANGED 2 METHODS:
   * getSwerveModuleObjects()
   * getVisionModuleObjects()
   *
   */

  @Override
  public SwerveModules getSwerveModuleObjects() {
    return new SwerveModules(
        new SwerveModule(0, this, new SwerveModuleIOTalonFX(this, FRONT_LEFT_MODULE_CONFIG)),
        new SwerveModule(1, this, new SwerveModuleIOTalonFX(this, FRONT_RIGHT_MODULE_CONFIG)),
        new SwerveModule(2, this, new SwerveModuleIOTalonFX(this, BACK_LEFT_MODULE_CONFIG)),
        new SwerveModule(3, this, new SwerveModuleIOTalonFX(this, BACK_RIGHT_MODULE_CONFIG)));
  }

  @Override
  public SwerveModules getReplaySwerveModuleObjects() {
    return new SwerveModules(
        new SwerveModule(0, this, new SwerveModuleIO.Fake()),
        new SwerveModule(1, this, new SwerveModuleIO.Fake()),
        new SwerveModule(2, this, new SwerveModuleIO.Fake()),
        new SwerveModule(3, this, new SwerveModuleIO.Fake()));
  }

  @Override
  public VisionModuleConfiguration[] getVisionModuleObjects() {
    return new VisionModuleConfiguration[] {
      VisionModuleConfiguration.build(REEF_SIDE_LEFT_CAMERA_NAME, REEF_SIDE_LEFT),
      VisionModuleConfiguration.build(REEF_SIDE_RIGHT_CAMERA_NAME, REEF_SIDE_RIGHT),
      VisionModuleConfiguration.build(CORAL_STATION_SIDE_LEFT_CAMERA_NAME, CORAL_STATION_SIDE_LEFT),
      VisionModuleConfiguration.build(
          CORAL_STATION_SIDE_RIGHT_CAMERA_NAME, CORAL_STATION_SIDE_RIGHT)
    };
  }

  @Override
  public VisionModuleConfiguration[] getReplayVisionModules() {
    return new VisionModuleConfiguration[] {
      VisionModuleConfiguration.buildReplayStub(REEF_SIDE_LEFT_CAMERA_NAME, REEF_SIDE_LEFT),
      VisionModuleConfiguration.buildReplayStub(REEF_SIDE_RIGHT_CAMERA_NAME, REEF_SIDE_RIGHT),
      VisionModuleConfiguration.buildReplayStub(
          CORAL_STATION_SIDE_LEFT_CAMERA_NAME, CORAL_STATION_SIDE_LEFT),
      VisionModuleConfiguration.buildReplayStub(
          CORAL_STATION_SIDE_RIGHT_CAMERA_NAME, CORAL_STATION_SIDE_RIGHT)
    };
  }

  @Override
  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return AprilTagFieldLayout.loadField(APRIL_TAG_FIELD);
  }

  @Override
  public boolean getPhoenix6Licensed() {
    return PHOENIX_PRO_LICENSE;
  }

  @Override
  public IndividualSwerveModuleConfig[] getIndividualModuleConfigurations() {
    return new IndividualSwerveModuleConfig[] {
      FRONT_LEFT_MODULE_CONFIG,
      FRONT_RIGHT_MODULE_CONFIG,
      BACK_LEFT_MODULE_CONFIG,
      BACK_RIGHT_MODULE_CONFIG
    };
  }

  @Override
  public int getGyroCANID() {
    return GYRO_CAN_ID;
  }

  @Override
  public Distance getTrackWidth_Y() {
    return TRACK_WIDTH_Y;
  }

  @Override
  public Distance getTrackWidth_X() {
    return TRACK_WIDTH_X;
  }

  @Override
  public double getRobotMaxLinearVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getRobotMaxCoastVelocity() {
    return MAX_COAST_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public String getCANBusName() {
    return CAN_BUS_NAME;
  }

  @Override
  public LoggedTunableNumber getDriveKS() {
    return DRIVE_KS;
  }

  @Override
  public LoggedTunableNumber getDriveKV() {
    return DRIVE_KV;
  }

  @Override
  public LoggedTunableNumber getDriveKA() {
    return DRIVE_KA;
  }

  @Override
  public LoggedTunableNumber getDriveKP() {
    return DRIVE_KP;
  }

  @Override
  public LoggedTunableNumber getDriveKD() {
    return DRIVE_KD;
  }

  @Override
  public LoggedTunableNumber getAngleKP() {
    return ANGLE_KP;
  }

  @Override
  public LoggedTunableNumber getAngleKD() {
    return ANGLE_KD;
  }

  @Override
  public LoggedTunableNumber getPathingMaxSpeedMPS() {
    return AUTO_MAX_SPEED_METERS_PER_SECOND;
  }

  @Override
  public LoggedTunableNumber getPathingMaxAccelerationMPSPS() {
    return AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
  }

  @Override
  public LoggedTunableNumber getPathingMaxAngularVelocityRadPerSecond() {
    return AUTO_MAX_ANGULAR_VEL_RADIANS_PER_SECOND;
  }

  @Override
  public LoggedTunableNumber getPathingMaxAngularAccelerationRadPerSecondSquared() {
    return AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
  }

  @Override
  public PIDController getAutoTranslationPidController() {
    return new PIDController(
        AUTO_TRANSLATION_KP.get(), AUTO_TRANSLATION_KI.get(), AUTO_TRANSLATION_KD.get());
  }

  @Override
  public PIDController getAutoThetaPidController() {
    return new PIDController(AUTO_THETA_KP.get(), AUTO_THETA_KI.get(), AUTO_THETA_KD.get());
  }

  @Override
  public Distance getWheelRadius() {
    return WHEEL_RADIUS;
  }

  @Override
  public double getSwerveModuleDriveGearRatio() {
    return SWERVE_DRIVE_GEAR_RATIO;
  }

  @Override
  public double getSwerveModuleTurnGearRatio() {
    return SWERVE_STEER_GEAR_RATIO;
  }

  @Override
  public CurrentLimitsConfigs getDriveTalonCurrentLimitConfig() {
    return DRIVE_TALON_CURRENT_LIMIT_CONFIGS;
  }

  @Override
  public CurrentLimitsConfigs getSteerTalonCurrentLimitConfig() {
    return STEER_TALON_CURRENT_LIMIT_CONFIGS;
  }

  @Override
  public double getGyroMountingPitch() {
    return GYRO_MOUNTING_PITCH;
  }

  @Override
  public double getGyroMountingRoll() {
    return GYRO_MOUNTING_ROLL;
  }

  @Override
  public double getGyroMountingYaw() {
    return GYRO_MOUNTING_YAW;
  }

  @Override
  public com.pathplanner.lib.config.RobotConfig pathPlannerConfig() {
    return PATH_PLANNER_CONFIG;
  }
}
