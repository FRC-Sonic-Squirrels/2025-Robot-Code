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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team6328.Alert;
import frc.lib.team6328.Alert.AlertType;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.commands.ScoreCoral.ScoringDirection;
import frc.robot.commands.ScoreCoral.ScoringSide;
import frc.robot.configs.RobotConfig;
import frc.robot.configs.RobotConfig2023Rober;
import frc.robot.configs.RobotConfig2024Maestro;
import frc.robot.configs.RobotConfig2025;
import frc.robot.configs.SimulatorRobotConfig;
import java.util.function.Supplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kDefaultPeriod = 0.02;
  public static final Translation2d zeroTranslation2d = new Translation2d();
  public static final Rotation2d zeroRotation2d = new Rotation2d();
  public static final Pose2d zeroPose2d = new Pose2d();
  public static final Pose3d zeroPose3d = new Pose3d();

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  public static class RobotMode {
    private static final RobotType ROBOT = RobotType.ROBOT_2025;

    private static final Alert invalidRobotAlert =
        new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

    public static boolean isSimBot() {
      switch (getRobot()) {
        case ROBOT_SIMBOT:
        case ROBOT_SIMBOT_REAL_CAMERAS:
          return true;

        default:
          return false;
      }
    }

    // FIXME: update for various robots
    public static Mode getMode() {
      if (isSimBot()) {
        return Mode.SIM;
      }

      return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    }

    public static RobotType getRobot() {
      if (!RobotBase.isReal()) {
        return RobotType.ROBOT_SIMBOT;
      }

      if (ROBOT != RobotType.ROBOT_2024_RETIRED_MAESTRO && ROBOT != RobotType.ROBOT_2025) {
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2025;
      }

      return ROBOT;
    }

    // FIXME: update for various robots
    public enum RobotType {
      // use supplier because if we just create the object, the fields in the
      // config classes are also created. Meaning tunableNumber values are stuck to
      // the first
      // object that is created. In this case ExampleRobotConfig. Suppliers solve this
      // by only creating the specific config object corresponding to the robot type
      ROBOT_SIMBOT(SimulatorRobotConfig::new),
      ROBOT_SIMBOT_REAL_CAMERAS(SimulatorRobotConfig::new),
      ROBOT_2023_RETIRED_ROBER(RobotConfig2023Rober::new),
      ROBOT_2024_RETIRED_MAESTRO(RobotConfig2024Maestro::new),
      ROBOT_2025(RobotConfig2025::new);

      public final Supplier<RobotConfig> config;

      RobotType(Supplier<RobotConfig> config) {
        this.config = config;
      }
    }

    public enum Mode {
      REAL,
      REPLAY,
      SIM
    }
  }

  public static double MAX_VOLTAGE = 12.0;

  public static class RobotDimensions {
    public static Distance BUMPER_THICKNESS = Units.Inches.of(3);

    /**
     * x = left to right
     *
     * <p>y = front to back
     */
    public static Translation2d ROBOT_DIMENSIONS_WITHOUT_BUMPERS() {
      if (RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
        return new Translation2d(Units.Inches.of(27.0), Units.Inches.of(32.5));
      } else {
        return new Translation2d(Units.Inches.of(28.0), Units.Inches.of(30.0));
      }
    }

    /**
     * x = left to right
     *
     * <p>y = front to back
     */
    public static Translation2d ROBOT_DIMENSIONS_WITH_BUMPERS =
        ROBOT_DIMENSIONS_WITHOUT_BUMPERS()
            .plus(new Translation2d(BUMPER_THICKNESS.times(2.0), BUMPER_THICKNESS.times(2.0)));
  }

  public static class FieldConstants {
    // official Field dimensions from game manual
    public static Distance FIELD_LENGTH = Units.Inches.of(690.875);
    public static Distance FIELD_WIDTH = Units.Inches.of(317.0);

    public static Distance REEF_DIST_FROM_WALL = Units.Inches.of(144.0);

    public static Distance REEF_WIDTH = Units.Inches.of(65.5);

    public static Distance REEF_DIAGONAL_WIDTH = REEF_WIDTH.times(2.0 / Math.sqrt(3));

    public static Translation2d BLUE_REEF_CENTER_POSE =
        new Translation2d(REEF_DIST_FROM_WALL.plus(REEF_WIDTH.div(2.0)), FIELD_WIDTH.div(2.0));

    public static Translation2d REEF_CENTER_POSE() {
      return AllianceFlipUtil.flipTranslationForAlliance(BLUE_REEF_CENTER_POSE);
    }

    public static Distance REEF_BRANCH_OFFSET = Units.Inches.of(6.5);

    private static ScoringSide[] SCORING_SIDE_ORDER = {
      ScoringSide.FAR_MID,
      ScoringSide.FAR_LEFT,
      ScoringSide.NEAR_LEFT,
      ScoringSide.NEAR_MID,
      ScoringSide.NEAR_RIGHT,
      ScoringSide.FAR_RIGHT
    };

    public static ScoringSideWithPose[] SCORING_SIDES() {
      ScoringSideWithPose[] poses = new ScoringSideWithPose[6];
      for (int i = 0; i < poses.length; i++) {
        Rotation2d angle = Rotation2d.fromRotations(i / 6.0);
        Translation2d offset =
            new Translation2d(
                REEF_WIDTH
                    .plus(RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getMeasureY())
                    .div(2.0)
                    .in(Units.Meters),
                angle);
        poses[i] =
            new ScoringSideWithPose(
                AllianceFlipUtil.flipPoseForAlliance(
                    new Pose2d(BLUE_REEF_CENTER_POSE.plus(offset), angle)),
                SCORING_SIDE_ORDER[i]);
      }
      return poses;
    }

    public record ScoringSideWithPose(Pose2d pose, ScoringSide side) {}

    public record ScoringSideWithPoseAndDirection(
        Pose2d pose, ScoringSide side, ScoringDirection direction) {}

    public static final Distance CORAL_STATION_WIDTH = Units.Inches.of(76);

    public static class Gamepieces {
      // TODO: add specific gamepiece dimensions for new season
      public static final Distance GAMEPIECE_HEIGHT =
          Units.Inches.of(0.0); // TODO: change this to new value
      public static final Distance GAMEPIECE_TOLERANCE = Units.Inches.of(20.0);
      public static final double GAMEPIECE_PERSISTENCE = 0.5;
    }
  }

  public static class MotorConstants {
    public static class KrakenConstants {
      public static final double MAX_RPM = 6000.0;
      public static final double NOMINAL_VOLTAGE_VOLTS = 12.0;
      public static final double STALL_TORQUE_NEWTON_METERS = 7.09;
      public static final double STALL_CURRENT_AMPS = 40.0;
      public static final double FREE_CURRENT_AMPS = 30.0;
      public static final double FREE_SPEED_RPM = 6000.0;
      public static final double SUPPLY_VOLTAGE_TIME = 0.02;
    }
  }

  public static class IntakeConstants { // TODO: check all constants for new season
    public static final double INTAKING_PERCENT_OUT = 1.0;

    public static final double GEARING = 1.0;
    public static final double MOI = 0.05;

    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final String ROOT_TABLE = "Intake";

    public static class PivotConstants { // TODO: check all constants
      public static final double SUPPLY_CURRENT_LIMIT = 0;

      public static final double GEAR_RATIO = (16.0 / 42.0) * (16.0 / 56.0);

      public static final double MOI = 1.0;

      public static final Rotation2d MAX_PIVOT_ANGLE = Rotation2d.fromDegrees(91);
      public static final Rotation2d MIN_PIVOT_ANGLE = Rotation2d.fromDegrees(-80);
      public static final Rotation2d HOME_POSITION = MAX_PIVOT_ANGLE;

      public static final Rotation2d INITIAL_PIVOT_ANGLE = Rotation2d.fromDegrees(-30);
      public static final Rotation2d PIVOT_STOWED_ANGLE = Rotation2d.fromDegrees(-30);

      public static final Distance PIVOT_LENGTH = Units.Inches.of(6.215);

      public static final String ROOT_TABLE = "IntakePivot";
    }
  }

  public static class EndEffectorConstants { // TODO: check all constants for new season
    public static final double INTAKING_PERCENT_OUT = 1.0;

    public static final double GEARING = 1.0;
    public static final double MOI = 0.05;

    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final String ROOT_TABLE = "EndEffector";
  }

  public static class ElevatorConstants { // TODO: check all constants for new season
    public static final double GEAR_RATIO =
        RobotMode.ROBOT == RobotType.ROBOT_2024_RETIRED_MAESTRO ? 23.05 : 10.667;
    public static final Distance PULLEY_DIAMETER = Units.Inches.of(2.256);
    public static final double CARRIAGE_MASS = 0.2; // arbitrary

    public static final double INCHES_TO_MOTOR_ROT =
        Constants.ElevatorConstants.GEAR_RATIO
            / (Math.PI * Constants.ElevatorConstants.PULLEY_DIAMETER.in(Units.Inches));

    public static final Distance MAX_HEIGHT =
        RobotMode.ROBOT == RobotType.ROBOT_2024_RETIRED_MAESTRO
            ? Units.Inches.of(26.2)
            : Units.Inches.of(55);

    public static final Distance SAFE_HEIGHT = Units.Inches.of(15.491);
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    public static final Distance HOME_POSITION = Units.Inches.of(7.35);
    public static final String ROOT_TABLE = "Elevator";

    public static final InterpolatingDoubleTreeMap SPEED_SCALAR_MAP =
        new InterpolatingDoubleTreeMap();

    static {
      SPEED_SCALAR_MAP.put(HOME_POSITION.in(Units.Inch), 1.0);
      SPEED_SCALAR_MAP.put(MAX_HEIGHT.in(Units.Inch), 0.8);
    }
  }

  public static class LEDConstants { // TODO: check all constants for new season
    public static final int PWM_PORT = 9;
    public static final int MAX_LED_LENGTH = 60;
  }

  public static class CanIDs { // TODO: check all constants for new season
    // READ ME: CAN ID's THAT ARE NOT VALID TO USE
    // 1, 11, 21, 31
    // 2, 12, 22, 32
    // 3, 13, 23, 33
    // 4, 14, 24, 34
    // all these CAN ID's are reserved for the Drivetrain

    // TODO: get actual can ids for new season
    public static final int INTAKE_CAN_ID = 34;

    public static final int SHOOTER_CAN_ID = 33;
    public static final int SHOOTER_PIVOT_CAN_ID = 32;

    public static final int ARM_CAN_ID = 17;

    public static final int ELEVATOR_CAN_ID = 37;

    public static final int END_EFFECTOR_CAN_ID = 30;
    public static final int END_EFFECTOR_TOF_CAN_ID = 39;
    public static final int SECOND_END_EFFECTOR_TOF_CAN_ID = 40;

    public static final int GYRO_2_CAN_ID = 41;

    public static final int CLIMBER_ARM_CAN_ID = 40;
    // TODO: make proper canID
    public static final int PIVOT_CAN_ID = 1000;
  }

  public static class DIOPorts {}

  public enum ControlMode {
    POSITION,
    VELOCITY,
    VOLTAGE
  }

  public static class ArmConstants { // TODO: check all constants for new season
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    public static final double GEAR_RATIO =
        RobotMode.ROBOT == RobotType.ROBOT_2024_RETIRED_MAESTRO
            ? (50.0 / 12.0) * (50.0 / 20.0) * (42.0 / 18.0)
            : 27.7778;

    public static final double MOI = 0.15;

    public static final Rotation2d MAX_ARM_ANGLE = Rotation2d.fromDegrees(165);
    public static final Rotation2d MIN_ARM_ANGLE = Rotation2d.fromDegrees(-90);
    public static final Rotation2d HOME_POSITION = MIN_ARM_ANGLE;

    public static final Rotation2d ARM_SAFE_ANGLE = Rotation2d.fromDegrees(-87);

    public static final Distance ARM_LENGTH =
        RobotMode.ROBOT == RobotType.ROBOT_2024_RETIRED_MAESTRO
            ? Units.Inches.of(14)
            : Units.Inches.of(22.080109);

    public static final String ROOT_TABLE = "Arm";
  }

  public static class ClimberConstants {
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    // TODO: update constants and dimensions for new robot design
    public static final double GEAR_RATIO = (50.0 / 12.0) * (50.0 / 20.0) * (42.0 / 18.0);

    public static final double MOI = 0.15;

    public static final Rotation2d MAX_CLIMBER_ANGLE = Rotation2d.fromDegrees(165);
    public static final Rotation2d MIN_CLIMBER_ANGLE = Rotation2d.fromDegrees(-90);
    public static final Rotation2d HOME_POSITION = MIN_CLIMBER_ANGLE;

    public static final Rotation2d CLIMBER_SAFE_ANGLE = Rotation2d.fromDegrees(-87);

    public static final Distance CLIMBER_LENGTH = Units.Inches.of(14);

    public static final String ROOT_TABLE = "Climber";
  }

  public static class VisionGamepieceConstants { // TODO: check all constants for new season
    public static final Pose3d GAMEPIECE_CAMERA_POSE =
        new Pose3d(
            Units.Inches.of(5.3).in(Units.Meters),
            0.0,
            Units.Inches.of(18.725).in(Units.Meters),
            new Rotation3d(0.0, Math.toRadians(-12), 0.0));
    public static final String CAMERA_NAME = RobotConfig2024Maestro.OBJECT_DETECTION_CAMERA_NAME;
    public static final int RESOLUTION_WIDTH_PIXELS = 960;
    public static final int RESOLUTION_HEIGHT_PIXELS = 720;
    public static final Rotation2d FOV_DIAGONAL = Rotation2d.fromDegrees(128.2);
    public static final double AVERAGE_ERROR_PIXELS = 0.35;
    public static final double AVERAGE_STANDARD_DEVIATION_PIXELS = 0.1;
    public static final double FPS = 15;
    public static final double AVERAGE_LATENCY_MS = 25;
    public static final double AVERAGE_STANDARD_DEVIATION_MS = 10;
  }

  public static class AutoConstants { // TODO: check all constants for new season
    public static final Distance DIST_TO_START_INTAKING = Units.Meters.of(1.0);
  }

  public static final boolean unusedCode = false;
}
