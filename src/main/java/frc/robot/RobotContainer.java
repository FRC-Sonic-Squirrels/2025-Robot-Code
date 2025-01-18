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

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.RunStateMachineCommand;
import frc.lib.team2930.commands.RunsWhenDisabledInstantCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.autonomous.AutosManager;
import frc.robot.autonomous.AutosManager.Auto;
import frc.robot.autonomous.AutosSubsystems;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoreCoral.ScoringDirection;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.commands.mechanism.elevator.ElevatorSetHeight;
import frc.robot.configs.SimulatorRobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.LED.RobotState;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.endEffector.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionModuleConfiguration;
import frc.robot.subsystems.visionGamepiece.*;
import frc.robot.visualization.MechanismVisualization;
import frc.robot.visualization.SimpleMechanismVisualization;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final LoggerEntry.EnumValue<RobotType> logRobotType =
      LoggerGroup.root.buildEnum("RobotType");
  private static final LoggerEntry.EnumValue<Mode> logRobotMode =
      LoggerGroup.root.buildEnum("RobotMode");

  private final Drivetrain drivetrain;
  private final DrivetrainWrapper drivetrainWrapper;
  private final AprilTagFieldLayout aprilTagLayout;
  private final Vision vision;
  private final Arm arm;
  private final Elevator elevator;
  private final Intake intake;
  private final EndEffector endEffector;
  private final VisionGamepiece visionGamepiece;
  private final LED led;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final LoggedDashboardChooser<String> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private final HashMap<String, Supplier<Auto>> stringToAutoSupplierMap = new HashMap<>();
  private final AutosManager autoManager;

  private Trigger gamepieceInRobot = new Trigger(() -> RobotStates.coralInRobot);

  public DigitalInput brakeModeButton = new DigitalInput(0);
  public DigitalInput zeroSensorsButton = new DigitalInput(1);

  private Trigger brakeModeButtonTrigger =
      new Trigger(() -> !brakeModeButton.get() && !DriverStation.isEnabled());

  private Trigger zeroSensorsButtonTrigger =
      new Trigger(() -> !zeroSensorsButton.get() && !DriverStation.isEnabled());

  private boolean brakeModeTriggered = true;

  private boolean is_teleop;
  private boolean is_autonomous;

  private boolean brakeModeFailure = false;

  private static LoggerGroup robotStateLogGroup = LoggerGroup.build("RobotState");
  private static LoggerEntry.EnumValue<ScoringLevel> logScoringLevelState =
      robotStateLogGroup.buildEnum("Levels/Level");
  private static LoggerEntry.Bool logL1State = robotStateLogGroup.buildBoolean("Levels/L1");
  private static LoggerEntry.Bool logL2State = robotStateLogGroup.buildBoolean("Levels/L2");
  private static LoggerEntry.Bool logL3State = robotStateLogGroup.buildBoolean("Levels/L3");
  private static LoggerEntry.Bool logL4State = robotStateLogGroup.buildBoolean("Levels/L4");
  private static LoggerEntry.Bool logAlgaeClearingState =
      robotStateLogGroup.buildBoolean("AlgaeClearing");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    RobotType robotType = Constants.RobotMode.getRobot();
    Mode mode = Constants.RobotMode.getMode();

    logRobotType.info(robotType);
    logRobotMode.info(mode);

    var config = robotType.config.get();
    aprilTagLayout = config.getAprilTagFieldLayout();

    if (mode == Mode.REPLAY) {
      drivetrain =
          new Drivetrain(
              config,
              new GyroIO.Fake(),
              new GyroIO.Fake(),
              config.getReplaySwerveModuleObjects(),
              () -> is_autonomous);

      vision =
          new Vision(
              aprilTagLayout,
              drivetrain::getPoseEstimatorPose,
              drivetrain::getRotationGyroOnly,
              drivetrain::addVisionEstimate,
              config.getReplayVisionModules());

      arm = new Arm(new ArmIO() {});
      elevator = new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
      endEffector = new EndEffector(new EndEffectorIO() {});
      visionGamepiece =
          new VisionGamepiece(
              new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPoseAtTimestamp);
      led =
          new LED(
              () -> brakeModeTriggered,
              drivetrain::isGyroConnected,
              () -> elevator.getHeight().in(Units.Inches) < 0.1);
    } else { // REAL and SIM robots HERE
      switch (robotType) {
        case ROBOT_SIMBOT_REAL_CAMERAS:
        case ROBOT_SIMBOT:
          com.ctre.phoenix6.unmanaged.Unmanaged.setPhoenixDiagnosticsStartTime(0.0);

          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIO.Fake(),
                  new GyroIO.Fake(),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);

          if (robotType == RobotType.ROBOT_SIMBOT_REAL_CAMERAS) {
            // Sim Robot, Real Cameras
            vision =
                new Vision(
                    aprilTagLayout,
                    drivetrain::getPoseEstimatorPose,
                    drivetrain::getRotationGyroOnly,
                    drivetrain::addVisionEstimate,
                    config.getVisionModuleObjects());

            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPoseAtTimestamp);

          } else {
            VisionModuleConfiguration[] visionModules = {
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT,
                  config,
                  drivetrain::getPoseEstimatorPose),
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT,
                  config,
                  drivetrain::getPoseEstimatorPose),
            };

            vision =
                new Vision(
                    aprilTagLayout,
                    drivetrain::getPoseEstimatorPose,
                    drivetrain::getRotationGyroOnly,
                    drivetrain::addVisionEstimate,
                    visionModules);

            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIOSim(config, drivetrain::getPoseEstimatorPose),
                    drivetrain::getPoseEstimatorPoseAtTimestamp);
          }

          arm = new Arm(new ArmIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          intake = new Intake(new IntakeIOSim());
          endEffector = new EndEffector(new EndEffectorIO() {});
          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> elevator.getHeight().in(Units.Inches) > 0.1);
          break;

        case ROBOT_2023_RETIRED_ROBER:
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIOPigeon2(config, config.getGyroCANID()),
                  new GyroIOPigeon2(config, Constants.CanIDs.GYRO_2_CAN_ID),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);

          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          visionGamepiece =
              new VisionGamepiece(
                  new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPoseAtTimestamp);

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> elevator.getHeight().in(Units.Inches) > 0.1);
          break;

        case ROBOT_2024_RETIRED_MAESTRO:
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIOPigeon2(config, config.getGyroCANID()),
                  new GyroIOPigeon2(config, Constants.CanIDs.GYRO_2_CAN_ID),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);
          intake = new Intake(new IntakeIOReal());
          endEffector = new EndEffector(new EndEffectorIOReal());
          elevator = new Elevator(new ElevatorIOReal());
          arm = new Arm(new ArmIOReal());
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());

          visionGamepiece =
              new VisionGamepiece(
                  new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPoseAtTimestamp);

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> elevator.getHeight().in(Units.Inches) > 0.1);
          break;

        case ROBOT_2025:
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIOPigeon2(config, config.getGyroCANID()),
                  new GyroIOPigeon2(config, Constants.CanIDs.GYRO_2_CAN_ID),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);
          intake = new Intake(new IntakeIOReal());
          endEffector = new EndEffector(new EndEffectorIOReal());
          elevator = new Elevator(new ElevatorIOReal());
          arm = new Arm(new ArmIOReal());
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());

          visionGamepiece =
              new VisionGamepiece(
                  new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPoseAtTimestamp);

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> elevator.getHeight().in(Units.Inches) > 0.1);
          break;

        default:
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIO.Fake(),
                  new GyroIO.Fake(),
                  config.getReplaySwerveModuleObjects(),
                  () -> is_autonomous);
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          visionGamepiece =
              new VisionGamepiece(
                  new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPoseAtTimestamp);

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> elevator.getHeight().in(Units.Inches) > 0.1);
          break;
      }
    }

    drivetrainWrapper =
        new DrivetrainWrapper(
            drivetrain,
            () ->
                Constants.ElevatorConstants.SPEED_SCALAR_MAP.get(
                    elevator.getHeight().in(Units.Inches)));

    // FIXME: uncomment and fix if we want to use path planner swerve
    // FIXME: remove once we are happy with path planner based swerve
    // characterization
    // AutoBuilder.configureHolonomic(
    //     drivetrain::getPoseEstimatorPose,
    //     drivetrain::setPose,
    //     drivetrain::getChassisSpeeds,
    //     drivetrainWrapper::setVelocityOverride,
    //     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
    // in
    //         // your Constants class
    //         new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
    //         new PIDConstants(0.0, 0.0, 0.0), // Rotation PID constants
    //         4.5, // Max module speed, in m/s
    //         config.getDriveBaseRadius(), // Drive base radius in meters.
    //         // Distance from robot center to
    //         // furthest module.
    //         new ReplanningConfig() // Default path replanning config. See the
    //         // API for the options
    //         // here
    //         ),
    //     () -> false,
    //     drivetrain);

    var subsystems = new AutosSubsystems(drivetrainWrapper, visionGamepiece, led);

    autoManager = new AutosManager(subsystems, config, autoChooser, stringToAutoSupplierMap);

    drivetrain.setDefaultCommand(
        new DrivetrainDefaultTeleopDrive(
            drivetrainWrapper,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ----------- DRIVER CONTROLS ------------

    driverController
        .back()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Pose2d pose = drivetrain.getPoseEstimatorPose();
                  drivetrain.setPose(
                      new Pose2d(pose.getX(), pose.getY(), Constants.zeroRotation2d));
                },
                drivetrain));

    driverController
        .rightBumper()
        .whileTrue(new IntakeGamepiece(intake, arm, elevator, endEffector))
        .whileTrue(
            Commands.run(
                    () -> {
                      if (RobotStates.coralInRobot) {
                        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
                        led.setBaseRobotState(BaseRobotState.INTAKE_SUCCESS);
                      }
                    })
                .finallyDo(
                    () -> {
                      driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                      led.setBaseRobotState(BaseRobotState.GAMEPIECE_STATUS);
                    }));

    driverController
        .leftTrigger()
        .whileTrue(
            new RunStateMachineCommand(
                () ->
                    new ScoreCoral(
                        drivetrainWrapper,
                        elevator,
                        arm,
                        endEffector,
                        led,
                        ScoringDirection.LEFT,
                        (r) -> driverController.getHID().setRumble(RumbleType.kBothRumble, r))));

    driverController
        .rightTrigger()
        .whileTrue(
            new RunStateMachineCommand(
                () ->
                    new ScoreCoral(
                        drivetrainWrapper,
                        elevator,
                        arm,
                        endEffector,
                        led,
                        ScoringDirection.RIGHT,
                        (r) -> driverController.getHID().setRumble(RumbleType.kBothRumble, r))));

    // driverController.povRight().onTrue(MechanismActions.coralStationPosition(elevator, arm));
    driverController
        .povLeft()
        .onTrue(MechanismActions.reefPosition(elevator, arm, ScoringLevel.L4));

    // Change scoring height
    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    Pose2d[] reefAprilTagPose = {
      layout.getTagPose(6).get().toPose2d(),
      layout.getTagPose(7).get().toPose2d(),
      layout.getTagPose(8).get().toPose2d(),
      layout.getTagPose(9).get().toPose2d(),
      layout.getTagPose(10).get().toPose2d(),
      layout.getTagPose(11).get().toPose2d(),
      layout.getTagPose(17).get().toPose2d(),
      layout.getTagPose(18).get().toPose2d(),
      layout.getTagPose(19).get().toPose2d(),
      layout.getTagPose(20).get().toPose2d(),
      layout.getTagPose(21).get().toPose2d(),
      layout.getTagPose(22).get().toPose2d()
    };

    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.scoringLevel = ScoringLevel.L4;
                }));
    driverController
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.scoringLevel = ScoringLevel.L3;
                }));
    driverController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.scoringLevel = ScoringLevel.L2;
                }));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.scoringLevel = ScoringLevel.L1;
                }));

    driverController
        .rightStick()
        .toggleOnTrue(
            new RotateToAngle(
                drivetrainWrapper,
                () -> {
                  Pose2d robotTranslation = drivetrainWrapper.getPoseEstimatorPose(true);

                  Pose2d nearestAprilTag = null;
                  nearestAprilTag = findNearestAprilTag(robotTranslation, reefAprilTagPose);

                  Rotation2d finalRotationValue = nearestAprilTag.getRotation();

                  return finalRotationValue;
                },
                () -> drivetrainWrapper.getPoseEstimatorPose(true)));
    driverController
        .leftStick()
        .toggleOnTrue(
            new RotateToAngle(
                drivetrainWrapper,
                () -> {
                  Pose2d robotTranslation = drivetrainWrapper.getPoseEstimatorPose(true);

                  return faceTowardsCenter(robotTranslation, reefAprilTagPose);
                },
                () -> drivetrainWrapper.getPoseEstimatorPose(true)));

    // Change clearing algae

    driverController
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.clearingAglae = true;
                }));

    driverController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.clearingAglae = false;
                }));

    // ---------- OPERATOR CONTROLS -----------

    operatorController.a().whileTrue(new ElevatorSetHeight(elevator, Units.Inches.of(5)));
    operatorController.b().whileTrue(new ElevatorSetHeight(elevator, Units.Inches.of(18)));

    // Toggle clearing algae

    operatorController
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.clearingAglae = true;
                }));
    operatorController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.clearingAglae = false;
                }));

    if (!DriverStation.isFMSAttached())
      operatorController
          .povDown()
          .whileTrue(
              new WheelRadiusCharacterization(
                  drivetrainWrapper, Constants.RobotMode.getRobot().config.get(), 0.5));

    // ---------- ON-ROBOT CONTROLS ------------

    zeroSensorsButtonTrigger.onTrue(
        new RunsWhenDisabledInstantCommand(
            () -> {
              elevator.resetSensorToHomePosition();
              arm.resetSensorToHomePosition();
              led.setRobotState(RobotState.ZERO_SUBSYSTEMS);
            })); // TODO: add climber?

    brakeModeButtonTrigger.onTrue(
        new ConditionalCommand(
            new RunsWhenDisabledInstantCommand(
                () -> {
                  boolean armSuccess = arm.setNeutralMode(NeutralModeValue.Coast);
                  boolean elevatorSuccess = elevator.setNeutralMode(NeutralModeValue.Coast);

                  brakeModeFailure = !armSuccess || !elevatorSuccess;

                  brakeModeTriggered = false;
                  led.setRobotState(
                      brakeModeFailure ? RobotState.BRAKE_MODE_FAILED : RobotState.BRAKE_MODE_OFF);
                },
                elevator,
                arm),
            new RunsWhenDisabledInstantCommand(
                () -> {
                  boolean armSuccess = arm.setNeutralMode(NeutralModeValue.Brake);
                  boolean elevatorSuccess = elevator.setNeutralMode(NeutralModeValue.Brake);

                  brakeModeFailure = !armSuccess || !elevatorSuccess;
                  brakeModeTriggered = true;
                  led.setRobotState(
                      brakeModeFailure ? RobotState.BRAKE_MODE_FAILED : RobotState.BRAKE_MODE_ON);
                },
                elevator,
                arm),
            () -> brakeModeTriggered)); // TODO: add climber?

    // Add Reset and Reboot buttons to SmartDashboard
    // TODO: add correct vision addresses
    SmartDashboard.putData(
        "PV Restart SW 1_Shooter_Left",
        new RunsWhenDisabledInstantCommand(() -> Vision.restartPhotonVision("10.29.30.13")));

    SmartDashboard.putData(
        "PV REBOOT 1_Shooter_Left",
        new RunsWhenDisabledInstantCommand(() -> Vision.rebootPhotonVision("10.29.30.13")));

    SmartDashboard.putData(
        "PV Restart SW 2_Shooter_Right",
        new RunsWhenDisabledInstantCommand(() -> Vision.restartPhotonVision("10.29.30.14")));

    SmartDashboard.putData(
        "PV REBOOT 2_Shooter_Right",
        new RunsWhenDisabledInstantCommand(() -> Vision.rebootPhotonVision("10.29.30.14")));

    SmartDashboard.putData(
        "USE GYRO 1", new RunsWhenDisabledInstantCommand(() -> drivetrain.chooseWhichGyro(false)));
    SmartDashboard.putData(
        "USE GYRO 2", new RunsWhenDisabledInstantCommand(() -> drivetrain.chooseWhichGyro(true)));
  }

  /**
   * Finds the nearest april tag position relative to the current robot translation
   *
   * @param robotTranslation - Current robot translation
   * @param reefAprilTagPose - List of all april tag positions
   * @return Nearest april tag
   */
  public static Pose2d findNearestAprilTag(Pose2d robotTranslation, Pose2d[] reefAprilTagPose) {
    Pose2d nearestAprilTag = null;
    int start;
    double minDistance = 10000000000.0;

    start = Constants.isRedAlliance() ? 0 : 6;

    for (int i = start; i < start + 6; i++) {
      Translation2d aprilTag = reefAprilTagPose[i].getTranslation();
      if (Constants.unusedCode) {
        System.out.printf("Tag %d: %s\n", i, reefAprilTagPose[i]);
      }

      double distance = robotTranslation.getTranslation().getDistance(aprilTag);
      if (distance < minDistance) {
        minDistance = distance;
        nearestAprilTag = reefAprilTagPose[i];
      }
    }
    if (Constants.unusedCode) {
      System.out.printf("Closest: %s\n", nearestAprilTag);
    }

    return nearestAprilTag;
  }

  /**
   * Makes the robot always face towards the center of the reef
   *
   * @param robotTranslation - Current robot translation
   * @param reefAprilTagPose - List of all april tag positions
   * @return Rotates robot to face center
   */
  public static Rotation2d faceTowardsCenter(Pose2d robotTranslation, Pose2d[] reefAprilTagPose) {
    Pose2d tag1;

    tag1 = Constants.isRedAlliance() ? reefAprilTagPose[1] : reefAprilTagPose[8];

    var center = FieldConstants.BLUE_REEF_CENTER_POSE;
    Pose2d centerPose = new Pose2d();

    AllianceFlipUtil.flipPoseForAlliance(centerPose);

    double targetX = center.getX();
    double targetY = center.getY();

    double robotX = robotTranslation.getX();
    double robotY = robotTranslation.getY();

    double angleToCenter = Math.atan2(targetY - robotY, targetX - robotX);

    double robotAngle = robotTranslation.getRotation().getRadians();
    double angularError = angleToCenter - robotAngle;
    double normalizedAngleToCenter = Math.atan2(Math.sin(angleToCenter), Math.cos(angleToCenter));

    var offset = centerPose.minus(robotTranslation);

    Rotation2d finalRotation = new Rotation2d(normalizedAngleToCenter);
    return finalRotation.rotateBy(Rotation2d.k180deg);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public LoggedDashboardChooser<String> getAutonomousChooser() {
    return autoChooser;
  }

  public Supplier<Auto> getAutoSupplierForString(String string) {
    return stringToAutoSupplierMap.getOrDefault(string, autoManager::doNothing);
  }

  public void setPose(Pose2d pose) {
    drivetrain.setPose(pose);
  }

  public void matchRawOdometryToPoseEstimatorValue() {
    drivetrain.setRawOdometryPose(drivetrain.getPoseEstimatorPose());
  }

  public void applyToDrivetrain() {
    drivetrainWrapper.apply();
  }

  public void enterDisabled() {
    resetSubsystems();
    vision.useMaxDistanceAwayFromExistingEstimate(false);
    vision.useGyroBasedFilteringForVision(false);

    is_teleop = false;
    is_autonomous = false;
  }

  public void enterAutonomous() {
    // setBrakeMode();
    vision.useMaxDistanceAwayFromExistingEstimate(true);
    vision.useGyroBasedFilteringForVision(true);

    visionGamepiece.setPipelineIndex(0);

    is_teleop = false;
    is_autonomous = true;
  }

  public void enterTeleop() {
    // setBrakeMode();
    resetDrivetrainResetOverrides();
    vision.useMaxDistanceAwayFromExistingEstimate(true);
    vision.useGyroBasedFilteringForVision(true);

    led.setBaseRobotState(BaseRobotState.GAMEPIECE_STATUS);

    visionGamepiece.setPipelineIndex(1);

    is_teleop = true;
    is_autonomous = false;
  }

  public void resetDrivetrainResetOverrides() {
    drivetrainWrapper.resetVelocityOverride();
    drivetrainWrapper.resetRotationOverride();
  }

  public void updateVisualization() {
    // Disable visualization for real robot
    if (Robot.isReal()) return;

    MechanismVisualization.logMechanism();
    SimpleMechanismVisualization.updateVisualization(elevator.getHeight(), arm.getAngle());
    SimpleMechanismVisualization.logMechanism();
  }

  public void resetSubsystems() {}

  public void setBrakeMode() {
    brakeModeTriggered = true;
  }

  public void updateRobotState() {
    RobotStates.coralInEndEffector = endEffector.tofDistance().in(Units.Inches) < 11.0;

    ScoringLevel level = RobotStates.scoringLevel;
    logScoringLevelState.info(level);
    logL1State.info(level == ScoringLevel.L1);
    logL2State.info(level == ScoringLevel.L2);
    logL3State.info(level == ScoringLevel.L3);
    logL4State.info(level == ScoringLevel.L4);
    logAlgaeClearingState.info(RobotStates.clearingAglae);

    if (gamepieceInRobot.getAsBoolean() && !led.getGamepieceStatus()) {
      led.setGamepieceStatus(true);
    }

    if (!gamepieceInRobot.getAsBoolean() && led.getGamepieceStatus()) {
      led.setGamepieceStatus(false);
    }
  }
}
