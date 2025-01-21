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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.RunStateMachineCommand;
import frc.lib.team2930.commands.RunsWhenDisabledInstantCommand;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.autonomous.AutosManager;
import frc.robot.autonomous.AutosManager.Auto;
import frc.robot.autonomous.AutosSubsystems;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoreCoral.ScoringDirection;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.commands.led.LedSetStateForSeconds;
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

  private Trigger gamepieceInRobot =
      new Trigger(
          () -> false); // TODO: move to constructor and add condition for gamepiece in robot
  private final Trigger
      twenty_Second_Warning; // TODO: decide whether this is necessary. If it is, make sure to test
  // it.

  public DigitalInput breakModeButton = new DigitalInput(0);
  public DigitalInput homeSensorsButton = new DigitalInput(1);

  private Trigger breakModeButtonTrigger =
      new Trigger(() -> !breakModeButton.get() && !DriverStation.isEnabled());

  private Trigger homeSensorsButtonTrigger =
      new Trigger(() -> !homeSensorsButton.get() && !DriverStation.isEnabled());

  private boolean brakeModeTriggered = true;

  private boolean is_teleop;
  private boolean is_autonomous;

  private boolean brakeModeFailure = false;

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
      led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected);
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
          led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected);
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

          led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected);
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

          led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected);
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

          led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected);
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

          led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected);
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

    twenty_Second_Warning = new Trigger(() -> DriverStation.getMatchTime() > 115);

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
        .whileTrue(
            new IntakeGamepiece()
                .finallyDo(
                    (interrupted) -> {
                      if (!interrupted)
                        CommandScheduler.getInstance()
                            .schedule(new LedSetStateForSeconds(led, RobotState.INTAKE_SUCCESS, 1));
                    }))
        .whileTrue(
            Commands.run(
                    () -> {
                      // TODO: change condition to if gamepiece is detected
                      if (true) {
                        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
                        led.setRobotState(RobotState.INTAKE_SUCCESS);
                      } else {
                        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        led.setRobotState(RobotState.BASE);
                      }
                    })
                .finallyDo(
                    () -> {
                      driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                      led.setRobotState(RobotState.BASE);
                    }));

    driverController
        .leftTrigger()
        .whileTrue(
            new RunStateMachineCommand(
                () -> new ScoreCoral(drivetrainWrapper, elevator, arm, ScoringDirection.LEFT)));

    driverController
        .rightTrigger()
        .whileTrue(
            new RunStateMachineCommand(
                () -> new ScoreCoral(drivetrainWrapper, elevator, arm, ScoringDirection.RIGHT)));

    driverController.a().whileTrue(MechanismActions.stowPosition(elevator, arm));

    // Change scoring height

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
    // ---------- OPERATOR CONTROLS -----------

    operatorController.a().whileTrue(new ElevatorSetHeight(elevator, Units.Inches.of(5)));
    operatorController.b().whileTrue(new ElevatorSetHeight(elevator, Units.Inches.of(18)));

    twenty_Second_Warning.onTrue(
        new LedSetStateForSeconds(led, RobotState.TWENTY_SECOND_WARNING, 0.5));

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

    homeSensorsButtonTrigger.onTrue(
        new RunsWhenDisabledInstantCommand(
            () -> {
              elevator.resetSensorToHomePosition();
              arm.resetSensorToHomePosition();
            }));

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
    // FIXME: need to remove max distance away from current estimate restriction for
    // vision
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
  }

  public void resetSubsystems() {}

  public void setBrakeMode() {
    brakeModeTriggered = true;
  }

  public void updateLedGamepieceState() {
    if (gamepieceInRobot.getAsBoolean() && !led.getGamepieceStatus()) {
      led.setGamepieceStatus(true);
    }
    if (!gamepieceInRobot.getAsBoolean() && led.getGamepieceStatus()) {
      led.setGamepieceStatus(false);
    }
  }
}
