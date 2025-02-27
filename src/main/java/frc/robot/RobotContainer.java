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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.*;
import frc.lib.team2930.commands.RunsWhenDisabledInstantCommand;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.autonomous.AutosManager;
import frc.robot.autonomous.AutosManager.Auto;
import frc.robot.autonomous.AutosSubsystems;
import frc.robot.autonomous.records.CoralStationLocation;
import frc.robot.autonomous.records.ScoringLocation;
import frc.robot.autonomous.records.ScoringLocation.ReefSide;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoreCoral.ScoringDirection;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.ClimberSetAngle;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.commands.endEffector.EndEffectorSetRPM;
import frc.robot.commands.intake.IntakeGround;
import frc.robot.commands.intake.IntakeSetPivotAngle;
import frc.robot.commands.intake.IntakeSetRPM;
import frc.robot.commands.intake.ScoreAlgae;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.commands.mechanism.WaitUntilMovedDist;
import frc.robot.commands.mechanism.arm.ArmManualControl;
import frc.robot.commands.mechanism.elevator.ElevatorManualControl;
import frc.robot.configs.SimulatorRobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.LED.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOReal;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionModuleConfiguration;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIO;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIOReal;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIOSim;
import frc.robot.visualization.MechanismVisualization;
import frc.robot.visualization.SimpleMechanismVisualization;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
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
  private final Trigger algaeInRobot;
  private final EndEffector endEffector;
  private final VisionGamepiece visionGamepiece;
  private final LED led;
  private final Climber climber;

  private final XboxControllerWrapper driverController = new XboxControllerWrapper(0);
  private final XboxControllerWrapper operatorController = new XboxControllerWrapper(1);

  private final LoggedDashboardChooser<String> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private final LoggedDashboardChooser<Boolean> flipAutoChooser =
      new LoggedDashboardChooser<>("Flip Auto?");

  private final List<LoggedDashboardChooser<ReefSide>> scoringPosChooser = new ArrayList<>();
  private final List<LoggedDashboardChooser<CoralStationLocation>> coralStationPosChooser =
      new ArrayList<>();
  private final int customGamepieceCount = 5;

  private final HashMap<String, Supplier<Auto>> stringToAutoSupplierMap = new HashMap<>();
  private final AutosManager autoManager;

  public DigitalInput brakeModeButton = new DigitalInput(0);
  public DigitalInput zeroSensorsButton = new DigitalInput(1);

  private final Trigger brakeModeButtonTrigger =
      new Trigger(() -> !brakeModeButton.get() && !DriverStation.isEnabled());

  private final Trigger zeroSensorsButtonTrigger =
      new Trigger(() -> !zeroSensorsButton.get() && !DriverStation.isEnabled());

  private boolean brakeModeTriggered = true;

  private boolean is_teleop;
  private boolean is_autonomous;

  private boolean brakeModeFailure = false;

  private final TunableNumberGroup tunableNumberGroup = new TunableNumberGroup("RobotContainer");
  private final LoggedTunableNumber tunableX = tunableNumberGroup.build("TunableX", 13.75);
  private final LoggedTunableNumber tunableY = tunableNumberGroup.build("TunableY", 5.15);
  private final LoggedTunableNumber tunableAngle = tunableNumberGroup.build("TunableAngle", 0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    DriverStation.silenceJoystickConnectionWarning(true);

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
              drivetrain::getReefPoseEstimatorPose,
              drivetrain::getRotationGyroOnly,
              drivetrain::addVisionEstimate,
              config.getReplayVisionModules());

      arm = new Arm(new ArmIO() {});
      elevator = new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
      endEffector = new EndEffector(new EndEffectorIO() {});
      climber = new Climber(new ClimberIO() {});

      if (Constants.unusedCode) {
        visionGamepiece =
            new VisionGamepiece(
                new VisionGamepieceIO() {}, drivetrain::getReefPoseEstimatorPoseAtTimestamp);
      } else {
        visionGamepiece = null;
      }

      led =
          new LED(
              () -> brakeModeTriggered,
              drivetrain::isGyroConnected,
              () -> elevator.getHeight().in(Units.Inches) < 0.3);
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
                    drivetrain::getReefPoseEstimatorPose,
                    drivetrain::getRotationGyroOnly,
                    drivetrain::addVisionEstimate,
                    config.getVisionModuleObjects());

            if (Constants.unusedCode) {
              visionGamepiece =
                  new VisionGamepiece(
                      new VisionGamepieceIOReal(), drivetrain::getReefPoseEstimatorPoseAtTimestamp);
            } else {
              visionGamepiece = null;
            }

          } else {
            VisionModuleConfiguration[] visionModules = {
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT,
                  config,
                  drivetrain::getReefPoseEstimatorPose),
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT,
                  config,
                  drivetrain::getReefPoseEstimatorPose),
            };

            vision =
                new Vision(
                    aprilTagLayout,
                    drivetrain::getReefPoseEstimatorPose,
                    drivetrain::getRotationGyroOnly,
                    drivetrain::addVisionEstimate,
                    visionModules);

            if (Constants.unusedCode) {
              visionGamepiece =
                  new VisionGamepiece(
                      new VisionGamepieceIOSim(config, drivetrain::getReefPoseEstimatorPose),
                      drivetrain::getReefPoseEstimatorPoseAtTimestamp);
            } else {
              visionGamepiece = null;
            }
          }

          arm = new Arm(new ArmIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          intake = new Intake(new IntakeIOSim());
          endEffector = new EndEffector(new EndEffectorIOSim());
          climber = new Climber(new ClimberIOSim());
          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> elevator.getHeight().in(Units.Inches) > 0.3);
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
                  drivetrain::getReefPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          climber = new Climber(new ClimberIO() {});

          if (Constants.unusedCode) {
            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIO() {}, drivetrain::getReefPoseEstimatorPoseAtTimestamp);
          } else {
            visionGamepiece = null;
          }

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> elevator.getHeight().in(Units.Inches) > 0.3);
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
          climber = new Climber(new ClimberIO() {});
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getReefPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());

          if (Constants.unusedCode) {
            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIOReal(), drivetrain::getReefPoseEstimatorPoseAtTimestamp);
          } else {
            visionGamepiece = null;
          }

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> elevator.getHeight().in(Units.Inches) > 0.3);
          break;

        case ROBOT_2025_HOLO:
          try {
            Thread.sleep(1000);
          } catch (InterruptedException e) {
            System.out.println("sleep interrupted");
          }
          intake = new Intake(new IntakeIOReal());
          endEffector = new EndEffector(new EndEffectorIOReal());
          elevator = new Elevator(new ElevatorIOReal());
          arm = new Arm(new ArmIOReal());
          climber = new Climber(new ClimberIOReal());
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIOPigeon2(config, config.getGyroCANID()),
                  new GyroIO.Fake(),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getReefPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());

          if (Constants.unusedCode) {
            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIOReal(), drivetrain::getReefPoseEstimatorPoseAtTimestamp);
          } else {
            visionGamepiece = null;
          }

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> elevator.getHeight().in(Units.Inches) > 0.3);

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
                  drivetrain::getReefPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          climber = new Climber(new ClimberIO() {});

          if (Constants.unusedCode) {
            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIO() {}, drivetrain::getReefPoseEstimatorPoseAtTimestamp);
          } else {
            visionGamepiece = null;
          }

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> elevator.getHeight().in(Units.Inches) > 0.3);
          break;
      }
    }

    algaeInRobot =
        new Trigger(() -> !intake.intakeTimeOfFlight() && intake.rollerStallDetected())
            .debounce(.1);

    drivetrainWrapper =
        new DrivetrainWrapper(
            drivetrain,
            () ->
                Constants.ElevatorConstants.SPEED_SCALAR_MAP.get(
                    elevator.getHeight().in(Units.Inches)));

    flipAutoChooser.addDefaultOption("No", false);
    flipAutoChooser.addOption("Yes", true);

    for (int i = 0; i < customGamepieceCount; i++) {
      LoggedDashboardChooser<ReefSide> chooser = new LoggedDashboardChooser<>(i + " CustomScoring");
      for (ReefSide side : ScoringLocation.ReefSide.values()) chooser.addOption(side.name(), side);
      chooser.addDefaultOption(ReefSide.CH.name(), ReefSide.CH);
      scoringPosChooser.add(chooser);
    }

    for (int i = 1; i < customGamepieceCount; i++) {
      LoggedDashboardChooser<CoralStationLocation> chooser =
          new LoggedDashboardChooser<>(i + " CustomPickup");
      for (CoralStationLocation side : CoralStationLocation.values()) {
        chooser.addOption(side.name(), side);
      }
      chooser.addDefaultOption(CoralStationLocation.IA.name(), CoralStationLocation.IA);
      coralStationPosChooser.add(chooser);
    }

    var subsystems = new AutosSubsystems(drivetrainWrapper, elevator, arm, endEffector, led);

    autoManager =
        new AutosManager(
            subsystems,
            config,
            autoChooser,
            stringToAutoSupplierMap,
            () -> {
              var result = flipAutoChooser.get();
              if (result != null) {
                return result;
              }
              return false;
            },
            this::getCustomScoringLocations,
            this::getCustomCoralStationLocations);

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
        .registerTrigger(XboxControllerWrapper.Button.back, "Zero Robot")
        .onTrue(
            Commands.runOnce(
                () -> {
                  Pose2d pose = drivetrain.getReefPoseEstimatorPose();
                  drivetrain.setPose(
                      new Pose2d(pose.getX(), pose.getY(), Constants.zeroRotation2d));
                },
                drivetrain));

    driverController
        .registerTrigger(XboxControllerWrapper.Button.start, "X Stance")
        .onTrue(Commands.runOnce(drivetrain::stopWithX));

    driverController
        .registerTrigger(XboxControllerWrapper.Button.rightStick, "Clear Algae")
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
                            (r) -> driverController.getHID().setRumble(RumbleType.kBothRumble, r),
                            Constants.RobotMode.getRobot().config.get(),
                            true))
                .finallyDo(() -> led.setBaseRobotState(BaseRobotState.LEVEL_MODE)));

    // driverController
    //     .registerTrigger(XboxControllerWrapper.Button.rightStick, "Teleop Autonomous")
    //     .whileTrue(
    //         new RunStateMachineCommand(
    //             () ->
    //                 new AutoStateMachine(
    //                     new AutosSubsystems(drivetrainWrapper, elevator, arm, endEffector, led),
    //                     Constants.RobotMode.getRobot().config.get(),
    //                     (r) -> driverController.getHID().setRumble(RumbleType.kBothRumble, r))));

    driverController
        .registerTrigger(XboxControllerWrapper.Button.rightBumper, "Intake Coral Station")
        .whileTrue(
            CommandComposer.intakeCoralFromStation(
                drivetrainWrapper, endEffector, elevator, arm, led, driverController, true));

    driverController
        .registerTrigger(XboxControllerWrapper.Button.povLeft, "Score Algae")
        .whileTrue(new ScoreAlgae(intake));

    driverController
        .registerTrigger(XboxControllerWrapper.Button.leftTrigger, "Scoring Alignment Left")
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
                            (r) -> driverController.getHID().setRumble(RumbleType.kBothRumble, r),
                            Constants.RobotMode.getRobot().config.get(),
                            false))
                .finallyDo(() -> led.setBaseRobotState(BaseRobotState.LEVEL_MODE)));

    driverController
        .registerTrigger(XboxControllerWrapper.Button.rightTrigger, "Scoring Alignment Right")
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
                            (r) -> driverController.getHID().setRumble(RumbleType.kBothRumble, r),
                            Constants.RobotMode.getRobot().config.get(),
                            false))
                .finallyDo(() -> led.setBaseRobotState(BaseRobotState.LEVEL_MODE)));

    // var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Pose2d[] reefAprilTagPose = {
    //   layout.getTagPose(6).get().toPose2d(),
    //   layout.getTagPose(7).get().toPose2d(),
    //   layout.getTagPose(8).get().toPose2d(),
    //   layout.getTagPose(9).get().toPose2d(),
    //   layout.getTagPose(10).get().toPose2d(),
    //   layout.getTagPose(11).get().toPose2d(),
    //   layout.getTagPose(17).get().toPose2d(),
    //   layout.getTagPose(18).get().toPose2d(),
    //   layout.getTagPose(19).get().toPose2d(),
    //   layout.getTagPose(20).get().toPose2d(),
    //   layout.getTagPose(21).get().toPose2d(),
    //   layout.getTagPose(22).get().toPose2d()
    // };

    // Pose2d[] coralStationPose = {
    //   layout.getTagPose(1).get().toPose2d(),
    //   layout.getTagPose(2).get().toPose2d(),
    //   layout.getTagPose(12).get().toPose2d(),
    //   layout.getTagPose(13).get().toPose2d()
    // };

    // Change scoring height

    driverController
        .registerTrigger(XboxControllerWrapper.Button.y, "Score L4")
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.scoringLevel = ScoringLevel.L4;
                  led.setBaseRobotState(BaseRobotState.LEVEL_MODE);
                }));
    driverController
        .registerTrigger(XboxControllerWrapper.Button.x, "Score L3")
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.scoringLevel = ScoringLevel.L3;
                  led.setBaseRobotState(BaseRobotState.LEVEL_MODE);
                }));
    driverController
        .registerTrigger(XboxControllerWrapper.Button.a, "Score L2")
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.scoringLevel = ScoringLevel.L2;
                  led.setBaseRobotState(BaseRobotState.LEVEL_MODE);
                }));
    driverController
        .registerTrigger(XboxControllerWrapper.Button.b, "Score L1")
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotStates.scoringLevel = ScoringLevel.L1;
                  led.setBaseRobotState(BaseRobotState.LEVEL_MODE);
                }));

    driverController
        .registerTrigger(XboxControllerWrapper.Button.leftStick, "ScoreCoral")
        .onTrue(
            Commands.waitUntil(() -> !RobotStates.coralInEndEffector)
                .deadlineFor(
                    Commands.runOnce(
                            () -> {
                              var ioSim = endEffector.getSim();
                              if (ioSim != null) {
                                ioSim.scoringSideTofDetecting = false;
                                ioSim.nonScoringSideTofDetecting = false;
                                FieldStates.setScoringLocationFilled(
                                    new ScoringLocation(
                                        RobotStates.targetReefSide, RobotStates.scoringLevel));
                              }
                            })
                        .andThen(new EndEffectorSetRPM(-6000))));
    // .toggleOnTrue(
    //     new RotateToAngle(
    //         drivetrainWrapper,
    //         () -> {
    //           Pose2d robotTranslation = drivetrainWrapper.getReefPoseEstimatorPose(true);

    //           Rotation2d finalRotationValue =
    //               RobotStates.coralInEndEffector
    //                   ? findNearestAprilTag(robotTranslation, reefAprilTagPose).getRotation()
    //                   : findNearestCoralStation(robotTranslation, coralStationPose)
    //                       .getRotation();
    //           return finalRotationValue;
    //         },
    //         () -> drivetrainWrapper.getReefPoseEstimatorPose(true)));

    // driverController
    //     .registerTrigger(XboxControllerWrapper.Button.leftStick, "Face center")
    //     .toggleOnTrue(
    //         new RotateToAngle(
    //             drivetrainWrapper,
    //             () -> {
    //               Pose2d robotTranslation = drivetrainWrapper.getReefPoseEstimatorPose(true);

    //               return faceTowardsCenter(robotTranslation, reefAprilTagPose);
    //             },
    //             () -> drivetrainWrapper.getReefPoseEstimatorPose(true)));

    // Manual Algae Clearing
    // driverController
    //     .registerTrigger(XboxControllerWrapper.Button.povUp, "Clear Algae High Position")
    //     .onTrue(MechanismActions.clearAlgaeHigh1Position(elevator, arm))
    //     .onFalse(MechanismActions.clearAlgaeHigh2Position(elevator, arm));

    // driverController
    //     .registerTrigger(XboxControllerWrapper.Button.povDown, "Clear Algae Low Position")
    //     .onTrue(MechanismActions.clearAlgaeLow1Position(elevator, arm))
    //     .onFalse(MechanismActions.clearAlgaeLow2Position(elevator, arm));

    // Automatic Algae Clearing
    // driverController
    //     .registerTrigger(XboxControllerWrapper.Button.povUp, "Clearing Algae")
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               RobotStates.clearingAlgae = true;
    //             }));

    // driverController
    //     .registerTrigger(XboxControllerWrapper.Button.povDown, "Done Clearing Algae")
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               RobotStates.clearingAlgae = false;
    //             }));

    driverController
        .registerTrigger(XboxControllerWrapper.Button.leftBumper, "Ground Intake")
        .whileTrue(
            new IntakeGround(intake).alongWith(MechanismActions.stowPosition(elevator, arm)));

    driverController
        .registerTrigger(XboxControllerWrapper.Button.povDown, "Climb")
        .onTrue(new Climb(climber, driverController.getPovDown()));
    // ---------- OPERATOR CONTROLS -----------

    // Manual mech positions

    // Reef positions
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.povDown, "L1 Position")
        .onTrue(MechanismActions.reefPosition(elevator, arm, ScoringLevel.L1));
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.povRight, "L2 Position")
        .onTrue(MechanismActions.reefPosition(elevator, arm, ScoringLevel.L2));
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.povLeft, "L3 Position")
        .onTrue(MechanismActions.reefPosition(elevator, arm, ScoringLevel.L3));
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.povUp, "L4 Position")
        .onTrue(MechanismActions.reefPosition(elevator, arm, ScoringLevel.L4));

    // Coral Station position
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.y, "CoralStation Position")
        .onTrue(MechanismActions.coralStationPosition(elevator, arm));

    // Stow position
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.x, "Score Prep Position")
        .onTrue(MechanismActions.stowPosition(elevator, arm));

    // Eject
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.leftBumper, "Intake Eject")
        .whileTrue(new IntakeSetRPM(intake, -1000));

    // Intake
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.rightBumper, "Intake")
        .whileTrue(new IntakeSetRPM(intake, 1000));

    // Intake positions
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.a, "Intake Pivot In")
        .onTrue(
            new IntakeSetPivotAngle(
                    intake, Constants.IntakeConstants.PivotConstants.PIVOT_STOWED_ANGLE)
                .andThen(Commands.runOnce(() -> intake.setHoldAlgae(false))));
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.rightTrigger, "Intake Pivot Out")
        .onTrue(
            new IntakeSetPivotAngle(
                intake, Constants.IntakeConstants.PivotConstants.MIN_PIVOT_ANGLE));

    // End Effector Rotation
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.start, "End Effector Out")
        .whileTrue(new EndEffectorSetRPM(-1000));

    operatorController
        .registerTrigger(XboxControllerWrapper.Button.back, "End Effector In")
        .whileTrue(new EndEffectorSetRPM(1000));

    // Climber in
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.b, "Climber In")
        .onTrue(
            Commands.runOnce(
                    () -> climber.setServoAngle(Constants.ClimberConstants.SERVO_UNLOCK_ANGLE))
                .andThen(Commands.waitSeconds(0.15))
                .andThen(
                    new ClimberSetAngle(climber, Constants.ClimberConstants.MIN_CLIMBER_ANGLE)));

    operatorController
        .registerTrigger(
            XboxControllerWrapper.Button.leftTrigger, "Manual arm and elevator override")
        .whileTrue(
            new ArmManualControl(operatorController::getRightX, arm)
                .alongWith(
                    new ElevatorManualControl(() -> -operatorController.getLeftY(), elevator)));

    operatorController
        .registerTrigger(XboxControllerWrapper.Button.b, "Climber in")
        .onTrue(new ClimberSetAngle(climber, Rotation2d.fromRotations(0)));

    // ---------- NON-CONTROLLER TRIGGERS

    RobotStates.triggerForCoralInEndEffector.onTrue(
        new WaitUntilMovedDist(drivetrainWrapper, Units.Meters.of(0.3))
            .andThen(
                MechanismActions.stowPosition(elevator, arm)
                    .finallyDo(
                        () -> {
                          RobotStates.changeEndEffectorIfNotAligning(
                              RobotStates.EndEffectorDesiredAction.AlignCoral);
                        }))
            .withName("GamepieceIntoEECommand"));

    RobotStates.triggerForCoralInEndEffector.onFalse(
        new WaitUntilMovedDist(drivetrainWrapper, Units.Meters.of(0.3))
            .andThen(MechanismActions.coralStationPosition(elevator, arm))
            .withName("GamepieceOutOfEECommand"));

    // ---------- ON-ROBOT CONTROLS ------------

    zeroSensorsButtonTrigger.onTrue(
        new RunsWhenDisabledInstantCommand(
            () -> {
              elevator.resetSensorToHomePosition();
              arm.resetSensorToHomePosition();
              intake.resetPivotSensorToHomePosition();
              led.setRobotState(RobotState.ZERO_SUBSYSTEMS);
              climber.resetWinchSensorToHomePosition();
            })); // TODO: add climber?

    brakeModeButtonTrigger.onTrue(
        new ConditionalCommand(
            new RunsWhenDisabledInstantCommand(
                () -> {
                  boolean armSuccess = arm.setNeutralMode(NeutralModeValue.Coast);
                  boolean elevatorSuccess = elevator.setNeutralMode(NeutralModeValue.Coast);
                  boolean intakePivotSuccess = intake.setPivotNeutralMode(NeutralModeValue.Coast);
                  boolean climberWinchSuccess = climber.setWinchNeutralMode(NeutralModeValue.Coast);

                  brakeModeFailure =
                      !armSuccess
                          || !elevatorSuccess
                          || !intakePivotSuccess
                          || !climberWinchSuccess;

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
                  boolean intakePivotSuccess = intake.setPivotNeutralMode(NeutralModeValue.Brake);
                  boolean climberWinchSuccess = climber.setWinchNeutralMode(NeutralModeValue.Brake);

                  brakeModeFailure =
                      !armSuccess
                          || !elevatorSuccess
                          || !intakePivotSuccess
                          || !climberWinchSuccess;

                  brakeModeTriggered = true;
                  led.setRobotState(
                      brakeModeFailure ? RobotState.BRAKE_MODE_FAILED : RobotState.BRAKE_MODE_ON);
                },
                elevator,
                arm),
            () -> brakeModeTriggered)); // TODO: add climber?

    // ---------- ELASTIC CONTROLS ------------

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

    SmartDashboard.putData("Confirm Auto", new RunsWhenDisabledInstantCommand(led::confirmAuto));

    SmartDashboard.putData(
        "Swerve Coast", new RunsWhenDisabledInstantCommand(() -> drivetrain.setBrakeMode(false)));
    SmartDashboard.putData(
        "Swerve Brake", new RunsWhenDisabledInstantCommand(() -> drivetrain.setBrakeMode(true)));

    SmartDashboard.putData(
        "WheelRadiusCharacterisation",
        new WheelRadiusCharacterization(
            drivetrainWrapper, Constants.RobotMode.getRobot().config.get()));

    SmartDashboard.putData(
        "Zero Mech",
        new RunsWhenDisabledInstantCommand(
            () -> {
              elevator.resetSensorToHomePosition();
              arm.resetSensorToHomePosition();
              intake.resetPivotSensorToHomePosition();
              led.setRobotState(RobotState.ZERO_SUBSYSTEMS);
            }));

    SmartDashboard.putData(
        "Zero Servo",
        new RunsWhenDisabledInstantCommand(
            () -> climber.setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE)));

    var endEffectorSim = endEffector.getSim();
    if (endEffectorSim != null) {
      SmartDashboard.putData(
          "SIM Coral in End Effector",
          new RunsWhenDisabledInstantCommand(
              () -> {
                endEffectorSim.scoringSideTofDetecting = true;
                endEffectorSim.nonScoringSideTofDetecting = true;
              }));
      SmartDashboard.putData(
          "SIM NO Coral in End Effector",
          new RunsWhenDisabledInstantCommand(
              () -> {
                endEffectorSim.scoringSideTofDetecting = true;
                endEffectorSim.nonScoringSideTofDetecting = true;
              }));
    }
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

      double distance = robotTranslation.getTranslation().getDistance(aprilTag);
      if (distance < minDistance) {
        minDistance = distance;
        nearestAprilTag = reefAprilTagPose[i];
      }
    }

    return nearestAprilTag;
  }

  /**
   * Finds the nearest coral station april tag position relative to the current robot translation
   *
   * @param robotTranslation - Current robot translation
   * @param coralStationPose - List of all coral station april tag positions
   * @return Nearest april tag
   */
  public static Pose2d findNearestCoralStation(Pose2d robotTranslation, Pose2d[] coralStationPose) {

    int coralStation1Index = Constants.isRedAlliance() ? 0 : 2;
    int coralStation2Index = Constants.isRedAlliance() ? 1 : 3;

    Pose2d coralStation1 = coralStationPose[coralStation1Index];
    Pose2d coralStation2 = coralStationPose[coralStation2Index];

    Pose2d nearestCoralStation =
        GeometryUtil.getDist(coralStation1, robotTranslation)
                < GeometryUtil.getDist(coralStation2, robotTranslation)
            ? coralStation1
            : coralStation2;

    return nearestCoralStation;
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

  public boolean autoFlipped() {
    return flipAutoChooser.get();
  }

  public void setPose(Pose2d pose) {
    drivetrain.setPose(pose);
    drivetrain.setCoralStationPoseToReefPose();
  }

  public void setCoralStationPoseToReefPose() {
    drivetrain.setCoralStationPoseToReefPose();
  }

  public void matchRawOdometryToPoseEstimatorValue() {
    drivetrain.setRawOdometryPose(drivetrain.getReefPoseEstimatorPose());
  }

  public void applyToDrivetrain() {
    drivetrainWrapper.apply();
  }

  public void enterDisabled() {
    resetSubsystems();
    vision.useMaxDistanceAwayFromExistingEstimate(false);
    vision.useGyroBasedFilteringForVision(false);

    arm.setVoltage(0);
    elevator.setPercentOut(0);
    climber.setWinchVoltage(0);
    intake.setPivotVoltage(0);
    intake.setRollerPercentOut(0);

    is_teleop = false;
    is_autonomous = false;
  }

  public void enterAutonomous() {
    // setBrakeMode();
    vision.useMaxDistanceAwayFromExistingEstimate(true);
    vision.useGyroBasedFilteringForVision(true);

    if (visionGamepiece != null) {
      visionGamepiece.setPipelineIndex(0);
    }

    is_teleop = false;
    is_autonomous = true;
  }

  public void enterTeleop() {
    // setBrakeMode();
    climber.setServoAngle(ClimberConstants.SERVO_UNLOCK_ANGLE);
    resetDrivetrainResetOverrides();
    vision.useMaxDistanceAwayFromExistingEstimate(true);
    vision.useGyroBasedFilteringForVision(true);

    led.setBaseRobotState(BaseRobotState.LEVEL_MODE);

    if (visionGamepiece != null) {
      visionGamepiece.setPipelineIndex(1);
    }

    is_teleop = true;
    is_autonomous = false;
  }

  public void resetDrivetrainResetOverrides() {
    drivetrainWrapper.resetVelocityOverride();
    drivetrainWrapper.resetRotationOverride();
  }

  public void updateVisualization() {
    MechanismVisualization.logMechanism();
    SimpleMechanismVisualization.updateVisualization(elevator.getHeight(), arm.getAngle());
    SimpleMechanismVisualization.logMechanism();
    MechanismVisualization.updateVisualization(
        drivetrainWrapper.getReefPoseEstimatorPose(true),
        elevator.getHeight(),
        arm.getAngle(),
        intake.getPivotAngle(),
        climber.getWinchAngle());
    MechanismVisualization.logMechanism();
    FieldStates.logGamepieceVisualization();
  }

  public void resetSubsystems() {}

  public void setBrakeMode() {
    brakeModeTriggered = true;
  }

  public void updateRobotState() {
    RobotStates.periodic();

    if (RobotStates.triggerForCoralInRobot.getAsBoolean() && !led.getGamepieceStatus()) {
      led.setGamepieceStatus(true);
    }

    if (!RobotStates.triggerForCoralInRobot.getAsBoolean() && led.getGamepieceStatus()) {
      led.setGamepieceStatus(false);
    }

    if (algaeInRobot.getAsBoolean()) {
      RobotStates.algaeInRobot = true;
    }

    if (intake.intakeTimeOfFlight()) {
      RobotStates.coralInIntake = true;
    }
  }

  public List<ScoringLocation> getCustomScoringLocations() {
    List<ScoringLocation> scoringLocations = new ArrayList<>();

    for (LoggedDashboardChooser<ReefSide> location : scoringPosChooser) {
      scoringLocations.add(
          new ScoringLocation(location.get(), ScoringLevel.L4)); // Assume L4 during auto for now
    }

    return scoringLocations;
  }

  public List<CoralStationLocation> getCustomCoralStationLocations() {
    List<CoralStationLocation> coralStationLocations = new ArrayList<>();

    for (LoggedDashboardChooser<CoralStationLocation> location : coralStationPosChooser) {
      coralStationLocations.add(location.get());
    }

    return coralStationLocations;
  }
}
