// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotStates;

public class EndEffector extends SubsystemBase {
  // Execution timing
  private static final ExecutionTiming timing =
      new ExecutionTiming(EndEffectorConstants.ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(EndEffectorConstants.ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_velocityRPM =
      logGroup.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal logInputs_currentAmps =
      logGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_tempCelsius =
      logGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_appliedVolts =
      logGroup.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_scoringSideTofDist =
      logGroup.buildDecimal("ToF/ScoringSideTofDist");
  private static final LoggerEntry.Decimal logInputs_nonScoringSideTOFDist =
      logGroup.buildDecimal("ToF/NonScoringSideTOFDist");
  private static final LoggerEntry.Bool logInputs_scoringSideTofActivated =
      logGroup.buildBoolean("ToF/ScoringSideTofActivated");
  private static final LoggerEntry.Bool logInputs_nonScoringSideTOFActivated =
      logGroup.buildBoolean("ToF/NonScoringSideTOFActivated");
  private static final LoggerEntry.Decimal logInputs_scoringSideTofSignalStrength =
      logGroup.buildDecimal("ToF/ScoringSideTofSignalStrength");
  private static final LoggerEntry.Decimal logInputs_nonScoringSideTOFSignalStrength =
      logGroup.buildDecimal("ToF/NonScoringSideTOFSignalStrength");

  private static final LoggerEntry.Decimal logTargetVelocityRPM =
      logGroup.buildDecimal("TargetVelocityRPM");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");
  private static final LoggerEntry.Decimal logVelocityOverride =
      logGroup.buildDecimal("VelocityOverride");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(EndEffectorConstants.ROOT_TABLE);

  private static final LoggedTunableNumber distanceToTriggerCoralDetection =
      group.build("distanceToTriggerCoral", 11.0);

  private static final LoggedTunableNumber kS = group.build("kS");
  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kV = group.build("kV");
  private static final LoggedTunableNumber targetAccelerationConfig =
      group.build("MaxAccelerationConstraint");

  private static final LoggedTunableNumber intakingVelocitySlow =
      group.build("intakingVelocitySlow", 800);
  private static final LoggedTunableNumber intakingVelocityHigh =
      group.build("intakingVelocity", 2500);

  private static final LoggedTunableNumber scoringVelocityRPM =
      group.build("ScoringVelocityRPM", -3000);
  private static final LoggedTunableNumber passOffVelocityRPM =
      group.build("passOffVelocityRPM", -500);

  private static final LoggedTunableNumber correctionVelocity = group.build("alignVelocity", 500);
  private static final LoggedTunableNumber alignTarget = group.build("alignTarget", 1);
  private static final LoggedTunableNumber alignTolerance = group.build("alignTolerance", 2);
  private static final LoggedTunableNumber alignL1Turns = group.build("alignL1Turns", 5);
  private static final LoggedTunableNumber alignL2Turns = group.build("alignL2Turns", 1);
  private static final LoggedTunableNumber alignL3Turns = group.build("alignL3Turns", 12);
  private static final LoggedTunableNumber alignL4Turns = group.build("alignL4Turns", 12);

  // -- //

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
      kS.initDefault(0);
      kP.initDefault(0.4);
      kV.initDefault(0.15);
      targetAccelerationConfig.initDefault(300.0);
    } else if (Constants.RobotMode.isSimBot()) {
      kS.initDefault(0);
      kP.initDefault(0.0006);
      kV.initDefault(0.0002);
      targetAccelerationConfig.initDefault(0.0);
    } else {
      kS.initDefault(0);
      kP.initDefault(0.2);
      kV.initDefault(0.115);
      targetAccelerationConfig.initDefault(400);
    }
  }

  private final EndEffectorIO io;
  private final EndEffectorIO.Inputs inputs = new EndEffectorIO.Inputs(logGroup);

  private ControlMode controlMode = ControlMode.OPEN_LOOP;
  private double zeroCoralPosition;

  /** Creates a new EndEffector. */
  public EndEffector(EndEffectorIO io) {
    this.io = io;

    setConstants();

    io.setVoltage(0.0);
  }

  public EndEffectorIOSim getSim() {
    if (io instanceof EndEffectorIOSim) {
      return (EndEffectorIOSim) io;
    }

    return null;
  }

  private int counter = 0;
  private LoggerEntry.Integer counterLog = logGroup.buildInteger("counter");

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      counter++;
      counterLog.info(counter % 100);
      // Logging
      io.updateInputs(inputs);

      var coralInEndEffectorScoringSide = scoringSideTofSeenGamepiece();
      var coralInEndEffectorNonScoringSide = nonScoringSideTOFSeenGamepiece();
      var coralInEndEffector = coralInEndEffectorScoringSide || coralInEndEffectorNonScoringSide;

      RobotStates.coralInEndEffectorScoringSide = coralInEndEffectorScoringSide;
      RobotStates.coralInEndEffectorNonScoringSide = coralInEndEffectorNonScoringSide;
      RobotStates.coralInEndEffector = coralInEndEffector;

      logInputs_velocityRPM.info(inputs.velocityRPM);
      logInputs_currentAmps.info(inputs.currentAmps);
      logInputs_tempCelsius.info(inputs.tempCelsius);
      logInputs_appliedVolts.info(inputs.appliedVolts);
      logInputs_scoringSideTofDist.info(inputs.scoringSideTofDistInches);
      logInputs_nonScoringSideTOFDist.info(inputs.nonScoringSideTofDistInches);
      logInputs_scoringSideTofActivated.info(inputs.scoringSideTofDetecting);
      logInputs_nonScoringSideTOFActivated.info(inputs.nonScoringSideTofDetecting);
      logInputs_scoringSideTofSignalStrength.info(inputs.scoringSideSignalStrength);
      logInputs_nonScoringSideTOFSignalStrength.info(inputs.nonScoringSideSignalStrength);

      logControlMode.info(controlMode);

      logVelocityOverride.info(RobotStates.endEffectorOverrideVelocity);

      // Update tunable numbers

      var hc = hashCode();
      if (kS.hasChanged(hc)
          || kP.hasChanged(hc)
          || kV.hasChanged(hc)
          || targetAccelerationConfig.hasChanged(hc)) {
        setConstants();
      }

      var desiredAction = RobotStates.endEffectorDesiredAction;

      if (DriverStation.isDisabled()) {
        if (desiredAction != RobotStates.EndEffectorDesiredAction.AlignedCoral) {
          desiredAction = RobotStates.EndEffectorDesiredAction.Idle;
        }
        RobotStates.endEffectorOverrideVelocity = Double.NaN;
      }

      if (Double.isFinite(RobotStates.endEffectorOverrideVelocity)) {
        setVelocity(RobotStates.endEffectorOverrideVelocity);
      } else {
        var endEffectorSim = getSim();
        while (true) {
          switch (desiredAction) {
            case Idle:
              if (coralInEndEffectorScoringSide && coralInEndEffectorNonScoringSide) {
                desiredAction = RobotStates.EndEffectorDesiredAction.AlignCoral;
              } else {
                setPercentOut(0);
              }
              break;

            case CoralStationIntake:
              if (coralInEndEffectorNonScoringSide) {
                desiredAction = RobotStates.EndEffectorDesiredAction.AlignCoral;
              } else if (coralInEndEffectorScoringSide) {
                setVelocity(intakingVelocitySlow.get());
              } else {
                setVelocity(intakingVelocityHigh.get());
              }
              break;

            case GroundIntake:
              if (coralInEndEffectorScoringSide) {
                desiredAction = RobotStates.EndEffectorDesiredAction.AlignCoral;
              } else if (coralInEndEffectorNonScoringSide) {
                setVelocity(-(intakingVelocitySlow.get()));
              } else {
                setVelocity(-(intakingVelocityHigh.get()));
              }
              break;

            case AlignCoral:
              if (!coralInEndEffector) {
                desiredAction = RobotStates.EndEffectorDesiredAction.Idle;
              } else if (!coralInEndEffectorNonScoringSide) {
                // Keep moving the coral in.
                setVelocity(correctionVelocity.get());
              } else {
                // Start backtracking the coral.
                setVelocity(-correctionVelocity.get());
                desiredAction = RobotStates.EndEffectorDesiredAction.AlignCoralPhase2;
              }
              break;

            case AlignCoralPhase2:
              if (!coralInEndEffectorNonScoringSide) {
                // Now reverse until we see it again.
                setVelocity(correctionVelocity.get());

                desiredAction = RobotStates.EndEffectorDesiredAction.AlignCoralPhase3;
              }
              break;

            case AlignCoralPhase3:
              if (coralInEndEffectorNonScoringSide) {
                zeroCoralPosition = getMotorPosition();

                desiredAction = RobotStates.EndEffectorDesiredAction.AlignCoralPhase4;
              }
              break;

            case AlignCoralPhase4:
              double diff = Math.abs(getMotorPosition() - zeroCoralPosition);
              //          log_Position.info(dif);
              if (diff >= alignTarget.get()) {
                zeroCoralPosition = getMotorPosition();
                desiredAction = RobotStates.EndEffectorDesiredAction.AlignedCoral;
              }
              break;

            case AlignedCoral:
              double pos = getMotorPosition() - zeroCoralPosition;
              double desiredPos = 0;
              switch (RobotStates.scoringLevel) {
                case L1:
                  desiredPos = alignL1Turns.get();
                  break;
                case L2:
                  desiredPos = alignL2Turns.get();
                  break;
                case L3:
                  desiredPos = alignL3Turns.get();
                  break;
                case L4:
                  desiredPos = alignL4Turns.get();
                  break;
                default:
                  setPercentOut(0);
                  break;
              }
              if (Math.abs(pos - desiredPos) < alignTolerance.get()) {
                setPercentOut(0);
              } else if (pos > desiredPos) {
                setVelocity(-correctionVelocity.get());
              } else if (pos < desiredPos) {
                setVelocity(correctionVelocity.get());
              }
              break;

            case ScoreFastForward:
              setVelocity(scoringVelocityRPM.get());
              if (!coralInEndEffector) {
                desiredAction = RobotStates.EndEffectorDesiredAction.Idle;
              }

              if (endEffectorSim != null) {
                endEffectorSim.scoringSideTofDetecting = false;
                endEffectorSim.nonScoringSideTofDetecting = false;
              }
              break;

            case ScoreFastBackward:
              setVelocity(-scoringVelocityRPM.get());
              if (!coralInEndEffector) {
                desiredAction = RobotStates.EndEffectorDesiredAction.Idle;
              }

              if (endEffectorSim != null) {
                endEffectorSim.scoringSideTofDetecting = false;
                endEffectorSim.nonScoringSideTofDetecting = false;
              }
              break;
            case PassToEndEffector:
              setVelocity(passOffVelocityRPM.get());
              break;
            case PassToIntake:
              setVelocity(-passOffVelocityRPM.get());
              break;
          }
          if (RobotStates.endEffectorDesiredAction == desiredAction) {
            break;
          }
          RobotStates.endEffectorDesiredAction = desiredAction;
        }
      }

      RobotStates.endEffectorDesiredAction = desiredAction;
    }
  }

  // Setters

  private void setConstants() {
    io.setClosedLoopConstants(kP.get(), kV.get(), kS.get(), targetAccelerationConfig.get());
  }

  private void setPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE);
    controlMode = ControlMode.OPEN_LOOP;
  }

  private void setVelocity(double revPerMin) {
    io.setVelocity(revPerMin);
    logTargetVelocityRPM.info(revPerMin);
    controlMode = ControlMode.CLOSED_LOOP;
  }

  // Getters
  public Current getCurrentDraw() {
    return Units.Amps.of(inputs.currentAmps);
  }

  public AngularVelocity getVelocity() {
    return Units.RPM.of(inputs.velocityRPM);
  }

  public Distance scoringSideTofDistance() {
    return Units.Inches.of(inputs.scoringSideTofDistInches);
  }

  public Distance nonScoringSideTofDistance() {
    return Units.Inches.of(inputs.nonScoringSideTofDistInches);
  }

  public boolean scoringSideTofSeenGamepiece() {
    return inputs.scoringSideTofDetecting;
  }

  public boolean nonScoringSideTOFSeenGamepiece() {
    return inputs.nonScoringSideTofDetecting;
  }

  public double getMotorPosition() {
    return inputs.position;
  }
}
