// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ControlMode;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.RobotMode.RobotType;

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
  private boolean gamepieceInRobot = false;

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

  private double targetRPM;

  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Creates a new EndEffector. */
  public EndEffector(EndEffectorIO io) {
    this.io = io;

    setConstants();

    io.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // Logging
      io.updateInputs(inputs);
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

      // Update tunable numbers

      var hc = hashCode();
      if (kS.hasChanged(hc)
          || kP.hasChanged(hc)
          || kV.hasChanged(hc)
          || targetAccelerationConfig.hasChanged(hc)) {
        setConstants();
      }
    }
  }

  // Setters

  public void setGamepieceInRobot(boolean value) {
    gamepieceInRobot = value;
  }

  private void setConstants() {
    io.setClosedLoopConstants(kP.get(), kV.get(), kS.get(), targetAccelerationConfig.get());
  }

  public void setPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE);
    controlMode = ControlMode.OPEN_LOOP;
  }

  public void setVelocity(double revPerMin) {
    io.setVelocity(revPerMin);
    targetRPM = revPerMin;
    logTargetVelocityRPM.info(targetRPM);
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

  public boolean isGamepieceFullyInEndEffector() {
    return gamepieceInRobot;
  }

  public double getMotorPosition() {
    return inputs.position;
  }
}
