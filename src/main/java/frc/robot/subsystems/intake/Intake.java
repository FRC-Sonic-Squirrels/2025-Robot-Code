// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ControlMode;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotStates;

public class Intake extends SubsystemBase {
  // Execution timing
  private static final ExecutionTiming timing = new ExecutionTiming(IntakeConstants.ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(IntakeConstants.ROOT_TABLE);
  // Rollers
  private static final LoggerGroup rollerLogGroup = logGroup.subgroup(RollerConstants.ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_rollerVelocityRPM =
      rollerLogGroup.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal logInputs_rollerCurrentAmps =
      rollerLogGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_rollerTempCelsius =
      rollerLogGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_rollerAppliedVolts =
      rollerLogGroup.buildDecimal("AppliedVolts");

  private static final LoggerEntry.Decimal logRollerTargetVelocityRPM =
      rollerLogGroup.buildDecimal("TargetVelocityRPM");
  private static final LoggerEntry.EnumValue<ControlMode> logRollerControlMode =
      rollerLogGroup.buildEnum("ControlMode");

  // Pivot
  private static final LoggerGroup pivotLogGroup = logGroup.subgroup(PivotConstants.ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_pivotAngle =
      pivotLogGroup.buildDecimal("Angle");
  private static final LoggerEntry.Decimal logInputs_pivotAppliedVolts =
      pivotLogGroup.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_pivotCurrentAmps =
      pivotLogGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_pivotTempCelsius =
      pivotLogGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_pivotVelocityDegreesPerSecond =
      pivotLogGroup.buildDecimal("VelocityDegreesPerSecond");
  private static final LoggerEntry.EnumValue<ControlMode> logPivotControlMode =
      pivotLogGroup.buildEnum("ControlMode");
  private static final LoggerEntry.Decimal logPivotTargetAngleDegrees =
      pivotLogGroup.buildDecimal("TargetAngleDegrees");

  // ToF
  private static final LoggerGroup tofLogGroup = logGroup.subgroup("ToF");
  private static final LoggerEntry.Decimal logToFDistance = tofLogGroup.buildDecimal("Distance");
  private static final LoggerEntry.Bool logToFActivated = tofLogGroup.buildBoolean("Activated");
  private static final LoggerEntry.Decimal logToFSignalStrength =
      tofLogGroup.buildDecimal("SignalStrength");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(IntakeConstants.ROOT_TABLE);

  private static final LoggedTunableNumber intakingCoralVel =
      group.build("Intaking/Coral/Vel", 2000);
  private static final LoggedTunableNumber intakingAlgaeVel =
      group.build("Intaking/Algae/Vel", 2000);
  private static final LoggedTunableNumber intakingCoralPivotAngle =
      group.build("Intaking/Coral/AngleDeg", 0);
  private static final LoggedTunableNumber intakingAlgaePivotAngle =
      group.build("Intaking/Algae/AngleDeg", 20);
  private static final LoggedTunableNumber passOffPivotAngle =
      group.build("PassOff/AngleDeg", 92.5);
  private static final LoggedTunableNumber passOffVelocity = group.build("PassOff/Vel", -500);
  private static final LoggedTunableNumber stowPivotAngle = group.build("StowAngleDeg", 105);
  private static final LoggedTunableNumber algaeScoreVelocity =
      group.build("ScoreAlgae/Velocity", -1000.0);
  private static final LoggedTunableNumber algaeScoreAngle =
      group.build("ScoreAlgae/Angle", PivotConstants.ALGAE_SCORE_ANGLE.getDegrees());
  private static final LoggedTunableNumber climbAngle =
      group.build("ClimbAngle", PivotConstants.PIVOT_SAFE_ANGLE.getDegrees());

  // Roller
  private static final TunableNumberGroup rollerSubgroup =
      group.subgroup(RollerConstants.ROOT_TABLE);
  private static final LoggedTunableNumber rKS = rollerSubgroup.build("KS");
  private static final LoggedTunableNumber rKP = rollerSubgroup.build("KP");
  private static final LoggedTunableNumber rKV = rollerSubgroup.build("KV");
  private static final LoggedTunableNumber rollerTargetAccelerationConfig =
      rollerSubgroup.build("MaxAccelerationConstraint");
  private static final LoggedTunableNumber holdAlgaeVel =
      rollerSubgroup.build("HoldAlgaeVel", 1000);
  private static final LoggedTunableNumber holdCoralVel = rollerSubgroup.build("HoldCoralVel", 200);

  // Pivot
  private static final TunableNumberGroup pivotSubgroup = group.subgroup(PivotConstants.ROOT_TABLE);
  private static final LoggedTunableNumber pKP = pivotSubgroup.build("KP");
  private static final LoggedTunableNumber pKD = pivotSubgroup.build("KD");
  private static final LoggedTunableNumber pKG = pivotSubgroup.build("KG");

  private static final LoggedTunableNumber pivotMaxVelocityConfig =
      pivotSubgroup.build("MaxVelocityConfig");
  private static final LoggedTunableNumber pivotTargetAccelerationConfig =
      pivotSubgroup.build("TargetAccelerationConfig");
  private static final LoggedTunableNumber pivotToleranceDegrees =
      pivotSubgroup.build("ToleranceDegrees", 1);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
      rKS.initDefault(0);
      rKP.initDefault(0.8);
      rKV.initDefault(0.15);
      rollerTargetAccelerationConfig.initDefault(300.0);

      pKP.initDefault(70.0);
      pKD.initDefault(1.6);
      pKG.initDefault(0.0);

      pivotMaxVelocityConfig.initDefault(10);
      pivotTargetAccelerationConfig.initDefault(10);
    } else if (Constants.RobotMode.isSimBot()) {
      rKS.initDefault(0);
      rKP.initDefault(0.00006);
      rKV.initDefault(0.0002);
      rollerTargetAccelerationConfig.initDefault(0.0);

      pKP.initDefault(50);
      pKD.initDefault(0);
      pKG.initDefault(0.0);

      pivotMaxVelocityConfig.initDefault(40);
      pivotTargetAccelerationConfig.initDefault(80);
    } else {
      rKS.initDefault(0);
      rKP.initDefault(0.4);
      rKV.initDefault(0.13);
      rollerTargetAccelerationConfig.initDefault(200);

      pKP.initDefault(50);
      pKD.initDefault(0.3);
      pKG.initDefault(0.4);

      pivotMaxVelocityConfig.initDefault(200);
      pivotTargetAccelerationConfig.initDefault(200);
    }
  }

  private final IntakeIO io;
  private final IntakeIO.Inputs inputs = new IntakeIO.Inputs(logGroup);

  private double rollerTargetRPM;

  private ControlMode pivotControlMode = ControlMode.OPEN_LOOP;
  private Rotation2d pivotTargetAngle = PivotConstants.HOME_POSITION;

  private ControlMode rollerControlMode = ControlMode.OPEN_LOOP;

  public boolean holdAlgae;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    io.setPivotVoltage(0.0);

    setPivotConstants();

    io.setRollerVoltage(0.0);

    setRollerConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // Logging
      io.updateInputs(inputs);
      logInputs_rollerVelocityRPM.info(inputs.rollerVelocityRPM);
      logInputs_rollerCurrentAmps.info(inputs.rollerCurrentAmps);
      logInputs_rollerTempCelsius.info(inputs.rollerTempCelsius);
      logInputs_rollerAppliedVolts.info(inputs.rollerAppliedVolts);

      logRollerControlMode.info(rollerControlMode);

      logInputs_pivotAngle.info(inputs.pivotPosition);
      logInputs_pivotAppliedVolts.info(inputs.pivotAppliedVolts);
      logInputs_pivotCurrentAmps.info(inputs.pivotCurrentAmps);
      logInputs_pivotTempCelsius.info(inputs.pivotTempCelsius);
      logInputs_pivotVelocityDegreesPerSecond.info(inputs.pivotVelocityDegreesPerSecond);

      logPivotControlMode.info(pivotControlMode);

      logToFActivated.info(inputs.intakeTofDetected);
      logToFDistance.info(inputs.intakeTofDistanceInches);
      logToFSignalStrength.info(inputs.intakeTofSignalStrength);

      // Update tunable numbers

      var rhc = hashCode();
      if (rKS.hasChanged(rhc)
          || rKP.hasChanged(rhc)
          || rKV.hasChanged(rhc)
          || rollerTargetAccelerationConfig.hasChanged(rhc)) {
        setRollerConstants();
      }
      var phc = hashCode();
      if (pKP.hasChanged(phc)
          || pKD.hasChanged(phc)
          || pKG.hasChanged(phc)
          || pivotMaxVelocityConfig.hasChanged(phc)
          || pivotTargetAccelerationConfig.hasChanged(phc)) {
        setPivotConstants();
      }

      switch (RobotStates.intakeState) {
        case Override:
          break;
        case Idle:
          setPivotVoltage(0);
          setRollerPercentOut(0);
          break;
        case IntakeCoral:
          setRollerVelocity(intakingCoralVel.get());
          setPivotAngle(Rotation2d.fromDegrees(intakingCoralPivotAngle.get()));
          break;
        case IntakeAlgae:
          setRollerVelocity(intakingAlgaeVel.get());
          setPivotAngle(Rotation2d.fromDegrees(intakingAlgaePivotAngle.get()));
          break;
        case PrepPassoff:
          setRollerVelocity(holdCoralVel.get());
          setPivotAngle(Rotation2d.fromDegrees(passOffPivotAngle.get()));
          break;
        case Passoff:
          setRollerVelocity(passOffVelocity.get());
          setPivotAngle(Rotation2d.fromDegrees(passOffPivotAngle.get()));
          break;
        case Stow:
          if (holdAlgae) {
            setRollerVelocity(holdAlgaeVel.get());
          } else setRollerPercentOut(0);
          setPivotAngle(Rotation2d.fromDegrees(stowPivotAngle.get()));
          break;
        case ScoreAlgae:
          Rotation2d targetAngle = Rotation2d.fromDegrees(algaeScoreAngle.get());
          if (isPivotAtTargetAngle(targetAngle)) {
            setRollerVelocity(algaeScoreVelocity.get());
          } else setRollerPercentOut(0);
          setPivotAngle(targetAngle);
          break;
        case Climb:
          setPivotAngle(Rotation2d.fromDegrees(climbAngle.get()));
          setRollerPercentOut(0);
          break;
        default:
          break;
      }
    }
  }

  // Setters

  private void setRollerConstants() {
    io.setRollerClosedLoopConstants(
        rKP.get(), rKV.get(), rKS.get(), rollerTargetAccelerationConfig.get());
  }

  public void setRollerPercentOut(double percent) {
    io.setRollerVoltage(percent * Constants.MAX_VOLTAGE);
    rollerControlMode = ControlMode.OPEN_LOOP;
  }

  public void setRollerVelocity(double revPerMin) {
    rollerControlMode = ControlMode.CLOSED_LOOP;
    io.setRollerVelocity(revPerMin);
    rollerTargetRPM = revPerMin;
    logRollerTargetVelocityRPM.info(rollerTargetRPM);
  }

  private void setPivotConstants() {
    MotionMagicConfigs configs = new MotionMagicConfigs();
    configs.MotionMagicCruiseVelocity = pivotMaxVelocityConfig.get();
    configs.MotionMagicAcceleration = pivotTargetAccelerationConfig.get();
    io.setPivotClosedLoopConstants(pKP.get(), pKD.get(), pKG.get(), configs);
  }

  public void setPivotAngle(Rotation2d angle) {
    Rotation2d targetAngle =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                angle.getRadians(),
                PivotConstants.MIN_PIVOT_ANGLE.getRadians(),
                PivotConstants.MAX_PIVOT_ANGLE.getRadians()));

    pivotControlMode = ControlMode.CLOSED_LOOP;
    pivotTargetAngle = targetAngle;
    io.setPivotClosedLoopPosition(targetAngle);
    logPivotTargetAngleDegrees.info(targetAngle.getDegrees());
  }

  public void resetPivotSubsystem() {
    pivotControlMode = ControlMode.OPEN_LOOP;
    io.setPivotVoltage(0.0);
  }

  public void setPivotVoltage(double percent) {
    pivotControlMode = ControlMode.OPEN_LOOP;
    io.setPivotVoltage(percent);
  }

  public void resetPivotSensorToHomePosition() {
    io.resetPivotSensorPosition(PivotConstants.HOME_POSITION);
  }

  public boolean setPivotNeutralMode(NeutralModeValue value) {
    return io.setPivotNeutralMode(value);
  }

  public void setHoldAlgae(boolean holdAlgae) {
    this.holdAlgae = holdAlgae;
  }

  // Getters

  public Current getRollerCurrentDraw() {
    return Units.Amps.of(inputs.rollerCurrentAmps);
  }

  public AngularVelocity getRollerVelocity() {
    return Units.RPM.of(inputs.rollerVelocityRPM);
  }

  public Rotation2d getPivotAngle() {
    return inputs.pivotPosition;
  }

  public boolean isPivotAtTargetAngle() {
    return isPivotAtTargetAngle(pivotTargetAngle);
  }

  public boolean isPivotAtTargetAngle(Rotation2d target, Rotation2d tolerance) {
    var error = inputs.pivotPosition.minus(target).getRadians();
    return Math.abs(error) <= tolerance.getRadians();
  }

  public boolean isPivotAtTargetAngle(Rotation2d target) {
    return isPivotAtTargetAngle(target, Rotation2d.fromDegrees(pivotToleranceDegrees.get()));
  }

  public Voltage getPivotVoltage() {
    return Units.Volts.of(inputs.pivotAppliedVolts);
  }

  public AngularVelocity getPivotVelocity() {
    return Units.DegreesPerSecond.of(inputs.pivotVelocityDegreesPerSecond);
  }

  public boolean rollerStallDetected() {
    return inputs.rollerStallDetected;
  }

  public boolean intakeTimeOfFlight() {
    // TODO: get an actual value fot this this code is only for testing purposes
    return inputs.intakeTofDetected;
  }

  public Rotation2d getPassOffPivotAngle() {
    return Rotation2d.fromDegrees(passOffPivotAngle.get());
  }

  public IntakeIOSim getSim() {
    if (io instanceof IntakeIOSim) {
      return (IntakeIOSim) io;
    }

    return null;
  }
}
