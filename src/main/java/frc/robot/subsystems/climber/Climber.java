package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.RobotMode.RobotType;

public class Climber extends SubsystemBase {
  // Execution timing
  private static final ExecutionTiming winchTiming =
      new ExecutionTiming(ClimberConstants.WINCH_ROOT_TABLE);
  private static final ExecutionTiming grabberTiming =
      new ExecutionTiming(EndEffectorConstants.ROOT_TABLE);
  // Logging
  private static final LoggerGroup winchLogGroup =
      LoggerGroup.build(ClimberConstants.WINCH_ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_winchAngle =
      winchLogGroup.buildDecimal("AngleDegrees");
  private static final LoggerEntry.Decimal logInputs_winchAppliedVolts =
      winchLogGroup.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_winchCurrentAmps =
      winchLogGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_winchTempCelsius =
      winchLogGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_winchVelocityDegreesPerSecond =
      winchLogGroup.buildDecimal("VelocityDegreesPerSecond");
  private static final LoggerEntry.EnumValue<ControlMode> logWinchControlMode =
      winchLogGroup.buildEnum("ControlMode");
  private static final LoggerEntry.Decimal logTargetAngleDegrees =
      winchLogGroup.buildDecimal("targetAngleDegrees");

  private static final LoggerGroup grabberLogGroup =
      LoggerGroup.build(ClimberConstants.GRABBER_ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_grabberVelocityRPM =
      grabberLogGroup.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal logInputs_grabberCurrentAmps =
      grabberLogGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_grabberTempCelsius =
      grabberLogGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_grabberAppliedVolts =
      grabberLogGroup.buildDecimal("AppliedVolts");

  private static final LoggerEntry.Decimal logTargetVelocityRPM =
      grabberLogGroup.buildDecimal("TargetVelocityRPM");
  private static final LoggerEntry.EnumValue<ControlMode> logGrabberControlMode =
      grabberLogGroup.buildEnum("ControlMode");

  // Tunable Numbers
  private static final TunableNumberGroup winchGroup =
      new TunableNumberGroup(ClimberConstants.WINCH_ROOT_TABLE);

  private static final LoggedTunableNumber winchkP = winchGroup.build("kP");
  private static final LoggedTunableNumber winchkD = winchGroup.build("kD");
  private static final LoggedTunableNumber winchkG = winchGroup.build("kG");

  private static final LoggedTunableNumber maxVelocityConfig =
      winchGroup.build("MaxVelocityConfig");
  private static final LoggedTunableNumber winchTargetAccelerationConfig =
      winchGroup.build("TargetAccelerationConfig");
  private static final LoggedTunableNumber toleranceDegrees =
      winchGroup.build("ToleranceDegrees", 1);

  // Tunable numbers

  private static final TunableNumberGroup grabberGroup =
      new TunableNumberGroup(EndEffectorConstants.ROOT_TABLE);

  private static final LoggedTunableNumber grabberkS = grabberGroup.build("kS");
  private static final LoggedTunableNumber grabberkP = grabberGroup.build("kP");
  private static final LoggedTunableNumber grabberkV = grabberGroup.build("kV");
  private static final LoggedTunableNumber grabberTargetAccelerationConfig =
      grabberGroup.build("MaxAccelerationConstraint");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
      winchkP.initDefault(70.0);
      winchkD.initDefault(1.6);
      winchkG.initDefault(0.0);

      // FIXME: find the theoretical from the JVN docs
      maxVelocityConfig.initDefault(10);
      winchTargetAccelerationConfig.initDefault(10);

      grabberkS.initDefault(0);
      grabberkP.initDefault(0.8);
      grabberkV.initDefault(0.15);
      grabberTargetAccelerationConfig.initDefault(300.0);

    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {

      winchkP.initDefault(.02);
      winchkD.initDefault(0);
      winchkG.initDefault(0.0);

      maxVelocityConfig.initDefault(40);
      winchTargetAccelerationConfig.initDefault(80);

      grabberkS.initDefault(0);
      grabberkP.initDefault(0.0006);
      grabberkV.initDefault(0.0002);
      grabberTargetAccelerationConfig.initDefault(0.0);
    }
  }

  private final ClimberIO io;
  private final ClimberIO.Inputs winchInputs = new ClimberIO.Inputs(winchLogGroup);
  private final ClimberIO.Inputs grabberInputs = new ClimberIO.Inputs(grabberLogGroup);

  private ControlMode winchControlMode = ControlMode.OPEN_LOOP;
  private Rotation2d winchTargetAngleDegrees = Constants.zeroRotation2d;

  private double grabberTargetRPM;

  private ControlMode grabberControlMode = ControlMode.OPEN_LOOP;

  /** Creates a new ClimberSubsystem. */
  public Climber(ClimberIO io) {
    this.io = io;

    io.setWinchVoltage(0.0);

    setConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = winchTiming.start()) {
      // Winch logging
      io.updateWinchInputs(winchInputs);
      logInputs_winchAngle.info(winchInputs.winchPosition);
      logInputs_winchAppliedVolts.info(winchInputs.winchAppliedVolts);
      logInputs_winchCurrentAmps.info(winchInputs.winchCurrentAmps);
      logInputs_winchTempCelsius.info(winchInputs.winchTempCelsius);
      logInputs_winchVelocityDegreesPerSecond.info(winchInputs.winchVelocityDegreesPerSecond);

      logWinchControlMode.info(winchControlMode);

      // Updating tunable numbers
      var hc = hashCode();
      if (winchkP.hasChanged(hc)
          || winchkD.hasChanged(hc)
          || winchkG.hasChanged(hc)
          || maxVelocityConfig.hasChanged(hc)
          || winchTargetAccelerationConfig.hasChanged(hc)) {
        setConstants();
      }
    }
    try (var ignored = grabberTiming.start()) {
      // Logging
      io.updateGrabberInputs(grabberInputs);
      logInputs_grabberVelocityRPM.info(grabberInputs.grabberVelocityRPM);
      logInputs_grabberCurrentAmps.info(grabberInputs.grabberCurrentAmps);
      logInputs_grabberTempCelsius.info(grabberInputs.grabberTempCelsius);
      logInputs_grabberAppliedVolts.info(grabberInputs.grabberAppliedVolts);

      logGrabberControlMode.info(grabberControlMode);

      // Update tunable numbers

      var hc = hashCode();
      if (grabberkS.hasChanged(hc)
          || grabberkP.hasChanged(hc)
          || grabberkV.hasChanged(hc)
          || grabberTargetAccelerationConfig.hasChanged(hc)) {
        setConstants();
      }
    }
  }

  // setters

  private void setConstants() {
    MotionMagicConfigs configs = new MotionMagicConfigs();
    configs.MotionMagicCruiseVelocity = maxVelocityConfig.get();
    configs.MotionMagicAcceleration = winchTargetAccelerationConfig.get();
    io.setWinchClosedLoopConstants(winchkP.get(), winchkD.get(), winchkG.get(), configs);
    io.setGrabberClosedLoopConstants(
        grabberkP.get(), grabberkV.get(), grabberkS.get(), grabberTargetAccelerationConfig.get());
  }

  public void setWinchAngle(Rotation2d angle) {
    angle =
        Rotation2d.fromDegrees(
            MathUtil.clamp(
                angle.getDegrees(),
                Constants.ClimberConstants.MIN_CLIMBER_ANGLE.getDegrees(),
                Constants.ClimberConstants.MAX_CLIMBER_ANGLE.getDegrees()));

    winchControlMode = ControlMode.CLOSED_LOOP;
    winchTargetAngleDegrees = angle;
    io.setWinchClosedLoopPosition(angle);
    logTargetAngleDegrees.info(winchTargetAngleDegrees);
  }

  public void resetWinchSubsystem() {
    winchControlMode = ControlMode.OPEN_LOOP;
    io.setWinchVoltage(0);
  }

  public void setWinchVoltage(double percent) {
    winchControlMode = ControlMode.OPEN_LOOP;
    io.setWinchVoltage(percent);
  }

  public void resetWinchSensorToHomePosition() {
    io.resetWinchSensorPosition(Constants.ClimberConstants.MIN_CLIMBER_ANGLE);
  }

  public boolean setWinchNeutralMode(NeutralModeValue value) {
    return io.setWinchNeutralMode(value);
  }

  public void setServoAngle(Rotation2d angle) {
    io.setClimberServoAngle(angle);
  }

  public void setGrabberPercentOut(double percent) {
    io.setGrabberVoltage(percent * Constants.MAX_VOLTAGE);
    grabberControlMode = ControlMode.OPEN_LOOP;
  }

  public void setGrabberVelocity(double revPerMin) {
    io.setGrabberVelocity(revPerMin);
    grabberTargetRPM = revPerMin;
    logTargetVelocityRPM.info(grabberTargetRPM);
    grabberControlMode = ControlMode.CLOSED_LOOP;
  }

  public void resetGrabberSubsystem() {
    grabberControlMode = ControlMode.OPEN_LOOP;
    io.setGrabberVoltage(0);
  }

  public boolean setGrabberNeutralMode(NeutralModeValue value) {
    return io.setGrabberNeutralMode(value);
  }

  // Winch getters

  public Rotation2d getWinchAngle() {
    return Rotation2d.fromDegrees(winchInputs.winchPosition);
  }

  public boolean isWinchAtTargetAngle() {
    return isWinchAtTargetAngle(winchTargetAngleDegrees);
  }

  public boolean isWinchAtTargetAngle(Rotation2d target, Rotation2d tolerance) {
    var error = winchInputs.winchPosition - target.getDegrees();
    return Math.abs(error) <= tolerance.getRadians();
  }

  public boolean isWinchAtTargetAngle(Rotation2d target) {
    return isWinchAtTargetAngle(target, Rotation2d.fromDegrees(toleranceDegrees.get()));
  }

  public Voltage getWinchVoltage() {
    return Units.Volts.of(winchInputs.winchAppliedVolts);
  }

  public AngularVelocity getWinchVelocity() {
    return Units.DegreesPerSecond.of(winchInputs.winchVelocityDegreesPerSecond);
  }

  // Grabber getters
  public Current getGrabberCurrentDraw() {
    return Units.Amps.of(grabberInputs.grabberCurrentAmps);
  }

  public AngularVelocity getVelocity() {
    return Units.RPM.of(grabberInputs.grabberVelocityRPM);
  }

  public Voltage getGrabberVoltage() {
    return Units.Volts.of(winchInputs.grabberAppliedVolts);
  }
}
