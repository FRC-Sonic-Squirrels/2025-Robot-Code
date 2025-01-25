package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotMode.RobotType;

public class Climber extends SubsystemBase {
    // Execution timing
  private static final ExecutionTiming timing = new ExecutionTiming(ClimberConstants.ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(ClimberConstants.ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_angle = logGroup.buildDecimal("AngleDegrees");
  private static final LoggerEntry.Decimal logInputs_appliedVolts =
      logGroup.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_currentAmps =
      logGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_tempCelsius =
      logGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_velocityDegreesPerSecond =
      logGroup.buildDecimal("VelocityDegreesPerSecond");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");
  private static final LoggerEntry.Decimal logTargetAngleDegrees =
      logGroup.buildDecimal("targetAngleDegrees");

  // Tunable Numbers
  private static final TunableNumberGroup group = new TunableNumberGroup(ClimberConstants.ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kD = group.build("kD");
  private static final LoggedTunableNumber kG = group.build("kG");

  private static final LoggedTunableNumber maxVelocityConfig = group.build("MaxVelocityConfig");
  private static final LoggedTunableNumber targetAccelerationConfig =
      group.build("TargetAccelerationConfig");
  private static final LoggedTunableNumber toleranceDegrees = group.build("ToleranceDegrees", 1);

  

}
