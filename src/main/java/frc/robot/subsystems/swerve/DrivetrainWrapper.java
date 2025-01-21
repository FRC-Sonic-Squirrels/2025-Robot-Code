package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import java.util.function.DoubleSupplier;

public class DrivetrainWrapper {

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build("DrivetrainWrapper");
  private static final LoggerEntry.Struct<ChassisSpeeds> logChassisSpeedsBase =
      logGroup.buildStruct(ChassisSpeeds.class, "chassisSpeedsBase");
  private static final LoggerEntry.Struct<ChassisSpeeds> logChassisSpeedsOverride =
      logGroup.buildStruct(ChassisSpeeds.class, "chassisSpeedsOverride");
  private static final LoggerEntry.Decimal logOmegaOverride =
      logGroup.buildDecimal("omegaOverride");
  private static final LoggerEntry.Decimal logGyroDrift = logGroup.buildDecimal("gyroDrift");

  private final Drivetrain drivetrain;
  private final DoubleSupplier baseSpeedScalar;

  private ChassisSpeeds chassisSpeedsBase = new ChassisSpeeds();
  private ChassisSpeeds chassisSpeedsOverride;
  private double omegaOverride = Double.NaN;

  public DrivetrainWrapper(Drivetrain drivetrain, DoubleSupplier baseSpeedScalar) {
    this.drivetrain = drivetrain;
    this.baseSpeedScalar = baseSpeedScalar;
  }

  /** Updated periodically, updates swerve state. */
  public void apply() {
    if (chassisSpeedsBase != null) {
      logChassisSpeedsBase.info(chassisSpeedsBase);
    }
    if (chassisSpeedsOverride != null) {
      logChassisSpeedsOverride.info(chassisSpeedsOverride);
    }
    logOmegaOverride.info(omegaOverride);

    ChassisSpeeds chassisSpeeds = chassisSpeedsBase.times(baseSpeedScalar.getAsDouble());

    boolean prioritizeRotation;

    if (Double.isFinite(omegaOverride)) {
      prioritizeRotation = true;
      chassisSpeeds =
          new ChassisSpeeds(
              chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, omegaOverride);
    } else {
      prioritizeRotation = false;
    }
    if (chassisSpeedsOverride != null) {
      chassisSpeeds = chassisSpeedsOverride;
    }

    drivetrain.runVelocity(chassisSpeeds, prioritizeRotation);

    var pose1 = getPoseEstimatorPose(false);
    var pose2 = getPoseEstimatorPose(true);

    logGyroDrift.info(pose1.getRotation().minus(pose2.getRotation()));
  }

  // Setters

  /**
   * Sets robot-centric chassis speeds
   *
   * @param chassisSpeeds speeds (m/s, rad/s)
   */
  public void setVelocity(ChassisSpeeds chassisSpeeds) {
    if (chassisSpeeds == null) chassisSpeeds = new ChassisSpeeds();
    chassisSpeedsBase = chassisSpeeds;
  }

  public void setVelocityOverride(ChassisSpeeds chassisSpeeds) {
    chassisSpeedsOverride = chassisSpeeds;
  }

  public void resetVelocityOverride() {
    chassisSpeedsOverride = null;
  }

  public void setRotationOverride(double omega) {
    omegaOverride = omega;
  }

  public void resetRotationOverride() {
    omegaOverride = Double.NaN;
  }

  public SysIdRoutine.Mechanism getSysIdMechanism() {
    return new SysIdRoutine.Mechanism(
        (voltage) -> drivetrain.runCharacterizationVolts(voltage.in(Units.Volts)),
        null, // No log consumer, since data is recorded by AdvantageKit
        drivetrain);
  }

  public void setPose(Pose2d pose) {
    drivetrain.setPose(pose);
  }

  // Getters

  public Subsystem getRequirements() {
    return drivetrain;
  }

  public Pose2d getPoseEstimatorPose(boolean prioritizeGyro) {
    var pose = drivetrain.getPoseEstimatorPose();
    if (prioritizeGyro) {
      pose = new Pose2d(pose.getTranslation(), getRotationGyroOnly());
    }

    return pose;
  }

  public double getVisionStaleness() {
    return drivetrain.getVisionStaleness();
  }

  public LinearVelocity getMaxLinearSpeed() {
    return Units.MetersPerSecond.of(drivetrain.getMaxLinearSpeedMetersPerSec());
  }

  public AngularVelocity getMaxAngularSpeed() {
    return Units.RadiansPerSecond.of(drivetrain.getMaxAngularSpeedRadPerSec());
  }

  public Pose2d getFieldRelativeVelocities() {
    return drivetrain.getFieldRelativeVelocities();
  }

  public Pose2d getRobotCentricVelocities() {
    return drivetrain.getRobotCentricVelocities();
  }

  public Rotation2d getRotationGyroOnly() {
    return drivetrain.getRotationGyroOnly();
  }

  public Angle[] getModuleRotations() {
    return drivetrain.getModuleDriveRotations();
  }
}
