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
import frc.robot.Constants;
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
    // Log chassis speeds
    if (chassisSpeedsBase != null) {
      logChassisSpeedsBase.info(chassisSpeedsBase);
    }
    if (chassisSpeedsOverride != null) {
      logChassisSpeedsOverride.info(chassisSpeedsOverride);
    }
    logOmegaOverride.info(omegaOverride);

    ChassisSpeeds chassisSpeeds = chassisSpeedsBase.times(baseSpeedScalar.getAsDouble());

    boolean prioritizeRotation;

    // If omega override, use that for omega
    if (Double.isFinite(omegaOverride)) {
      prioritizeRotation = true;
      chassisSpeeds =
          new ChassisSpeeds(
              chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, omegaOverride);
    } else {
      prioritizeRotation = false;
    }

    // If chassisSpeedsOverride, replace chassisSpeeds with that

    if (chassisSpeedsOverride != null) {
      chassisSpeeds = chassisSpeedsOverride;
    }

    // Run drivetrain

    drivetrain.runVelocity(chassisSpeeds, prioritizeRotation, chassisSpeedsOverride != null);

    var pose1 = getReefPoseEstimatorPose(false);
    var pose2 = getReefPoseEstimatorPose(true);

    logGyroDrift.info(
        pose1
            .getRotation()
            .minus(pose2.getRotation())); // Dif between strictly gyro estimate and vision assisted
    // estimate
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

  /**
   * Sets robot-centric chassis speeds
   *
   * <p>Overrides 'setVelocity' and 'setRotationOverride'
   *
   * @param chassisSpeeds speeds (m/s, rad/s)
   */
  public void setVelocityOverride(ChassisSpeeds chassisSpeeds) {
    chassisSpeedsOverride = chassisSpeeds;
  }

  /** Gives velocity control back to base */
  public void resetVelocityOverride() {
    chassisSpeedsOverride = null;
  }

  /**
   * Sets robot rotational vel
   *
   * <p>Overrides 'setVelocity'
   *
   * @param omega
   */
  public void setRotationOverride(double omega) {
    omegaOverride = omega;
  }

  /** Gives rotational vel control back to base */
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

  public Pose2d getReefPoseEstimatorPose(boolean prioritizeGyro) {
    var pose = drivetrain.getReefPoseEstimatorPose();
    if (Constants.unusedCode && prioritizeGyro) {
      pose = new Pose2d(pose.getTranslation(), getRotationGyroOnly());
    }

    return pose;
  }

  public Pose2d getCoralStationPoseEstimatorPose(boolean prioritizeGyro) {
    var pose = drivetrain.getCoralStationPoseEstimatorPose();
    if (Constants.unusedCode && prioritizeGyro) {
      pose = new Pose2d(pose.getTranslation(), getRotationGyroOnly());
    }

    return pose;
  }

  public double getVisionStaleness() {
    return drivetrain.getReefVisionStaleness();
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

  public ChassisSpeeds getCurrentRobotRelativeChassisSpeeds() {
    return drivetrain.getChassisSpeeds();
  }

  public Pose2d getRawOdometryPose() {
    return drivetrain.getRawOdometryPose();
  }

  public double getLinearVel() {
    return drivetrain.getLinearVel();
  }
}
