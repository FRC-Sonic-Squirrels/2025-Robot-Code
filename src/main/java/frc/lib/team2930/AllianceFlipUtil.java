package frc.lib.team2930;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class AllianceFlipUtil {

  public static Translation2d flipVelocitiesForAlliance(Translation2d originalVelocity) {
    return Constants.isRedAlliance() ? originalVelocity.unaryMinus() : originalVelocity;
  }

  public static Translation2d rotateTranslation2DAroundCenterPoint(
      Translation2d originalTranslation) {
    return new Translation2d(
        Constants.FieldConstants.FIELD_LENGTH.in(Units.Meters) - originalTranslation.getX(),
        Constants.FieldConstants.FIELD_WIDTH.in(Units.Meters) - originalTranslation.getY());
  }

  public static Pose2d rotatePose2DAroundCenterPoint(Pose2d originalPose) {
    return new Pose2d(
        rotateTranslation2DAroundCenterPoint(originalPose.getTranslation()),
        originalPose.getRotation().plus(Rotation2d.k180deg));
  }

  /**
   * @return blue alliance reference pose
   */
  public static Pose2d flipPoseForAlliance(Pose2d originalPose) {
    return Constants.isRedAlliance() ? rotatePose2DAroundCenterPoint(originalPose) : originalPose;
  }

  /**
   * @return blue alliance reference translation
   */
  public static Translation2d flipTranslationForAlliance(Translation2d originalTranslation) {
    return Constants.isRedAlliance()
        ? rotateTranslation2DAroundCenterPoint(originalTranslation)
        : originalTranslation;
  }
}
