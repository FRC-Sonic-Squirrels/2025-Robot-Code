package frc.robot.autonomous.substates;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autonomous.AutosSubsystems;
import frc.robot.autonomous.ChoreoTrajectoryWithName;
import frc.robot.autonomous.DriveToGamepieceHelper;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.visionGamepiece.ProcessedGamepieceData;
import java.util.function.Supplier;

public class AutoSubstateMachineDriveTranslation extends AutoSubstateMachine {
  private final Translation2d gamepieceTranslation;

  // private final RotateToAngle rotateToAngle;

  /** Creates a new AutoSubstateMachineDriveTranslation. */
  public AutoSubstateMachineDriveTranslation(
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean useVision,
      boolean ploppedGamepeice,
      Translation2d gamepieceTranslation,
      ChoreoTrajectoryWithName trajToShoot,
      Supplier<ProcessedGamepieceData> closestGamepiece) {
    super(
        "AutoSub " + ChoreoTrajectoryWithName.getName(trajToShoot),
        subsystems,
        config,
        useVision,
        ploppedGamepeice,
        trajToShoot,
        closestGamepiece,
        gamepieceTranslation);

    this.gamepieceTranslation = gamepieceTranslation;
    // this.rotateToAngle = new RotateToAngle(drive,
    // DriverStation.getAlliance().get().equals(Alliance.Blue), drive.getPoseEstimatorPose(true))

    setInitialState(stateWithName("initDriveToGamepiece", this::initDriveToGamepiece));
  }

  private StateHandler initDriveToGamepiece() {
    intakeCommand = new IntakeGamepiece(null, null, null, null);
    intakeCommand.schedule();

    driveToGamepieceHelper =
        new DriveToGamepieceHelper(
            drive.getPoseEstimatorPose(true), drive.getFieldRelativeVelocities());

    return stateWithName("pickupGamepiece", this::pickupGamepiece);
  }

  private StateHandler pickupGamepiece() {
    // led.setBaseRobotState(BaseRobotState.AUTO_DRIVE_TO_POSE);
    ChassisSpeeds speeds =
        driveToGamepieceHelper.calculateChassisSpeeds(
            gamepieceTranslation, drive.getPoseEstimatorPose(true));

    if (useVisionForGamepiece()) {
      led.setBaseRobotState(BaseRobotState.GAMEPIECE_STATUS);
      return stateWithName("visionPickupGamepiece", super::visionPickupGamepiece);
    }

    // TODO: change to if no gamepiece is posessed
    if (true) {
      drive.setVelocityOverride(speeds);
      if (driveToGamepieceHelper.isAtTarget()) {
        drive.resetVelocityOverride();
        led.setBaseRobotState(BaseRobotState.GAMEPIECE_STATUS);
        return super::gamepieceConfirmation;
      }
      return null;
    }
    drive.resetVelocityOverride();
    led.setBaseRobotState(BaseRobotState.GAMEPIECE_STATUS);
    return stateWithName("prepFollowPathToShooting", super::prepFollowPathToShooting);
  }

  // private StateHandler rotateTowardOpponentWall(){

  // }
}
