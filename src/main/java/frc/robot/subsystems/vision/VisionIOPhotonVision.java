package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;

  private double lastTimestampCTRETime = -1;
  private PhotonPipelineResult lastResult = new PhotonPipelineResult();

  public double medianUpdateTime = 0.0;
  MedianFilter updateTimeMedianFilter = new MedianFilter(10);

  public VisionIOPhotonVision(String cameraName) {
    camera = new PhotonCamera(cameraName);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    camera.setDriverMode(false);
    camera.setLED(VisionLEDMode.kOff);

    /*
       * based on
    https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-change.html#listening-for-changes
       * and
    https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java
       */
    StructSubscriber targetPoseSub =
        inst.getTable("/photonvision/" + cameraName)
            .getStructTopic("targetPose", Transform3d.struct)
            .subscribe(null);

    inst.addListener(
        targetPoseSub,
        java.util.EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            for (var target : result.getTargets()) {
              if (target.altCameraToTarget.getTranslation().getNorm() < 0.01) {
                 target.altCameraToTarget = target.bestCameraToTarget;
                // Reject results with no valid target translation.
              }
            }

            var timestamp = result.getTimestampSeconds();
            var fpga = Timer.getFPGATimestamp();
            var ctre = Utils.getCurrentTimeSeconds();

            // we use CTRE time here because drivetrain odometry uses CTRE time NOT FPGA
            timestamp -= fpga;
            timestamp += ctre;

            synchronized (VisionIOPhotonVision.this) {
              updateTimeMedian(timestamp - lastTimestampCTRETime);

              lastTimestampCTRETime = timestamp;
              lastResult = result;
            }
          }
        });
  }

  @Override
  public synchronized void updateInputs(Inputs inputs) {
    inputs.lastTimestampCTRETime = this.lastTimestampCTRETime;
    inputs.lastResult = this.lastResult;
    inputs.connected = camera.isConnected();
    inputs.medianUpdateTime = medianUpdateTime;
  }

  @Override
  public PhotonCamera getCamera() {
    return camera;
  }

  public void updateTimeMedian(double timeSinceLastUpdate) {
    medianUpdateTime = updateTimeMedianFilter.calculate(timeSinceLastUpdate);
  }
}
