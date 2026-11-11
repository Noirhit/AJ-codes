package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;

public class FuelTrayCameraSubsystem extends SubsystemBase {
  private final PhotonCamera fuelTrayCamera;
  private boolean jamDetected = false;
  private int stationaryFrameCount = 0;
  private double lastTargetArea = 0.0;

  public FuelTrayCameraSubsystem() {
    fuelTrayCamera = new PhotonCamera(FUEL_TRAY_CAMERA_NAME);
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> results = fuelTrayCamera.getAllUnreadResults();
    if (results.isEmpty()) {
      return;
    }

    PhotonPipelineResult latestResult = results.get(results.size() - 1);
    if (!latestResult.hasTargets()) {
      stationaryFrameCount = 0;
      jamDetected = false;
      SmartDashboard.putBoolean("Fuel Tray Jam", false);
      return;
    }

    PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
    double currentArea = bestTarget.getArea();

    if (currentArea > JAM_DETECTION_TARGET_AREA_THRESHOLD) {
      double areaChange = Math.abs(currentArea - lastTargetArea);
      if (areaChange < JAM_AREA_CHANGE_THRESHOLD) {
        stationaryFrameCount++;
      } else {
        stationaryFrameCount = 0;
      }
    } else {
      stationaryFrameCount = 0;
    }

    lastTargetArea = currentArea;
    jamDetected = stationaryFrameCount >= JAM_FRAME_THRESHOLD;

    SmartDashboard.putBoolean("Fuel Tray Jam", jamDetected);
    SmartDashboard.putNumber("Fuel Tray Area", currentArea);
  }

  public boolean isJamDetected() {
    return jamDetected;
  }
}
