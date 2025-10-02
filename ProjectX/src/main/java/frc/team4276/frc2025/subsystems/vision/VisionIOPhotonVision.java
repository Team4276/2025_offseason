package frc.team4276.frc2025.subsystems.vision;

import static frc.team4276.frc2025.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.team4276.frc2025.subsystems.vision.VisionConstants.configs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final int index;
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  public VisionIOPhotonVision(int index) {
    this.index = index;
    camera = new PhotonCamera(configs[index].name);
    robotToCamera = configs[index].robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();
    List<TargetObservation> txtyObservations = new ArrayList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Add tx/ty observation
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          if (target.getFiducialId() != -1) {
            txtyObservations.add(
                new TargetObservation(
                    result.getTimestampSeconds(),
                    target.getFiducialId(),
                    index,
                    Units.degreesToRadians(target.getYaw()),
                    Units.degreesToRadians(target.getPitch()),
                    target.bestCameraToTarget.getTranslation().getNorm()));
          }
        }
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Pose3d camPose = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                multitagResult.fiducialIDsUsed.size(), // Tag count
                camPose, // 3D pose estimate
                camPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                totalTagDistance / result.targets.size(), // Average tag distance
                PoseObservationType.PHOTONVISION)); // Observation type
      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget1 =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget1 = target.bestCameraToTarget;
          Transform3d fieldToCamera1 = fieldToTarget1.plus(cameraToTarget1.inverse());
          Pose3d camPose1 =
              new Pose3d(fieldToCamera1.getTranslation(), fieldToCamera1.getRotation());

          Pose3d camPose2;

          if (Math.abs(target.altCameraToTarget.getTranslation().getNorm()) >= 0.0001) {
            Transform3d fieldToTarget2 =
                new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
            Transform3d cameraToTarget2 = target.altCameraToTarget;
            Transform3d fieldToCamera2 = fieldToTarget2.plus(cameraToTarget2.inverse());

            camPose2 = new Pose3d(fieldToCamera2.getTranslation(), fieldToCamera2.getRotation());
          } else {
            camPose2 = camPose1;
          }

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  1, // Tag count
                  camPose1, // 3D pose estimate
                  camPose2, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  cameraToTarget1.getTranslation().getNorm(), // Tag distances
                  PoseObservationType.PHOTONVISION)); // Observation type
        }
      }
    }

    // Save tx/ty observations to inputs object
    inputs.targetObservations = new TargetObservation[txtyObservations.size()];
    for (int i = 0; i < txtyObservations.size(); i++) {
      inputs.targetObservations[i] = txtyObservations.get(i);
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
