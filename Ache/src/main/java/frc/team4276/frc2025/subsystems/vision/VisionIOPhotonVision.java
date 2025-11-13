// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team4276.frc2025.subsystems.vision;

import static frc.team4276.frc2025.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final PhotonPoseEstimator poseEstimator;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name             The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(int index, Supplier<Pose2d> robotPoseSupplier) {
    camera = new PhotonCamera(configs[index].name);
    this.robotToCamera = configs[index].robotToCamera;
    this.robotPoseSupplier = robotPoseSupplier;

    poseEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.CONSTRAINED_SOLVEPNP, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // For single tag ambiguity correct, feed in full estimated pose
    poseEstimator.setReferencePose(robotPoseSupplier.get());

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation = new TargetObservation(
            Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
            Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      Optional<EstimatedRobotPose> estimate = poseEstimator.update(result);

      estimate.ifPresent(
          (poseEstimate) -> {
            // Calculate average tag distance
            double totalTagDistance = 0.0;
            for (var target : poseEstimate.targetsUsed) {
              // Add tag IDs
              tagIds.add((short) target.fiducialId);
              totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
            }

            // Add observation
            poseObservations.add(
                new PoseObservation(
                    poseEstimate.timestampSeconds, // Timestamp
                    poseEstimate.estimatedPose, // 3D pose estimate
                    poseEstimate.targetsUsed.get(0).poseAmbiguity, // Ambiguity
                    poseEstimate.targetsUsed.size(), // Tag count
                    totalTagDistance / poseEstimate.targetsUsed.size(), // Average tag distance
                    PoseObservationType.PHOTONVISION)); // Observation type
          });
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