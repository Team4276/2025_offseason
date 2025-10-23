package frc.team4276.frc2025.subsystems.vision;

import static frc.team4276.frc2025.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final int index;
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  private final double vFov;
  private final int verticalResolution;

  private final Set<Short> tagIds;

  private final List<TagObservation> txtyObservations;
  private final ArrayList<Pair<Double, Double>> cornerListPairs;

  public VisionIOPhotonVision(int index) {
    this.index = index;
    camera = new PhotonCamera(configs[index].name);
    robotToCamera = configs[index].robotToCamera;
    vFov = configs[index].vFov;
    verticalResolution = configs[index].verticalResolution;

    tagIds = new HashSet<>();

    txtyObservations = new ArrayList<>();
    cornerListPairs = new ArrayList<Pair<Double, Double>>();

    SmartDashboard.putBoolean("Camera_" + index + "_Use_PU_Distance_Calc", false);
    SmartDashboard.putBoolean("Camera_" + index + "_Use_3D_Transform_Distance_Calc", true);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    tagIds.clear();
    txtyObservations.clear();
    cornerListPairs.clear();

    // Read new camera observations
    for (var result : camera.getAllUnreadResults()) {
      if (!result.hasTargets()) {
        continue;
      }

      // Add tx/ty observations
      for (var target : result.getTargets()) {
        if (target.fiducialId == -1) {
          continue;
        }

        tagIds.add((short) target.fiducialId);

        cornerListPairs.clear();
        for (int i = 0; i < 4; i++) {
          cornerListPairs.add(
              new Pair<>(target.detectedCorners.get(i).x, target.detectedCorners.get(i).y));
        }

        double distanceToTag;

        if (SmartDashboard.getBoolean("Camera_" + index + "_Use_PU_Distance_Calc", false)) {
          distanceToTag =
              PhotonUtils.calculateDistanceToTargetMeters(
                  robotToCamera.getZ(),
                  FieldConstants.apriltagLayout.getTagPose(target.fiducialId).get().getZ(),
                  -robotToCamera.getRotation().getY(),
                  Units.degreesToRadians(-target.pitch));

        } else if (SmartDashboard.getBoolean(
                "Camera_" + index + "_Use_3D_Transform_Distance_Calc", true)
            && target.bestCameraToTarget != null) {
          distanceToTag =
              Math.sqrt(
                  Math.pow(target.bestCameraToTarget.getTranslation().getNorm(), 2)
                      - Math.pow(
                          FieldConstants.apriltagLayout.getTagPose(target.fiducialId).get().getZ()
                              - robotToCamera.getZ(),
                          2));

        } else {
          distanceToTag =
              calculateDistanceToAprilTagInMetersUsingTrigMethod(
                  calculateAngleEncompassingTagHeight(
                      calculateTargetHeightInPixels(cornerListPairs)),
                  FieldConstants.apriltagLayout.getTagPose(target.fiducialId).get().getZ());
        }

        var poseEstimate =
            calculateRobotPose(
                target.fiducialId,
                distanceToTag,
                Rotation2d.fromDegrees(target.yaw),
                result.getTimestampSeconds());

        if (poseEstimate.isPresent()) {
          txtyObservations.add(
              new TagObservation(
                  target.fiducialId,
                  result.getTimestampSeconds(),
                  index,
                  Units.degreesToRadians(target.yaw),
                  distanceToTag,
                  poseEstimate.get()));
        }
      }
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }

    // Save tx/ty observations to inputs object
    inputs.targetObservations = new TagObservation[txtyObservations.size()];
    for (int j = 0; j < txtyObservations.size(); j++) {
      inputs.targetObservations[j] = txtyObservations.get(j);
    }
  }

  private double calculateTargetHeightInPixels(Collection<Pair<Double, Double>> sortedCorners) {
    return sortedCorners.stream().mapToDouble(Pair::getSecond).max().orElse(-1)
        - sortedCorners.stream().mapToDouble(Pair::getSecond).min().orElse(-1);
  }

  private Rotation2d calculateAngleEncompassingTagHeight(double pixelHeight) {
    return Rotation2d.fromDegrees(pixelHeight * vFov / verticalResolution);
  }

  private double calculateDistanceToAprilTagInMetersUsingTrigMethod(
      Rotation2d angle, double tagHeightOffGround) {
    var tanOfTheta1 = angle.getTan();
    var sqrtTerm =
        Math.sqrt(
            Math.abs(
                Math.pow(tagHeight, 2)
                    - (4
                        * tanOfTheta1
                        * (tagHeightOffGround
                            - robotToCamera.getZ()
                            - Units.inchesToMeters(3.25)
                            + tagHeight)
                        * tanOfTheta1
                        * (tagHeightOffGround
                            - robotToCamera.getZ()
                            - Units.inchesToMeters(3.25)))));
    return Math.abs((tagHeight + sqrtTerm) / (2 * tanOfTheta1));
  }

  private Optional<Pose2d> calculateRobotPose(
      final int tagId,
      double distanceToTagMeters,
      Rotation2d horizontalAngleToTarget,
      double timestamp) {
    var tagPose = FieldConstants.apriltagLayout.getTagPose(tagId).get().toPose2d();

    var robotPoseAtTime = RobotState.getInstance().getEstimatedOdomPoseAtTime(timestamp);

    if (robotPoseAtTime.isEmpty()) {
      return Optional.empty();
    }

    var robotRotation = robotPoseAtTime.get().getRotation();

    var correctedCameraRotation = robotRotation.plus(robotToCamera.getRotation().toRotation2d());

    var scaledTx = Rotation2d.fromDegrees(-horizontalAngleToTarget.div(1.0).getDegrees());

    var cameraToRobotCenter =
        this.robotToCamera.inverse().getTranslation().toTranslation2d().rotateBy(robotRotation);

    var angleToTag = scaledTx.plus(correctedCameraRotation);

    var translation = new Translation2d(distanceToTagMeters, angleToTag);
    var translatedPose = tagPose.getTranslation().minus(translation);

    var fieldRelativeRobotTranslation = translatedPose.plus(cameraToRobotCenter);

    return Optional.of(new Pose2d(fieldRelativeRobotTranslation, robotRotation));
  }
}
