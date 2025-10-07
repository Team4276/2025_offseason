package frc.team4276.frc2025.subsystems.vision;

import static frc.team4276.frc2025.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final int index;
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  private final double vFov;
  private final int verticalResolution;

  private final List<TagObservation> txtyObservations;

  private final ArrayList<Pair<Double, Double>> cornerListPairs;
  private final Set<Short> tagIds;

  public VisionIOPhotonVision(int index) {
    this.index = index;
    camera = new PhotonCamera(configs[index].name);
    robotToCamera = configs[index].robotToCamera;
    vFov = configs[index].vFov;
    verticalResolution = configs[index].verticalResolution;

    tagIds = new HashSet<>();

    txtyObservations = new ArrayList<>();
    cornerListPairs = new ArrayList<Pair<Double, Double>>();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    tagIds.clear();
    txtyObservations.clear();

    // Read new camera observations
    for (var result : camera.getAllUnreadResults()) {
      // Add tx/ty observation
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          if (target.getFiducialId() != -1) {
            for (int i = 0; i < 4; i++) {
              cornerListPairs.add(
                  new Pair<>(target.detectedCorners.get(i).x, target.detectedCorners.get(i).y));
            }

            double distanceToTag =
                calculateDistanceToAprilTagInMetersUsingTrigMethod(
                    calculateAngleEncompassingTagHeight(
                        calculateTargetHeightInPixels(cornerListPairs)),
                    FieldConstants.apriltagLayout.getTagPose(target.fiducialId).get().getZ());

            txtyObservations.add(
                new TagObservation(
                    target.fiducialId,
                    result.getTimestampSeconds(),
                    index,
                    Units.degreesToRadians(target.yaw),
                    distanceToTag,
                    calculateRobotPose(
                        target.fiducialId, distanceToTag, Rotation2d.fromDegrees(target.yaw))));

            tagIds.add((short) target.fiducialId);
          }
        }
      }
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    for (int i = 0; i < tagIds.size(); i++) {
      inputs.tagIds[i++] = tagIds[i];
    }

    // Save tx/ty observations to inputs object
    inputs.targetObservations = new TagObservation[txtyObservations.size()];
    for (int i = 0; i < txtyObservations.size(); i++) {
      inputs.targetObservations[i] = txtyObservations.get(i);
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
                        * (tagHeightOffGround - robotToCamera.getZ() + tagHeight)
                        * tanOfTheta1
                        * (tagHeightOffGround - robotToCamera.getZ()))));
    return Math.abs((tagHeight + sqrtTerm) / (2 * tanOfTheta1));
  }

  private Pose2d calculateRobotPose(
      final int tagId, double distanceToTagMeters, Rotation2d horizontalAngleToTarget) {
    var tagPose = FieldConstants.apriltagLayout.getTagPose(tagId).get().toPose2d();

    var robotRotation = RobotState.getInstance().getEstimatedPose().getRotation();

    var robotRotationWithLimelightCorrection =
        robotRotation.plus(robotToCamera.getRotation().toRotation2d());

    var scaledTx = Rotation2d.fromDegrees(-horizontalAngleToTarget.div(1.0).getDegrees());

    var cameraToRobotCenter =
        this.robotToCamera.inverse().getTranslation().toTranslation2d().rotateBy(robotRotation);

    var angleToTag = scaledTx.plus(robotRotationWithLimelightCorrection);

    var translation = new Translation2d(distanceToTagMeters, angleToTag);
    var translatedPose = tagPose.getTranslation().minus(translation);

    var fieldRelativeRobotTranslation = translatedPose.plus(cameraToRobotCenter);

    return new Pose2d(fieldRelativeRobotTranslation, robotRotation);
  }
}
