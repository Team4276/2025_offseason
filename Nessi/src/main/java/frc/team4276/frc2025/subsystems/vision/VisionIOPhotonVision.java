package frc.team4276.frc2025.subsystems.vision;

import static frc.team4276.frc2025.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

  private final List<PoseObservation> poseObservations;

  public enum DistanceCalcMethod {
    MECH_A,
    TRANSFORM_3D,
    PHOTON_UTILS,
    JITB,
  }

  public VisionIOPhotonVision(int index) {
    this.index = index;
    camera = new PhotonCamera(configs[index].name);
    robotToCamera = configs[index].robotToCamera;
    vFov = configs[index].vFov;
    verticalResolution = configs[index].verticalResolution;

    tagIds = new HashSet<>();

    txtyObservations = new ArrayList<>();
    cornerListPairs = new ArrayList<Pair<Double, Double>>();

    poseObservations = new ArrayList<>();

    SmartDashboard.putBoolean("Camera_" + index + "_Use_PU_Distance_Calc", false);
    SmartDashboard.putBoolean("Camera_" + index + "_Use_3D_Transform_Distance_Calc", true);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    tagIds.clear();
    txtyObservations.clear();
    cornerListPairs.clear();
    poseObservations.clear();

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

        double distanceToTag = -1;
        DistanceCalcMethod distanceCalcMethod = SmartDashboard
            .getBoolean("Camera_" + index + "_Use_MECH_A_Distance_Calc", false)
                ? DistanceCalcMethod.MECH_A
                : SmartDashboard.getBoolean(
                    "Camera_" + index + "_Use_3D_Transform_Distance_Calc", true)
                        ? DistanceCalcMethod.TRANSFORM_3D
                        : SmartDashboard.getBoolean("Camera_" + index + "_Use_PU_Distance_Calc", false)
                            ? DistanceCalcMethod.PHOTON_UTILS
                            : DistanceCalcMethod.JITB;

        double rawDistToTag = target.bestCameraToTarget.getTranslation().getNorm();
        double distanceToTagMechA = Rotation2d.fromDegrees(
            target.pitch + Units.radiansToDegrees(-robotToCamera.getRotation().getY()))
            .getCos()
            * rawDistToTag;

        double a = FieldConstants.apriltagLayout.getTagPose(target.fiducialId).get().getZ()
            - robotToCamera.getZ();
        double distanceToTag3d = Math.sqrt(rawDistToTag * rawDistToTag - (a * a));

        double distanceToTagPU = PhotonUtils.calculateDistanceToTargetMeters(
            robotToCamera.getZ(),
            FieldConstants.apriltagLayout.getTagPose(target.fiducialId).get().getZ(),
            -robotToCamera.getRotation().getY(),
            Units.degreesToRadians(target.pitch));

        double distanceToTagJitb = calculateDistanceToAprilTagInMetersUsingTrigMethod(
            calculateAngleEncompassingTagHeight(calculateTargetHeightInPixels(cornerListPairs)),
            FieldConstants.apriltagLayout.getTagPose(target.fiducialId).get().getZ());

        switch (distanceCalcMethod) {
          case MECH_A:
            distanceToTag = distanceToTagMechA;

            break;

          case TRANSFORM_3D:
            distanceToTag = distanceToTag3d;

            break;

          case PHOTON_UTILS:
            distanceToTag = distanceToTagPU;
            break;

          case JITB:
            distanceToTag = distanceToTagJitb;
            break;

          default:
            break;
        }

        if (distanceToTag < 0) {
          continue;
        }

        var poseEstimate =
            // calculateRobotPose(
            // target.fiducialId,
            // distanceToTag,
            // Rotation2d.fromDegrees(target.yaw),
            // result.getTimestampSeconds());
            calculateRobotPose(
                target.getFiducialId(),
                result.getTimestampSeconds(),
                Units.degreesToRadians(target.yaw),
                Units.degreesToRadians(target.pitch),
                target.bestCameraToTarget.getTranslation().getNorm());

        if (poseEstimate.isPresent()) {
          txtyObservations.add(
              new TagObservation(
                  target.fiducialId,
                  result.getTimestampSeconds(),
                  index,
                  Units.degreesToRadians(target.yaw),
                  distanceToTag,
                  poseEstimate.get(),
                  distanceCalcMethod,
                  new double[] {
                      distanceToTagMechA, distanceToTag3d, distanceToTagPU, distanceToTagJitb
                  }));
        }
      }

      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();
        int[] tagsUsed = new int[multitagResult.fiducialIDsUsed.size()];
        for (int i = 0; i < multitagResult.fiducialIDsUsed.size(); i++) {
          tagsUsed[i] = (int) multitagResult.fiducialIDsUsed.get(i);
        }
        var cameraPose = multitagResult.estimatedPose.best;
        var robotPose = cameraPose.plus(robotToCamera.inverse());

        poseObservations.add(
            new PoseObservation(
                tagsUsed,
                result.getTimestampSeconds(),
                index,
                Pose3d.kZero.transformBy(robotPose),
                cameraPose.getTranslation().getNorm()));

      } else if (result.hasTargets()) {
        double ambiguity = result.getBestTarget().getPoseAmbiguity();

        var bestCameraToTarget = result.getBestTarget().getBestCameraToTarget();
        var altCameraToTarget = result.getBestTarget().getAlternateCameraToTarget();

        var bestRobotPose = Pose3d.kZero.transformBy(bestCameraToTarget);
        var altRobotPose = Pose3d.kZero.transformBy(altCameraToTarget);

        Pose3d robotPose;

        // Borrow from Mech A
        if (ambiguity < (1 - ambiguity) * 0.4) {
          Rotation2d odomRotation = RobotState.getInstance().getEstimatedPose().getRotation();
          Rotation2d bestVisionRotation = bestRobotPose.getRotation().toRotation2d();
          Rotation2d altVisionRotation = altRobotPose.getRotation().toRotation2d();

          if (Math.abs(odomRotation.minus(bestVisionRotation).getRadians()) < Math
              .abs(odomRotation.minus(altVisionRotation).getRadians())) {
            robotPose = bestRobotPose;

          } else {
            robotPose = altRobotPose;
          }

          // Exit if robot pose is off the field
          if (robotPose.getX() < -fieldBorderMargin
              || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
              || robotPose.getY() < -fieldBorderMargin
              || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
            continue;
          }

          poseObservations.add(
              new PoseObservation(
                  new int[] { result.getBestTarget().fiducialId },
                  result.getTimestampSeconds(),
                  index,
                  robotPose,
                  bestCameraToTarget.getTranslation().getNorm()));
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

    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int j = 0; j < poseObservations.size(); j++) {
      inputs.poseObservations[j] = poseObservations.get(j);
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
    var sqrtTerm = Math.sqrt(
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

  /**
   * This no work bc target yaw when the camera is pitched results in a different
   * actual yaw of the target relative to the robot
   */
  @SuppressWarnings("unused") // hehe
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

    var cameraToRobotCenter = this.robotToCamera.inverse().getTranslation().toTranslation2d().rotateBy(robotRotation);

    var angleToTag = scaledTx.plus(correctedCameraRotation);

    var translation = new Translation2d(distanceToTagMeters, angleToTag);
    var translatedPose = tagPose.getTranslation().minus(translation);

    var fieldRelativeRobotTranslation = translatedPose.plus(cameraToRobotCenter);

    return Optional.of(new Pose2d(fieldRelativeRobotTranslation, robotRotation));
  }

  private Optional<Pose2d> calculateRobotPose(int tagId, double timestamp, double targetYaw, double targetPitch,
      double camToTargetDist) {
    var robotPoseAtTime = RobotState.getInstance().getEstimatedOdomPoseAtTime(timestamp);

    if (robotPoseAtTime.isEmpty()) {
      return Optional.empty();
    }

    Translation2d camToTagTranslation = new Pose3d(Translation3d.kZero, new Rotation3d(0, targetPitch, -targetYaw))
        .transformBy(
            new Transform3d(new Translation3d(camToTargetDist, 0, 0), Rotation3d.kZero))
        .getTranslation()
        .rotateBy(new Rotation3d(0, robotToCamera.getRotation().getY(), 0))
        .toTranslation2d();
    Rotation2d camToTagRotation = robotPoseAtTime.get().getRotation().plus(
        robotToCamera.getRotation().toRotation2d().plus(camToTagTranslation.getAngle()));
    var tagPose2d = FieldConstants.apriltagLayout.getTagPose(tagId);

    if (tagPose2d.isEmpty()) {
      return Optional.empty();
    }

    Translation2d fieldToCameraTranslation = new Pose2d(tagPose2d.get().getTranslation().toTranslation2d(),
        camToTagRotation.plus(Rotation2d.kPi))
        .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0.0, Rotation2d.kZero))
        .getTranslation();
    Pose2d robotPose = new Pose2d(
        fieldToCameraTranslation, robotPoseAtTime.get().getRotation().plus(robotToCamera.getRotation().toRotation2d()))
        .transformBy(new Transform2d(
            new Pose2d(robotToCamera.getTranslation().toTranslation2d(), robotToCamera.getRotation().toRotation2d()),
            Pose2d.kZero));
    // Use gyro angle at time for robot rotation
    return Optional.of(new Pose2d(robotPose.getTranslation(), robotPoseAtTime.get().getRotation()));
  }
}
