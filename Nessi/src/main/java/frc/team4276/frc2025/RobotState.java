package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.field.FieldConstants.ReefSide;
import frc.team4276.frc2025.subsystems.SuperstructureConstants.ScoringSide;
import frc.team4276.frc2025.subsystems.vision.VisionIO.PoseObservation;
import frc.team4276.frc2025.subsystems.vision.VisionIO.TagObservation;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.ElasticUI;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Pose2d poseEstimate = Pose2d.kZero;
  private Pose2d odomPoseEstimate = Pose2d.kZero;
  private Pose2d lastOdometryPose = Pose2d.kZero;

  private double poseBufferHistorySeconds = 2.0;
  private TimeInterpolatableBuffer<Pose2d> odomPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferHistorySeconds);

  public enum VisionMode {
    ACCEPT_ALL,
    REJECT_ALL,
    ROTATION_BASED,
    POSE_BASED,
    ACCEPT_SIDE
  }

  private VisionMode visionMode = VisionMode.ACCEPT_ALL;
  private ReefSide reefSideToAccept = ReefSide.AB;
  private ScoringSide scoringSideToAccept = ScoringSide.BOTH;

  private SwerveDrivePoseEstimator poseEstimator3d =
      new SwerveDrivePoseEstimator(
          kinematics,
          poseEstimate.getRotation(),
          lastWheelPositions,
          poseEstimate,
          VecBuilder.fill(0.1, 0.1, 0.1),
          VecBuilder.fill(0.9, 0.9, 2.0));

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private Pose2d trajectorySetpoint = Pose2d.kZero;

  private static RobotState mInstance;

  private boolean enableSimTrajPoseEstimation = true;

  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }
    return mInstance;
  }

  private RobotState() {
    ElasticUI.putPoseEstimate(() -> getEstimatedPose());
  }

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    odomPoseBuffer.clear();
    poseEstimate = pose;
    odomPoseEstimate = pose;
    lastOdometryPose = pose;
  }

  public void setTrajectorySetpoint(Pose2d setpoint) {
    trajectorySetpoint = setpoint;
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {
    // Derive from kinematics
    var twist = kinematics.toTwist2d(lastWheelPositions, wheelPositions);
    lastOdometryPose = odomPoseEstimate;
    lastWheelPositions = wheelPositions;
    odomPoseEstimate = odomPoseEstimate.exp(twist);

    // Update gyro angle
    if (yaw != null) {
      odomPoseEstimate = new Pose2d(odomPoseEstimate.getTranslation(), yaw);
    }

    odomPoseBuffer.addSample(timestamp, odomPoseEstimate);

    poseEstimate = poseEstimate.exp(lastOdometryPose.log(odomPoseEstimate));
    poseEstimator3d.updateWithTime(timestamp, odomPoseEstimate.getRotation(), wheelPositions);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionObservation(TagObservation... observations) {
    for (var obs : observations) {
      if (scoringSideToAccept != ScoringSide.BOTH) {
        if (FieldConstants.getIsLeftScoringRelativeToRobot(getReefSide(), scoringSideToAccept)) {
          if (obs.camera() == 1) {
            continue;
          }
        } else {
          if (obs.camera() == 0) {
            continue;
          }
        }
      }

      if (!shouldAcceptTagEstimate(obs.tagId())) continue;

      // Get rotation at timestamp
      var sample = odomPoseBuffer.getSample(obs.timestamp());
      if (sample.isEmpty() || Timer.getTimestamp() - obs.timestamp() >= poseBufferHistorySeconds) {
        // exit if not there
        return;
      }

      // Use gyro angle at time for robot rotation
      poseEstimate = obs.robotPose().transformBy(new Transform2d(sample.get(), odomPoseEstimate));
      // poseEstimate = obs.robotPose();
    }
  }

  private boolean shouldAcceptTagEstimate(int observationTagId) {
    if (!FieldConstants.isReefTag(observationTagId)) {
      return false;
    }

    return switch (visionMode) {
      case ACCEPT_ALL:
        yield true;

      case REJECT_ALL:
        yield false;

      case ROTATION_BASED:
        yield observationTagId == getTagIdFromClosestRotationSide(getClosest60DegreeRotation());

      case POSE_BASED:
        yield observationTagId == getTagIdFromClosestPoseSide();

      case ACCEPT_SIDE:
        yield observationTagId == FieldConstants.getTagIdFromSide(reefSideToAccept);
    };
  }

  private ReefSide getReefSide() {
    return FieldConstants.getSideFromTagId(
            visionMode == VisionMode.ROTATION_BASED
                ? RobotState.getInstance().getTagIdFromClosest60DegreeRotation()
                : RobotState.getInstance().getTagIdFromClosestPoseSide())
        .get();
  }

  public int getTagIdFromClosest60DegreeRotation() {
    return getTagIdFromClosestRotationSide(getClosest60DegreeRotation());
  }

  public int getTagIdFromClosestRotationSide(Rotation2d closest60DegreeRotation) {
    var id =
        AllianceFlipUtil.shouldFlip()
            ? FieldConstants.redAllianceAngleToTagIDsMap.get(closest60DegreeRotation)
            : FieldConstants.blueAllianceAngleToTagIDsMap.get(closest60DegreeRotation);
    Logger.recordOutput("RobotState/ValidTagIdFromRotation/FrontID", id);
    return id;
  }

  public Rotation2d getClosest60DegreeRotation() {
    double[] list = {60, 120, 180, -60, -120, 0};
    double desiredRotation = 0;
    for (double e : list) {
      var rotation = Rotation2d.fromDegrees(e);
      if (poseEstimate.getRotation().minus(rotation).getDegrees() < 30.0
          && poseEstimate.getRotation().minus(rotation).getDegrees() >= -30) {
        desiredRotation = e;
      }
    }
    Logger.recordOutput("RobotState/Closest60DegreeAngle", desiredRotation);
    return Rotation2d.fromDegrees(desiredRotation);
  }

  public int getTagIdFromClosestPoseSide() {
    int closestTag = 7;
    double minDistance = Double.POSITIVE_INFINITY;
    double currDistance = 0.0;
    for (AprilTag tag : FieldConstants.apriltagLayout.getTags()) {
      if (!FieldConstants.isReefTag(tag.ID)) {
        continue;
      }

      if (AllianceFlipUtil.shouldFlip() && tag.ID > 11) {
        continue;
      }

      currDistance =
          tag.pose
              .getTranslation()
              .toTranslation2d()
              .getDistance(getEstimatedPose().getTranslation());

      if (currDistance < minDistance) {
        closestTag = tag.ID;
        minDistance = currDistance;
      }
    }

    return closestTag;
  }

  public void setVisionMode(VisionMode visionMode) {
    this.visionMode = visionMode;
  }

  public void setSideToAccept(ScoringSide scoringSideToAccept) {
    Logger.recordOutput("RobotState/ScoringSideToAccept", scoringSideToAccept);
    this.scoringSideToAccept = scoringSideToAccept;
  }

  public void setReefSideToAccept(ReefSide reefSideToAccept) {
    this.reefSideToAccept = reefSideToAccept;
    setVisionMode(VisionMode.ACCEPT_SIDE);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVision3dPoseObservation(PoseObservation... observations) {
    for (var obs : observations) {
      poseEstimator3d.addVisionMeasurement(obs.robotPose().toPose2d(), obs.timestamp());
    }
  }

  public Pose2d getEstimatedPose() {
    Logger.recordOutput("RobotState/Estimated3dPose", poseEstimator3d.getEstimatedPosition());

    return useTrajectorySetpoint() ? trajectorySetpoint : poseEstimate;
  }

  public Pose2d getEstimatedOdomPose() {
    return odomPoseEstimate;
  }

  public Pose2d getEstimated3dPose() {
    return poseEstimator3d.getEstimatedPosition();
  }

  public Optional<Pose2d> getEstimatedOdomPoseAtTime(double timestamp) {
    return odomPoseBuffer.getSample(timestamp);
  }

  private boolean useTrajectorySetpoint() {
    return enableSimTrajPoseEstimation
        ? false
        : Constants.isSim && DriverStation.isAutonomousEnabled();
  }

  public Pose2d getTrajectorySetpoint() {
    return trajectorySetpoint;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getEstimatedPose().getRotation());
  }
}
