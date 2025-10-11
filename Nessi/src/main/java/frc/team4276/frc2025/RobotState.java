package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.field.FieldConstants.ReefSide;
import frc.team4276.frc2025.subsystems.SuperstructureConstants.ScoringSide;
import frc.team4276.frc2025.subsystems.vision.VisionIO.TagObservation;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.ElasticUI;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

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

  private SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, Rotation2d.kZero, lastWheelPositions);
  private TimeInterpolatableBuffer<Pose2d> odomPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(2.0);

  private final Map<Integer, TagObservation> priorityTagObservations = new HashMap<>();

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
    odometry.resetPose(pose);
    odomPoseBuffer.clear();
    poseEstimate = pose;
    odomPoseEstimate = pose;
  }

  public void setTrajectorySetpoint(Pose2d setpoint) {
    trajectorySetpoint = setpoint;
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {
    // Update gyro angle
    if (yaw == null) {
      // Derive from kinematics
      yaw =
          odomPoseEstimate
              .getRotation()
              .rotateBy(
                  new Rotation2d(kinematics.toTwist2d(lastWheelPositions, wheelPositions).dtheta));
    }

    lastWheelPositions = wheelPositions;

    var latestOdomEstimate = odometry.update(yaw, wheelPositions);

    poseEstimate = poseEstimate.exp(odomPoseEstimate.log(latestOdomEstimate));

    odomPoseBuffer.addSample(timestamp, latestOdomEstimate);

    odomPoseEstimate = latestOdomEstimate;
  }

  // TODO: check if this is taking a lot of processing time; if so, remove further
  // tags in vision
  /** Adds a new timestamped vision measurement. */
  public void addVisionObservation(int camera, TagObservation... observations) {
    for (var obs : observations) {
      if (scoringSideToAccept == ScoringSide.LEFT) {
        if (camera == 1) {
          continue;
        }
      } else if (scoringSideToAccept == ScoringSide.RIGHT) {
        if (camera == 0) {
          continue;
        }
      }

      if (!shouldAcceptTagEstimate(obs.tagId())) continue;

      if (priorityTagObservations.containsKey(obs.tagId())
          && obs.timestamp() <= priorityTagObservations.get(obs.tagId()).timestamp()) {
        continue;
      }

      priorityTagObservations.put(obs.tagId(), obs);

      // Get rotation at timestamp
      var sample = odomPoseBuffer.getSample(obs.timestamp());
      if (sample.isEmpty()) {
        // exit if not there
        return;
      }

      // Use gyro angle at time for robot rotation
      // poseEstimate = obs.robotPose().transformBy(new Transform2d(sample.get(),
      // odomPoseEstimate));
      poseEstimate = obs.robotPose();
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
        yield observationTagId == getTagIdFromClosestRotationSide();

      case POSE_BASED:
        yield observationTagId == getTagIdFromClosestPoseSide();

      case ACCEPT_SIDE:
        yield observationTagId == FieldConstants.getTagIdFromSide(reefSideToAccept);
    };
  }

  public int getTagIdFromClosestRotationSide() {
    return 7;
  }

  public int getTagIdFromClosestPoseSide() {
    int closestTag = -1;
    double minDistance = Double.POSITIVE_INFINITY;
    double currDistance = 0.0;
    for (AprilTag tag : FieldConstants.apriltagLayout.getTags()) {
      if (!FieldConstants.isReefTag(tag.ID)) {
        if (AllianceFlipUtil.shouldFlip() && tag.ID > 11) {
          continue;
        }
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
    this.scoringSideToAccept = scoringSideToAccept;
  }

  public void setReefSideToAccept(ReefSide reefSideToAccept) {
    this.reefSideToAccept = reefSideToAccept;
    setVisionMode(VisionMode.ACCEPT_SIDE);
  }

  public Pose2d getEstimatedPose() {
    return useTrajectorySetpoint() ? trajectorySetpoint : poseEstimate;
  }

  public Pose2d getEstimatedOdomPose() {
    return odomPoseEstimate;
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
