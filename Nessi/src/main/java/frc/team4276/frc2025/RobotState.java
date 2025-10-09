package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.field.FieldConstants.ReefSide;
import frc.team4276.frc2025.subsystems.vision.VisionIO.TagObservation;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.ElasticUI;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

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
      poseEstimate = obs.robotPose().transformBy(new Transform2d(sample.get(), odomPoseEstimate));
    }
  }

  private boolean shouldAcceptTagEstimate(int observationTagId) {
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
        yield observationTagId == reefSideToAccept.getTagId();
    };
  }

  public int getTagIdFromClosestRotationSide() {
    return -1;
  }

  public int getTagIdFromClosestPoseSide() {
    int closestTag = -1;
    double minDistance = Double.POSITIVE_INFINITY;
    double currDistance = 0.0;
    for (AprilTag tag : FieldConstants.apriltagLayout.getTags()) {
      if ((tag.ID >= 1 && tag.ID < 6) || (tag.ID >= 12 && tag.ID < 17)) {
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

  public void setSideToAccept(ReefSide reefSideToAccept) {
    this.reefSideToAccept = reefSideToAccept;
    setVisionMode(VisionMode.ACCEPT_SIDE);
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return useTrajectorySetpoint() ? trajectorySetpoint : poseEstimate;
  }

  @AutoLogOutput(key = "RobotState/EstimatedOdomPose")
  public Pose2d getEstimatedOdomPose() {
    return odomPoseEstimate;
  }

  public Optional<Pose2d> getEstimatedOdomPoseAtTime(double timestamp) {
    return odomPoseBuffer.getSample(timestamp);
  }

  public int getTagIdFromSide(ReefSide side) {
    return switch (side) {
      case AB -> AllianceFlipUtil.shouldFlip() ? 7 : 18;
      case CD -> AllianceFlipUtil.shouldFlip() ? 8 : 17;
      case EF -> AllianceFlipUtil.shouldFlip() ? 9 : 22;
      case GH -> AllianceFlipUtil.shouldFlip() ? 10 : 21;
      case IJ -> AllianceFlipUtil.shouldFlip() ? 11 : 20;
      case KL -> AllianceFlipUtil.shouldFlip() ? 6 : 19;
    };
  }

  public Optional<ReefSide> getSideFromTagId(int id) {
    return switch (id) {
      case 6 -> Optional.of(ReefSide.KL);
      case 7 -> Optional.of(ReefSide.AB);
      case 8 -> Optional.of(ReefSide.CD);
      case 9 -> Optional.of(ReefSide.EF);
      case 10 -> Optional.of(ReefSide.GH);
      case 11 -> Optional.of(ReefSide.IJ);

      case 17 -> Optional.of(ReefSide.CD);
      case 18 -> Optional.of(ReefSide.AB);
      case 19 -> Optional.of(ReefSide.KL);
      case 20 -> Optional.of(ReefSide.IJ);
      case 21 -> Optional.of(ReefSide.GH);
      case 22 -> Optional.of(ReefSide.EF);

      default -> Optional.empty();
    };
  }

  private boolean useTrajectorySetpoint() {
    return enableSimTrajPoseEstimation
        ? false
        : Constants.isSim && DriverStation.isAutonomousEnabled();
  }

  public Pose2d getTrajectorySetpoint() {
    return trajectorySetpoint;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getEstimatedPose().getRotation());
  }
}
