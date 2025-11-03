package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.kinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.Optional;

public class RobotState {
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Pose2d estimatedPose = Pose2d.kZero;
  private Pose2d odomPoseEstimate = Pose2d.kZero;
  private Rotation2d gyroOffset = Rotation2d.kZero;

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

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private static RobotState mInstance;

  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }
    return mInstance;
  }

  private RobotState() {
  }

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    gyroOffset = pose.getRotation().minus(odomPoseEstimate.getRotation().minus(gyroOffset));
    odomPoseBuffer.clear();
    estimatedPose = pose;
    odomPoseEstimate = pose;
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {
    // Derive from kinematics
    var twist = kinematics.toTwist2d(lastWheelPositions, wheelPositions);
    var lastOdometryPose = odomPoseEstimate;
    lastWheelPositions = wheelPositions;
    odomPoseEstimate = odomPoseEstimate.exp(twist);

    // Update gyro angle
    if (yaw != null) {
      Rotation2d angle = yaw.plus(gyroOffset);
      odomPoseEstimate = new Pose2d(odomPoseEstimate.getTranslation(), angle);
    }

    odomPoseBuffer.addSample(timestamp, odomPoseEstimate);

    estimatedPose = estimatedPose.exp(lastOdometryPose.log(odomPoseEstimate));
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public Pose2d getEstimatedOdomPose() {
    return odomPoseEstimate;
  }

  public Optional<Pose2d> getEstimatedOdomPoseAtTime(double timestamp) {
    return odomPoseBuffer.getSample(timestamp);
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getEstimatedPose().getRotation());
  }
}
