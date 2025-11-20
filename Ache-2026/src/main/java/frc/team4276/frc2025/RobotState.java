package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.kinematics;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.team4276.frc2025.field.FieldConstants;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  private Rotation2d lastYaw = Rotation2d.kZero;

  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      lastYaw,
      lastWheelPositions,
      Pose2d.kZero,
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(.9, .9, 2.0));

  private SwerveDrivePoseEstimator odomPoseEstimator = new SwerveDrivePoseEstimator(kinematics, lastYaw,
      lastWheelPositions, Pose2d.kZero);

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  @AutoLogOutput
  private int visionUpdateCount = 0;

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
    poseEstimator.resetPosition(lastYaw, lastWheelPositions, pose);
    odomPoseEstimator.resetPosition(lastYaw, lastWheelPositions, pose);
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {
    if(yaw == null){
      var twist = kinematics.toTwist2d(lastWheelPositions, wheelPositions);
      yaw = odomPoseEstimator.getEstimatedPosition().getRotation().rotateBy(new Rotation2d(twist.dtheta));
    }
    poseEstimator.updateWithTime(timestamp, yaw, wheelPositions);
    odomPoseEstimator.updateWithTime(timestamp, yaw, wheelPositions);
    lastWheelPositions = wheelPositions;
    lastYaw = yaw;
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    visionUpdateCount++;

    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public int getTagIdFromClosestPoseSide() {
    int closestTag = -1;
    double minDistance = Double.POSITIVE_INFINITY;
    double currDistance = 0.0;
    for (AprilTag tag : FieldConstants.apriltagLayout.getTags()) {
      if (!FieldConstants.isReefTag(tag.ID)) {
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

  // @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // @AutoLogOutput(key = "RobotState/EstimatedOdomPose")
  public Pose2d getEstimatedOdomPose() {
    return odomPoseEstimator.getEstimatedPosition();
  }

  public Optional<Pose2d> getEstimatedOdomPoseAtTime(double timestamp) {
    return odomPoseEstimator.sampleAt(timestamp);
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getEstimatedPose().getRotation());
  }
}
