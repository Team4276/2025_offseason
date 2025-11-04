package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.kinematics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

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
      VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(1)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(40)));
      
  private SwerveDrivePoseEstimator odomPoseEstimator = new SwerveDrivePoseEstimator(kinematics, lastYaw,
      lastWheelPositions, Pose2d.kZero);

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  @AutoLogOutput private int visionUpdateCount = 0;

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

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/EstimatedOdomPose")
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
