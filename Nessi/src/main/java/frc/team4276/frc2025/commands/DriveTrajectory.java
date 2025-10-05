package frc.team4276.frc2025.commands;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.drive.Drive.DriveMode;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import frc.team4276.util.dashboard.LoggedTunablePID;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveTrajectory extends Command {
  private static final LoggedTunablePID xController =
      new LoggedTunablePID(4.0, 0.0, 0.0, 0.1, "DriveTrajectory/TranslationX");
  private static final LoggedTunablePID yController =
      new LoggedTunablePID(4.0, 0.0, 0.0, 0.1, "DriveTrajectory/TranslationY");
  private static final LoggedTunablePID rotController =
      new LoggedTunablePID(3.0, 0.0, 0.0, Math.toRadians(1.0), "DriveTrajectory/Rotation");
  private static final LoggedTunableNumber maxError =
      new LoggedTunableNumber("DriveTrajectory/maxError", 0.75);

  private final Drive drive;
  private final PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> robotPose;

  private double startTime = 0.0;
  private double timeOffset = 0.0;

  private List<Vector<N2>> moduleForces =
      List.of(
          VecBuilder.fill(0.0, 0.0),
          VecBuilder.fill(0.0, 0.0),
          VecBuilder.fill(0.0, 0.0),
          VecBuilder.fill(0.0, 0.0));

  private final double[] dummyForces = {0.0, 0.0, 0.0, 0.0};

  public DriveTrajectory(Drive drive, PathPlannerTrajectory trajectory) {
    this(drive, trajectory, () -> RobotState.getInstance().getEstimatedPose());
  }

  public DriveTrajectory(
      Drive drive, PathPlannerTrajectory trajectory, Supplier<Pose2d> robotPose) {
    this.drive = drive;
    this.trajectory = trajectory;
    this.robotPose = robotPose;

    rotController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startTime = Timer.getTimestamp();
    timeOffset = 0.0;
  }

  @Override
  public void execute() {
    var currentPose = robotPose.get();

    var sampledState = trajectory.sample(getTrajectoryTime());

    if (sampledState.pose.getTranslation().getDistance(currentPose.getTranslation())
        > maxError.getAsDouble()) {
      timeOffset += 0.02;

      var dummyState = trajectory.sample(getTrajectoryTime());

      sampledState = new PathPlannerTrajectoryState();
      sampledState.timeSeconds = dummyState.timeSeconds;
      sampledState.pose = dummyState.pose;
      sampledState.heading = dummyState.heading;
      sampledState.feedforwards =
          new DriveFeedforwards(dummyForces, dummyForces, dummyForces, dummyForces, dummyForces);
    }

    RobotState.getInstance().setTrajectorySetpoint(sampledState.pose);

    double xError = sampledState.pose.getX() - currentPose.getTranslation().getX();
    double yError = sampledState.pose.getY() - currentPose.getTranslation().getY();
    double xFeedback = xController.calculate(0.0, xError);
    double yFeedback = yController.calculate(0.0, yError);
    double thetaError =
        MathUtil.angleModulus(
            sampledState.pose.getRotation().minus(currentPose.getRotation()).getRadians());
    double thetaFeedback = rotController.calculate(0.0, thetaError);

    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            sampledState.fieldSpeeds.vxMetersPerSecond + xFeedback,
            sampledState.fieldSpeeds.vyMetersPerSecond + yFeedback,
            sampledState.fieldSpeeds.omegaRadiansPerSecond + thetaFeedback,
            currentPose.getRotation());

    moduleForces = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      moduleForces.add(
          VecBuilder.fill(
              sampledState.feedforwards.robotRelativeForcesXNewtons()[i],
              sampledState.feedforwards.robotRelativeForcesYNewtons()[i]));
    }

    Logger.recordOutput("DriveTrajectory/SetpointPose", sampledState.pose);
    Logger.recordOutput("DriveTrajectory/SetpointSpeeds", sampledState.fieldSpeeds);
    Logger.recordOutput("DriveTrajectory/OutputSpeeds", outputSpeeds);
    Logger.recordOutput("DriveTrajectory/TrajectoryTime", getTrajectoryTime());

    drive.runVelocity(outputSpeeds, moduleForces, DriveMode.TRAJECTORY);
  }

  private double getTrajectoryTime() {
    return Timer.getTimestamp() - startTime - timeOffset;
  }

  @Override
  public boolean isFinished() {
    return getTrajectoryTime() > trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
