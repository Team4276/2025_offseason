package frc.team4276.frc2025.commands;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.subsystems.superstructure.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.drive.Drive.DriveMode;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import frc.team4276.util.dashboard.LoggedTunablePID;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveChoreo extends Command { // TODO: test choreo trajectory auto
  private static final LoggedTunablePID xController =
      new LoggedTunablePID(4.0, 0.0, 0.0, 0.1, "DriveTrajectory/TranslationX");
  private static final LoggedTunablePID yController =
      new LoggedTunablePID(4.0, 0.0, 0.0, 0.1, "DriveTrajectory/TranslationY");
  private static final LoggedTunablePID rotController =
      new LoggedTunablePID(3.0, 0.0, 0.0, Math.toRadians(1.0), "DriveTrajectory/Rotation");
  private static final LoggedTunableNumber maxError =
      new LoggedTunableNumber("DriveTrajectory/maxError", 0.75);

  private final Drive drive;
  private final Trajectory<SwerveSample> trajectory;
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

  public DriveChoreo(Drive drive, Trajectory<SwerveSample> trajectory) {
    this(drive, trajectory, () -> RobotState.getInstance().getEstimatedPose());
  }

  public DriveChoreo(Drive drive, Trajectory<SwerveSample> trajectory, Supplier<Pose2d> robotPose) {
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

    var sampledState = trajectory.sampleAt(getTrajectoryTime(), false).get();

    if (sampledState.getPose().getTranslation().getDistance(currentPose.getTranslation())
        > maxError.getAsDouble()) {
      timeOffset += 0.02;

      var dummyState = trajectory.sampleAt(getTrajectoryTime(), false).get();

      sampledState =
          new SwerveSample(
              dummyState.getTimestamp(),
              dummyState.x,
              dummyState.y,
              dummyState.heading,
              dummyState.vx,
              dummyState.vy,
              dummyState.omega,
              dummyState.ax,
              dummyState.ay,
              dummyState.alpha,
              dummyForces,
              dummyForces);
    }

    RobotState.getInstance().setTrajectorySetpoint(sampledState.getPose());

    double xError = sampledState.x - currentPose.getTranslation().getX();
    double yError = sampledState.y - currentPose.getTranslation().getY();
    double xFeedback = xController.calculate(0.0, xError);
    double yFeedback = yController.calculate(0.0, yError);
    double thetaError =
        MathUtil.angleModulus(
            sampledState.getPose().getRotation().minus(currentPose.getRotation()).getRadians());
    double thetaFeedback = rotController.calculate(0.0, thetaError);

    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            sampledState.vx + xFeedback,
            sampledState.vy + yFeedback,
            sampledState.omega + thetaFeedback,
            currentPose.getRotation());

    moduleForces = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      moduleForces.add(
          VecBuilder.fill(sampledState.moduleForcesX()[i], sampledState.moduleForcesY()[i]));
    }

    Logger.recordOutput("DriveTrajectory/SetpointPose", sampledState.getPose());
    Logger.recordOutput("DriveTrajectory/SetpointSpeeds", sampledState.getChassisSpeeds());
    Logger.recordOutput("DriveTrajectory/OutputSpeeds", outputSpeeds);
    Logger.recordOutput("DriveTrajectory/TrajectoryTime", getTrajectoryTime());

    drive.runVelocity(outputSpeeds, moduleForces, DriveMode.TRAJECTORY);
  }

  private double getTrajectoryTime() {
    return Timer.getTimestamp() - startTime - timeOffset;
  }

  @Override
  public boolean isFinished() {
    return getTrajectoryTime() > trajectory.getTotalTime();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
