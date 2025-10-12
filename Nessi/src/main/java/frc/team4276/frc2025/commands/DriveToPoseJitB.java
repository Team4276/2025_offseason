package frc.team4276.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.drive.Drive.DriveMode;
import frc.team4276.util.dashboard.LoggedTunablePID;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveToPoseJitB extends Command {
  private static final LoggedTunablePID driveController =
      new LoggedTunablePID(3.0, 0.0, 0.0, 0.01, "DriveToPoseJitB/Translation");
  private static final LoggedTunablePID thetaController =
      new LoggedTunablePID(4.0, 0.1, 0.0, Units.degreesToRadians(1.0), "DriveToPoseJitB/Rotation");

  private static boolean isRunning = false;

  private static Pose2d error = Pose2d.kZero;

  private final Drive drive;
  private final Supplier<Pose2d> target;
  private final Supplier<Pose2d> robotPose;

  public DriveToPoseJitB(Drive drive, Supplier<Pose2d> target) {
    this(drive, target, () -> RobotState.getInstance().getEstimatedPose());
  }

  public DriveToPoseJitB(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> robotPose) {
    this.drive = drive;
    this.target = target;
    this.robotPose = robotPose;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    thetaController.reset();
    driveController.reset();
  }

  @Override
  public void execute() {
    isRunning = true;

    Pose2d currentPose = robotPose.get();

    Translation2d trans = currentPose.getTranslation().minus(target.get().getTranslation());
    Translation2d linearOutput =
        new Translation2d(driveController.calculate(trans.getNorm(), 0.0), trans.getAngle());
    if (trans.getNorm() < thetaController.getErrorTolerance()) {
      linearOutput = Translation2d.kZero;
    }

    double thetaError =
        MathUtil.angleModulus(
            currentPose.getRotation().minus(target.get().getRotation()).getRadians());
    double omega = thetaController.calculate(thetaError, 0.0);
    if (Math.abs(thetaError) < thetaController.getErrorTolerance()) {
      omega = 0.0;
    }

    error = target.get().relativeTo(currentPose);

    Logger.recordOutput("DriveToPoseJitB/DistanceError", trans.getNorm());
    Logger.recordOutput("DriveToPoseJitB/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPoseJitB/ThetaSetpoint", target.get().getRotation().getRadians());
    Logger.recordOutput("DriveToPoseJitB/ThetaError", thetaError);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearOutput.getX(), linearOutput.getY(), omega, currentPose.getRotation()),
        DriveMode.AUTO_ALIGN);
  }

  @AutoLogOutput(key = "DriveToPoseJitB/atGoal")
  public static boolean atGoal() {
    return isRunning && driveController.atSetpoint() && thetaController.atSetpoint();
  }

  @AutoLogOutput(key = "DriveToPoseJitB/withinTol")
  public static boolean withinTolerance(double translation, double heading) {
    return isRunning
        && Math.abs(error.getTranslation().getNorm()) < translation
        && Math.abs(error.getRotation().getRadians()) < heading;
  }

  public static Pose2d distToGoal() {
    return error;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    isRunning = false;
    // Clear logs
    Logger.recordOutput("DriveToPoseJitB/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPoseJitB/Goal", new Pose2d[] {});
  }
}
