package frc.team4276.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.drive.Drive.DriveMode;
import frc.team4276.frc2025.subsystems.drive.DriveConstants;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.LoggedTunablePID;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double LINEAR_VELOCITY_SCALAR = Constants.isDemo ? 0.10 : 1.0;
  private static final double ANGULAR_VELOCITY_SCALAR = Constants.isDemo ? 0.10 : 0.65;

  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          double linearMagnitude = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Square magnitude for more precise control
          linearMagnitude = linearMagnitude * linearMagnitude;

          Translation2d linearVelocity = Translation2d.kZero;

          if (linearMagnitude > 1e-6) {
            linearVelocity =
                new Translation2d(
                        linearMagnitude,
                        new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()))
                    .times(LINEAR_VELOCITY_SCALAR);
          }

          // Square rotation value for more precise control
          double omega =
              Math.copySign(
                  omegaSupplier.getAsDouble() * omegaSupplier.getAsDouble(),
                  omegaSupplier.getAsDouble());

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  new ChassisSpeeds(
                      linearVelocity.getX() * DriveConstants.maxSpeed,
                      linearVelocity.getY() * DriveConstants.maxSpeed,
                      omega * DriveConstants.maxAngularSpeed * ANGULAR_VELOCITY_SCALAR),
                  AllianceFlipUtil.apply(
                      RobotState.getInstance().getEstimatedPose().getRotation())),
              DriveMode.TELEOP);
        },
        drive);
  }

  private static final LoggedTunablePID controller =
      new LoggedTunablePID(3.0, 0.0, 0.1, Units.degreesToRadians(1.0), "DriveAtAngle");

  public static Command joystickDriveAtHeading(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> headingSupplier) {
    return Commands.run(
            () -> {
              double linearMagnitude = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Square magnitude for more precise control
              linearMagnitude = linearMagnitude * linearMagnitude;

              Translation2d linearVelocity = Translation2d.kZero;

              if (linearMagnitude > 1e-6) {
                linearVelocity =
                    new Translation2d(
                            linearMagnitude,
                            new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()))
                        .times(LINEAR_VELOCITY_SCALAR);
              }

              controller.enableContinuousInput(-Math.PI, Math.PI);

              double target = headingSupplier.get().getRadians();
              double error =
                  MathUtil.angleModulus(
                      RobotState.getInstance()
                          .getEstimatedPose()
                          .getRotation()
                          .minus(Rotation2d.fromRadians(headingSupplier.get().getRadians()))
                          .getRadians());

              double output = controller.calculate(error, 0.0);

              Logger.recordOutput("DriveAtAngle/TargetHeading", target);
              Logger.recordOutput("DriveAtAngle/Error", error);
              Logger.recordOutput("DriveAtAngle/Output", output);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(
                          linearVelocity.getX() * DriveConstants.maxSpeed,
                          linearVelocity.getY() * DriveConstants.maxSpeed,
                          output),
                      AllianceFlipUtil.apply(
                          RobotState.getInstance().getEstimatedPose().getRotation())),
                  DriveMode.TELEOP);
            },
            drive)
        // Reset PID controller when command starts
        .beforeStarting(() -> controller.reset());
  }
}
