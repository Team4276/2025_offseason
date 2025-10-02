package frc.team4276.frc2025.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.ViXController;
import java.util.function.DoubleSupplier;

public class IntakeCommands {

  public static Command gamerIntake(
      Superstructure superstructure,
      Drive drive,
      ViXController driver,
      DoubleSupplier driverX,
      DoubleSupplier driverY) {
    return intake(superstructure, driver)
        .alongWith(
            DriveCommands.joystickDriveAtHeading(
                drive,
                driverX,
                driverY,
                () ->
                    AllianceFlipUtil.apply(
                        AllianceFlipUtil.applyY(RobotState.getInstance().getEstimatedPose().getY())
                                < FieldConstants.fieldWidth / 2
                            ? Rotation2d.fromDegrees(55.0)
                            : Rotation2d.fromDegrees(305.0))));
  }

  public static Command intakeAtAngle(
      Rotation2d angle,
      Superstructure superstructure,
      Drive drive,
      ViXController driver,
      DoubleSupplier driverX,
      DoubleSupplier driverY) {
    return intake(superstructure, driver)
        .alongWith(
            DriveCommands.joystickDriveAtHeading(
                drive, driverX, driverY, () -> AllianceFlipUtil.apply(angle)));
  }

  public static Command intake(Superstructure superstructure, ViXController driver) {
    return Commands.parallel(
            superstructure.setGoalCommand(Superstructure.Goal.INTAKE),
            Commands.waitUntil(superstructure::hasCoral)
                .andThen(driver.rumbleCommand(RumbleType.kBothRumble, 1.0, 1.0)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
}
