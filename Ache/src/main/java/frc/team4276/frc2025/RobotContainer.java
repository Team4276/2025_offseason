// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.subsystems.Superstructure;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.drive.GyroIO;
import frc.team4276.frc2025.subsystems.drive.GyroIOADIS;
import frc.team4276.frc2025.subsystems.drive.ModuleIO;
import frc.team4276.frc2025.subsystems.drive.ModuleIOSim;
import frc.team4276.frc2025.subsystems.drive.ModuleIOSpark;
import frc.team4276.frc2025.subsystems.elevator.Elevator;
import frc.team4276.frc2025.subsystems.elevator.ElevatorIO;
import frc.team4276.frc2025.subsystems.elevator.ElevatorIOSparkMax;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.frc2025.subsystems.vision.VisionIO;
import frc.team4276.frc2025.subsystems.vision.VisionIOPhotonVision;
import frc.team4276.lib.AllianceFlipUtil;
import frc.team4276.lib.hid.CowsController;
import frc.team4276.lib.hid.ViXController;

public class RobotContainer {
  private Drive drive;
  private Vision vision;
  private Elevator elevator;
  private Superstructure superstructure;

  private final ViXController driver = new ViXController(Ports.DRIVER_CONTROLLER);
  private final CowsController demoController = new CowsController(Ports.DEMO_CONTROLLER_LEFT,
      Ports.DEMO_CONTROLLER_RIGHT);

  public RobotContainer() {

    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getType()) {
        case COMPBOT -> {
          // Real robot, instantiate hardware IO implementations
          drive = new Drive(
              Constants.isDemo ? demoController : driver,
              new GyroIOADIS(),
              new ModuleIOSpark(0),
              new ModuleIOSpark(1),
              new ModuleIOSpark(2),
              new ModuleIOSpark(3));
          vision = new Vision(RobotState.getInstance()::addVisionMeasurement,
              new VisionIOPhotonVision(0, RobotState.getInstance()::getEstimatedPose),
              new VisionIOPhotonVision(1, RobotState.getInstance()::getEstimatedPose));
          elevator = new Elevator(new ElevatorIOSparkMax());
        }

        case SIMBOT -> {
          // Sim robot, instantiate physics sim IO implementations
          drive = new Drive(
              Constants.isDemo ? demoController : driver,
              new GyroIO() {
              },
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
          vision = new Vision(RobotState.getInstance()::addVisionMeasurement);
          elevator = new Elevator(new ElevatorIO() {

          });
        }
      }
    }

    // No-op implmentations for replay
    if (drive == null) {
      drive = new Drive(
          Constants.isDemo ? demoController : driver,
          new GyroIO() {
          },
          new ModuleIO() {
          },
          new ModuleIO() {
          },
          new ModuleIO() {
          },
          new ModuleIO() {
          });
    }

    if (vision == null) {
      vision = new Vision(RobotState.getInstance()::addVisionMeasurement, new VisionIO() {
      }, new VisionIO() {
      });
    }

    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {

      });
    }

    superstructure = new Superstructure(drive, vision, elevator, driver);

    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance()
                    .resetPose(
                        new Pose2d(
                            RobotState.getInstance().getEstimatedPose().getTranslation(),
                            AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    driver
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> superstructure.test()))
        .onFalse(Commands.runOnce(() -> superstructure.stow()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
