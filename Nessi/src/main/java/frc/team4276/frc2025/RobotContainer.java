package frc.team4276.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.AutoSelector.AutoQuestion;
import frc.team4276.frc2025.AutoSelector.AutoQuestionResponse;
import frc.team4276.frc2025.commands.DriveCommands;
import frc.team4276.frc2025.commands.auto.AutoBuilder;
import frc.team4276.frc2025.subsystems.Superstructure;
import frc.team4276.frc2025.subsystems.Superstructure.CurrentSuperState;
import frc.team4276.frc2025.subsystems.Superstructure.WantedSuperState;
import frc.team4276.frc2025.subsystems.clopper.Clopper;
import frc.team4276.frc2025.subsystems.clopper.climber.ClimberIO;
import frc.team4276.frc2025.subsystems.clopper.climber.ClimberIOSparkMax;
import frc.team4276.frc2025.subsystems.clopper.hopper.HopperIO;
import frc.team4276.frc2025.subsystems.clopper.hopper.HopperIOSparkMax;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.elevator.Elevator;
import frc.team4276.frc2025.subsystems.elevator.ElevatorIO;
import frc.team4276.frc2025.subsystems.elevator.ElevatorIOSparkMax;
import frc.team4276.frc2025.subsystems.endeffector.EndEffector;
import frc.team4276.frc2025.subsystems.endeffector.EndEffectorIO;
import frc.team4276.frc2025.subsystems.endeffector.EndEffectorIOSparkMax;
import frc.team4276.frc2025.subsystems.toggles.TogglesIO;
import frc.team4276.frc2025.subsystems.toggles.TogglesIOHardware;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.frc2025.subsystems.vision.VisionIO;
import frc.team4276.frc2025.subsystems.vision.VisionIOPhotonVision;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.hid.CowsController;
import frc.team4276.util.hid.ViXController;
import frc.team4276.util.ios.GyroIO;
import frc.team4276.util.ios.GyroIOADIS;
import frc.team4276.util.ios.ModuleIO;
import frc.team4276.util.ios.ModuleIOSim;
import frc.team4276.util.ios.ModuleIOSpark;
import frc.team4276.util.ios.VisionIOPhotonVisionSim;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Elevator elevator;
  private EndEffector endEffector;
  private Clopper clopper;
  private TogglesIO toggles;

  private final Superstructure superstructure;
  private final AutoBuilder autoBuilder;

  // Controller
  private enum BindSetting {
    DEFAULT,
    DEMO,
    EXPERIMENTAL
  }

  private final BindSetting bindSetting = Constants.isDemo ? BindSetting.DEMO : BindSetting.DEFAULT;

  private final ViXController driver = new ViXController(0);
  private final CowsController demoController = new CowsController(1, 2);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);

  @AutoLogOutput private boolean disableVisionSim = false;

  // Dashboard inputs
  private final AutoSelector autoSelector = new AutoSelector();

  public RobotContainer() {
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getType()) {
        case COMPBOT -> {
          // Real robot, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOADIS(),
                  new ModuleIOSpark(0),
                  new ModuleIOSpark(1),
                  new ModuleIOSpark(2),
                  new ModuleIOSpark(3));
          vision =
              new Vision(
                  RobotState.getInstance()::addVisionMeasurement,
                  new VisionIOPhotonVision(0),
                  new VisionIOPhotonVision(1));
          elevator = new Elevator(new ElevatorIOSparkMax());
          endEffector =
              new EndEffector(
                  new EndEffectorIOSparkMax(
                      Ports.ENDEFFECTOR_LEFT, Ports.ENDEFFECTOR_RIGHT, 40, false, true));
          clopper =
              new Clopper(
                  new ClimberIOSparkMax(Ports.CLIMBER_WENCH, Ports.CLIMBER_WHEEL, 40, 40),
                  new HopperIOSparkMax(Ports.HOPPER_LEFT, true),
                  new HopperIOSparkMax(Ports.HOPPER_RIGHT, false));
          toggles = new TogglesIOHardware();
        }

        case SIMBOT -> {
          // Sim robot, instantiate physics sim IO implementations
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          if (disableVisionSim) {
            vision = new Vision(RobotState.getInstance()::addVisionMeasurement);
          } else {
            vision =
                new Vision(
                    RobotState.getInstance()::addVisionMeasurement,
                    new VisionIOPhotonVisionSim(0, RobotState.getInstance()::getEstimatedPose),
                    new VisionIOPhotonVisionSim(1, RobotState.getInstance()::getEstimatedPose));
          }
          elevator = new Elevator(new ElevatorIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          clopper = new Clopper(new ClimberIO() {}, new HopperIO() {}, new HopperIO() {});
          toggles = new TogglesIO() {};
        }
      }
    }

    // No-op implmentations for replay
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    if (clopper == null) {
      clopper = new Clopper(new ClimberIO() {}, new HopperIO() {}, new HopperIO() {});
    }

    if (vision == null) {
      vision =
          new Vision(
              RobotState.getInstance()::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    }

    if (toggles == null) {
      toggles = new TogglesIO() {};
    }

    superstructure = new Superstructure(drive, vision, elevator, endEffector, clopper, toggles);

    autoBuilder = new AutoBuilder(superstructure, autoSelector);
    configureAutos();
    configureButtonBindings();

    // Peace and quiet
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureAutos() {
    // Set up auto routines
    autoSelector.addRoutine(
        "RP Shrimple OCR Auto",
        List.of(
            new AutoQuestion(
                "Is Processor Side?", List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO))),
        () -> autoBuilder.rpShrimpleOcrAuto());
    autoSelector.addRoutine(
        "(FBAE) Shrimple OCR Auto",
        List.of(
            new AutoQuestion(
                "Is Processor Side?", List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO))),
        () ->
            autoBuilder.shrimpleOcrAuto(
                List.of(
                    AutoQuestionResponse.F,
                    AutoQuestionResponse.B,
                    AutoQuestionResponse.A,
                    AutoQuestionResponse.E)));
    autoSelector.addRoutine(
        "Sandy Eggos Auto",
        List.of(
            new AutoQuestion(
                "Is Processor Side?", List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO)),
            new AutoQuestion(
                "Is Sweaty Start?", List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO)),
            new AutoQuestion(
                "Is Close Intake?", List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO)),
            new AutoQuestion("[Empty Question]", List.of(AutoQuestionResponse.EMPTY)),
            new AutoQuestion("[Empty Question]", List.of(AutoQuestionResponse.EMPTY)),
            new AutoQuestion(
                "1st Reef?",
                List.of(
                    AutoQuestionResponse.EMPTY,
                    AutoQuestionResponse.E,
                    AutoQuestionResponse.F,
                    AutoQuestionResponse.G)),
            new AutoQuestion(
                "2nd Reef?",
                List.of(
                    AutoQuestionResponse.EMPTY,
                    AutoQuestionResponse.A,
                    AutoQuestionResponse.B,
                    AutoQuestionResponse.C,
                    AutoQuestionResponse.D,
                    AutoQuestionResponse.E,
                    AutoQuestionResponse.F)),
            new AutoQuestion(
                "3rd Reef?",
                List.of(
                    AutoQuestionResponse.EMPTY,
                    AutoQuestionResponse.A,
                    AutoQuestionResponse.B,
                    AutoQuestionResponse.C,
                    AutoQuestionResponse.D,
                    AutoQuestionResponse.E,
                    AutoQuestionResponse.F)),
            new AutoQuestion(
                "4th Reef?",
                List.of(
                    AutoQuestionResponse.EMPTY,
                    AutoQuestionResponse.A,
                    AutoQuestionResponse.B,
                    AutoQuestionResponse.C,
                    AutoQuestionResponse.D,
                    AutoQuestionResponse.E,
                    AutoQuestionResponse.F))),
        () -> autoBuilder.sandyEggosAuto());
    autoSelector.addRoutine(
        "Shrimple OCR Auto",
        List.of(
            new AutoQuestion(
                "Is Processor Side?", List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO)),
            new AutoQuestion(
                "1st Reef?",
                List.of(
                    AutoQuestionResponse.EMPTY,
                    AutoQuestionResponse.E,
                    AutoQuestionResponse.F,
                    AutoQuestionResponse.G)),
            new AutoQuestion(
                "2nd Reef?",
                List.of(
                    AutoQuestionResponse.EMPTY,
                    AutoQuestionResponse.A,
                    AutoQuestionResponse.B,
                    AutoQuestionResponse.C,
                    AutoQuestionResponse.D,
                    AutoQuestionResponse.E,
                    AutoQuestionResponse.F)),
            new AutoQuestion(
                "3rd Reef?",
                List.of(
                    AutoQuestionResponse.EMPTY,
                    AutoQuestionResponse.A,
                    AutoQuestionResponse.B,
                    AutoQuestionResponse.C,
                    AutoQuestionResponse.D,
                    AutoQuestionResponse.E,
                    AutoQuestionResponse.F)),
            new AutoQuestion(
                "4th Reef?",
                List.of(
                    AutoQuestionResponse.EMPTY,
                    AutoQuestionResponse.A,
                    AutoQuestionResponse.B,
                    AutoQuestionResponse.C,
                    AutoQuestionResponse.D,
                    AutoQuestionResponse.E,
                    AutoQuestionResponse.F))),
        () -> autoBuilder.shrimpleOcrAuto());
    autoSelector.addRoutine(
        "Taxi Wall",
        List.of(
            new AutoQuestion(
                "Is Processor Side?", List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO))),
        () -> autoBuilder.taxiAuto("t_WALL"));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    switch (bindSetting) {
      case DEFAULT:
        configureControllerBindings();
        break;

      case DEMO:
        configureDemoBindings();
        break;

      case EXPERIMENTAL:
        configureExperimentalBindings();
        break;

      default:
        break;
    }
  }

  private void configureControllerBindings() {
    DoubleSupplier driverX = () -> -driver.getLeftWithDeadband().y;
    DoubleSupplier driverY = () -> -driver.getLeftWithDeadband().x;
    DoubleSupplier driverOmega = () -> -driver.getRightWithDeadband().x;

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega));

    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    // Reef Scoring
    driver
        .rightTrigger()
        .and(driver.y().negate())
        .onTrue(
            superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_RIGHT_L3,
                Superstructure.WantedSuperState.INTAKE_CORAL,
                Superstructure.WantedSuperState.SCORE_RIGHT_L1))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .leftTrigger()
        .and(driver.y().negate())
        .onTrue(
            superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_LEFT_L3,
                Superstructure.WantedSuperState.INTAKE_CORAL,
                Superstructure.WantedSuperState.SCORE_LEFT_L1))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .rightBumper()
        .and(driver.y().negate())
        .onTrue(
            superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_RIGHT_L2,
                Superstructure.WantedSuperState.STOW,
                Superstructure.WantedSuperState.STOW))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .leftBumper()
        .and(driver.y().negate())
        .onTrue(
            superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_LEFT_L2,
                Superstructure.WantedSuperState.STOW,
                Superstructure.WantedSuperState.STOW))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .rightBumper()
        .and(driver.y())
        .onTrue(
            superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_RIGHT_L1,
                Superstructure.WantedSuperState.STOW,
                Superstructure.WantedSuperState.STOW))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .leftBumper()
        .and(driver.y())
        .onTrue(
            superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_LEFT_L1,
                Superstructure.WantedSuperState.STOW,
                Superstructure.WantedSuperState.STOW))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .a()
        .onTrue(
            superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.MANUAL_SCORE,
                Superstructure.WantedSuperState.PURGE_GAMEPIECE,
                Superstructure.WantedSuperState.MANUAL_SCORE))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    // Purge Gamepiece(s)
    driver
        .b()
        .onTrue(
            superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.PURGE_GAMEPIECE,
                Superstructure.WantedSuperState.PURGE_GAMEPIECE,
                Superstructure.WantedSuperState.PURGE_GAMEPIECE))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .x()
        .onTrue(superstructure.setStateCommand(WantedSuperState.REEF_ALGAE))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    // Climb
    driver
        .povUp()
        .onTrue(
            Commands.either(
                superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMB, true),
                superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMB_PREP, true),
                () -> superstructure.getCurrentSuperState() == CurrentSuperState.CLIMB_PREP));

    // L1 Mode
    driver.povRight().onTrue(Commands.runOnce(() -> superstructure.setL1ModeEnabled(true)));

    // Exit Mode
    driver
        .povDown()
        .onTrue(
            Commands.runOnce(() -> superstructure.setL1ModeEnabled(false))
                .alongWith(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW)));
  }

  private void configureDemoBindings() {
    DoubleSupplier driverX = () -> -demoController.getLeftWithDeadband().y;
    DoubleSupplier driverY = () -> -demoController.getLeftWithDeadband().x;
    DoubleSupplier driverOmega = () -> -demoController.getRightWithDeadband().x;

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega));

    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    driver
        .leftTrigger()
        .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_CORAL))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .a()
        .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.SCORE_LEFT_L1))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .x()
        .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.SCORE_LEFT_L2))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .b()
        .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.SCORE_LEFT_L3))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));

    driver
        .a()
        .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_CORAL))
        .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.STOW));
  }

  public void configureExperimentalBindings() {}

  public void update() {
    updateAlerts();

    SimManager.periodic();
  }

  private void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(
        Constants.isSim
            ? false
            : !DriverStation.isJoystickConnected(driver.getHID().getPort())
                || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.getCommand();
  }
}
