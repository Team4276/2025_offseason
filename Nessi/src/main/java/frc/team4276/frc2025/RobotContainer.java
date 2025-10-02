package frc.team4276.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team4276.frc2025.AutoSelector.AutoQuestion;
import frc.team4276.frc2025.AutoSelector.AutoQuestionResponse;
import frc.team4276.frc2025.commands.AutoScore;
import frc.team4276.frc2025.commands.DriveCommands;
import frc.team4276.frc2025.commands.DriveToPose;
import frc.team4276.frc2025.commands.FeedForwardCharacterization;
import frc.team4276.frc2025.commands.IntakeCommands;
import frc.team4276.frc2025.commands.WheelRadiusCharacterization;
import frc.team4276.frc2025.commands.auto.AutoBuilder;
import frc.team4276.frc2025.subsystems.climber.Climber;
import frc.team4276.frc2025.subsystems.climber.ClimberIO;
import frc.team4276.frc2025.subsystems.climber.ClimberIOSparkMax;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.hopper.Hopper;
import frc.team4276.frc2025.subsystems.hopper.HopperIO;
import frc.team4276.frc2025.subsystems.hopper.HopperIOSparkMax;
import frc.team4276.frc2025.subsystems.superstructure.RollerSensorsIO;
import frc.team4276.frc2025.subsystems.superstructure.RollerSensorsIOHardware;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.superstructure.displacer.Displacer;
import frc.team4276.frc2025.subsystems.superstructure.elevator.Elevator;
import frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorIO;
import frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorIOSparkMax;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffector;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffectorIO;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffectorIOSparkMax;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.frc2025.subsystems.vision.VisionIO;
import frc.team4276.frc2025.subsystems.vision.VisionIOPhotonVision;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.ViXController;
import frc.team4276.util.dashboard.ElasticUI;
import frc.team4276.util.ios.GyroIO;
import frc.team4276.util.ios.GyroIOADIS;
import frc.team4276.util.ios.ModuleIO;
import frc.team4276.util.ios.ModuleIOSim;
import frc.team4276.util.ios.ModuleIOSpark;
import frc.team4276.util.ios.RollerIO;
import frc.team4276.util.ios.RollerIOSparkMax;
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
  private Superstructure superstructure;
  private Hopper hopper;
  private Climber climber;
  private Vision vision;

  private AutoBuilder autoBuilder;

  // Controller
  private enum BindSetting {
    DEFAULT,
    DEMO,
    EXPERIMENTAL,
    TEST
  }

  private final BindSetting bindSetting = Constants.isDemo ? BindSetting.DEMO : BindSetting.TEST;

  private final ViXController driver = new ViXController(0);
  private final CommandGenericHID buttonBoard = new CommandGenericHID(1);
  private final ViXController operator = new ViXController(2);
  private final Joystick demoStickRight = new Joystick(3);
  private final Joystick demoStickLeft = new Joystick(4);

  private final ScoringHelper scoringHelper = new ScoringHelper(buttonBoard, operator);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);

  // Overrides
  private final DigitalInput elevatorCoastOverride =
      new DigitalInput(Ports.ELEVATOR_COAST_OVERRIDE);
  private final DigitalInput climberCoastOverride = new DigitalInput(Ports.CLIMBER_COAST_OVERRIDE);
  private final DigitalInput hopperCoastOverride = new DigitalInput(Ports.HOPPER_COAST_OVERRIDE);
  private final DigitalInput armCoastOverride = new DigitalInput(Ports.ARM_COAST_OVERRIDE);

  // Coral Scoring Logic
  @AutoLogOutput private boolean disableHeadingAutoAlign = true;
  @AutoLogOutput private boolean disableTranslationAutoAlign = true;
  @AutoLogOutput private boolean disableVisionSim = false;

  // Dashboard inputs
  private final AutoSelector autoSelector = new AutoSelector();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
          superstructure =
              new Superstructure(
                  new Elevator(new ElevatorIOSparkMax()),
                  new EndEffector(
                      new EndEffectorIOSparkMax(
                          Ports.ENDEFFECTOR_LEFT, Ports.ENDEFFECTOR_RIGHT, 40, false, true),
                      new RollerSensorsIOHardware()),
                  new Displacer(new RollerIOSparkMax(Ports.ALGAE_DISPLACER, 20, false, true)));
          hopper =
              new Hopper(
                  new HopperIOSparkMax(Ports.HOPPER_LEFT, true),
                  new HopperIOSparkMax(Ports.HOPPER_RIGHT, false));
          climber =
              new Climber(new ClimberIOSparkMax(Ports.CLIMBER_WENCH, Ports.CLIMBER_WHEEL, 40, 40));
          vision =
              new Vision(
                  RobotState.getInstance()::addVisionMeasurement,
                  new VisionIOPhotonVision(0),
                  new VisionIOPhotonVision(1)
                  // ,
                  // new VisionIOPhotonVision(2)
                  );
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
          superstructure =
              new Superstructure(
                  new Elevator(new ElevatorIO() {}),
                  new EndEffector(new EndEffectorIO() {}, new RollerSensorsIO() {}),
                  new Displacer(new RollerIO() {}));
          hopper = new Hopper(new HopperIO() {}, new HopperIO() {});
          climber = new Climber(new ClimberIO() {});
          if (disableVisionSim) {
            vision = new Vision(RobotState.getInstance()::addVisionMeasurement);
          } else {
            vision =
                new Vision(
                    RobotState.getInstance()::addVisionMeasurement,
                    new VisionIOPhotonVisionSim(0, RobotState.getInstance()::getEstimatedPose),
                    new VisionIOPhotonVisionSim(1, RobotState.getInstance()::getEstimatedPose));
          }
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

    if (vision == null) {
      vision =
          new Vision(
              RobotState.getInstance()::addVisionMeasurement, new VisionIO() {}, new VisionIO() {}
              // ,
              // new VisionIO() {}
              );
    }

    if (superstructure == null) {
      superstructure =
          new Superstructure(
              new Elevator(new ElevatorIO() {}),
              new EndEffector(new EndEffectorIO() {}, new RollerSensorsIO() {}),
              new Displacer(new RollerIO() {}));
    }

    if (hopper == null) {
      hopper = new Hopper(new HopperIO() {}, new HopperIO() {});
    }

    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }

    configureOverrides();
    configureAutos();
    if (Constants.isTuning) {
      configureTuningRoutines();
    }
    configureButtonBindings();
    configureUI();

    // Peace and quiet
    DriverStation.silenceJoystickConnectionWarning(true);

    calibrationBuffer.reset();
  }

  private void configureOverrides() {
    superstructure.setCoastOverride(elevatorCoastOverride::get);
    climber.setCoastOverride(climberCoastOverride::get);
    hopper.setCoastOverride(hopperCoastOverride::get);
  }

  private void configureAutos() {
    autoBuilder = new AutoBuilder(drive, superstructure, autoSelector);
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

  private void configureTuningRoutines() {
    // Set up SysId routines
    autoSelector.addRoutine("Box Test", () -> autoBuilder.testTraj("z_BoxTest"));
    autoSelector.addRoutine(
        "Drive Wheel Radius Characterization", () -> new WheelRadiusCharacterization(drive));
    autoSelector.addRoutine(
        "Drive Simple FF Characterization",
        () ->
            new FeedForwardCharacterization(
                drive, drive::runCharacterization, drive::getFFCharacterizationVelocity));
    autoSelector.addRoutine(
        "Drive SysId (Quasistatic Forward)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoSelector.addRoutine(
        "Drive SysId (Quasistatic Reverse)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoSelector.addRoutine(
        "Drive SysId (Dynamic Forward)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoSelector.addRoutine(
        "Drive SysId (Dynamic Reverse)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoSelector.addRoutine(
        "Elevator Simple FF Characterization",
        () ->
            new FeedForwardCharacterization(
                superstructure,
                superstructure::acceptCharacterizationInput,
                superstructure::getFFCharacterizationVelocity));
    autoSelector.addRoutine(
        "(Reverse) Elevator Simple FF Characterization",
        () ->
            new FeedForwardCharacterization(
                superstructure,
                superstructure::acceptCharacterizationInput,
                superstructure::getFFCharacterizationVelocity,
                true));
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

      case TEST:
        configureTestBinds();
        break;

      default:
        break;
    }
  }

  private void configureDemoBindings() {
    /***************** Drive Triggers *****************/
    // Drive suppliers
    DoubleSupplier driverX = () -> -demoStickLeft.getY();
    DoubleSupplier driverY = () -> -demoStickLeft.getX();
    DoubleSupplier driverOmega = () -> -demoStickRight.getX();

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega));

    driver
        .a()
        .and(driver.povUp())
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    operator.leftTrigger().whileTrue(superstructure.setGoalCommand(Superstructure.Goal.INTAKE));

    operator
        .a()
        .and(driver.povUp().negate())
        .toggleOnTrue(superstructure.setGoalCommand(Superstructure.Goal.L1));

    operator.x().toggleOnTrue(superstructure.setGoalCommand(Superstructure.Goal.L2));

    operator.b().toggleOnTrue(superstructure.setGoalCommand(Superstructure.Goal.L3));

    operator.rightTrigger().whileTrue(superstructure.scoreCommand(false));
  }

  private void configureControllerBindings() {
    /***************** Drive Triggers *****************/
    // Drive suppliers
    DoubleSupplier driverX = () -> -driver.getLeftWithDeadband().y;
    DoubleSupplier driverY = () -> -driver.getLeftWithDeadband().x;
    DoubleSupplier driverOmega = () -> -driver.getRightWithDeadband().x;

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega));

    // Reset gyro to 0° when A button is pressed
    driver
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    /***************** Coral Triggers *****************/

    // Intake
    driver
        .x()
        .and(() -> !disableHeadingAutoAlign)
        .whileTrue(
            IntakeCommands.intakeAtAngle(
                Rotation2d.fromDegrees(305), superstructure, drive, driver, driverX, driverY));

    driver
        .b()
        .and(() -> !disableHeadingAutoAlign)
        .whileTrue(
            IntakeCommands.intakeAtAngle(
                Rotation2d.fromDegrees(55), superstructure, drive, driver, driverX, driverY));

    driver
        .x()
        .and(() -> disableHeadingAutoAlign)
        .whileTrue(IntakeCommands.intake(superstructure, driver));

    driver
        .b()
        .and(() -> disableHeadingAutoAlign)
        .whileTrue(IntakeCommands.intake(superstructure, driver));

    // Scoring
    driver
        .rightTrigger()
        .and(() -> disableHeadingAutoAlign)
        .whileTrue(superstructure.setGoalCommand(scoringHelper::getSuperstructureGoal));

    driver
        .rightTrigger()
        .and(
            () ->
                (disableTranslationAutoAlign && !disableHeadingAutoAlign)
                    || scoringHelper.getSuperstructureGoal() == Superstructure.Goal.L1)
        .whileTrue(
            DriveCommands.joystickDriveAtHeading(
                    drive,
                    driverX,
                    driverY,
                    () -> scoringHelper.getSelectedReef().getScore().getRotation())
                .alongWith(superstructure.setGoalCommand(scoringHelper::getSuperstructureGoal)));

    driver
        .rightTrigger()
        .and(
            () ->
                !disableTranslationAutoAlign
                    && !disableHeadingAutoAlign
                    && scoringHelper.getSuperstructureGoal() != Superstructure.Goal.L1)
        .whileTrue(
            AutoScore.coralAlignCommand(drive, driverX, driverY, superstructure, scoringHelper)
                .alongWith(
                    Commands.waitUntil(
                            () ->
                                superstructure.getGoal() != Superstructure.Goal.STOW
                                    && superstructure.atGoal()
                                    && DriveToPose.atGoal())
                        .andThen(driver.rumbleCommand(RumbleType.kBothRumble, 1.0, 0.2, 3))));

    driver.rightBumper().and(driver.rightTrigger()).whileTrue(superstructure.scoreCommand(false));

    driver.leftBumper().and(driver.rightTrigger()).whileTrue(superstructure.scoreCommand(true));

    // Modal
    driver
        .povRight()
        .onTrue(Commands.runOnce(() -> disableTranslationAutoAlign = !disableTranslationAutoAlign));

    driver
        .povLeft()
        .onTrue(Commands.runOnce(() -> disableHeadingAutoAlign = !disableHeadingAutoAlign));

    // Misc
    driver
        .rightBumper()
        .and(() -> !driver.getRT())
        .whileTrue(superstructure.setGoalCommand(Superstructure.Goal.SHUFFLE));

    /***************** Algae Triggers *****************/

    // Displacing
    driver
        .y()
        .and(
            () ->
                scoringHelper.getSuperstructureGoal() == Superstructure.Goal.L2
                    || scoringHelper.getSuperstructureGoal() == Superstructure.Goal.L1)
        .and(() -> superstructure.getGoal() != Superstructure.Goal.L3)
        .toggleOnTrue(superstructure.setGoalCommand(Superstructure.Goal.LO_ALGAE));

    driver
        .y()
        .and(
            () ->
                !(superstructure.getGoal() == Superstructure.Goal.L2
                    || superstructure.getGoal() == Superstructure.Goal.L1))
        .and(() -> scoringHelper.getSuperstructureGoal() == Superstructure.Goal.L3)
        .toggleOnTrue(superstructure.setGoalCommand(Superstructure.Goal.HI_ALGAE));

    /***************** Climbing Triggers *****************/

    driver
        .povUp()
        .toggleOnTrue(
            climber
                .climbCommand()
                .alongWith(hopper.setGoalCommand(Hopper.Goal.CLIMB))
                .alongWith(superstructure.setGoalCommand(Superstructure.Goal.CLIMB))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    driver
        .leftTrigger()
        .and(() -> climber.isClimbing())
        .whileTrue(climber.setGoalCommand(Climber.Goal.RAISE));

    driver
        .rightTrigger()
        .and(() -> climber.isClimbing())
        .whileTrue(climber.setGoalCommand(Climber.Goal.CLIMB));
  }

  public void configureExperimentalBindings() {
    /***************** Drive Triggers *****************/
    // Drive suppliers
    DoubleSupplier driverX = () -> -driver.getLeftWithDeadband().y;
    DoubleSupplier driverY = () -> -driver.getLeftWithDeadband().x;
    DoubleSupplier driverOmega = () -> -driver.getRightWithDeadband().x;

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega));

    // Reset gyro to 0° when A button is pressed
    driver
        .a()
        .and(driver.povDown())
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    /***************** Coral Triggers *****************/
    // Intake
    driver
        .rightTrigger()
        .and(() -> !superstructure.hasCoral())
        .whileTrue(IntakeCommands.gamerIntake(superstructure, drive, driver, driverX, driverY));

    driver
        .leftTrigger()
        .and(() -> !superstructure.hasCoral())
        .whileTrue(IntakeCommands.intake(superstructure, driver));

    // Align and Score / Lock
    driver
        .a()
        .and(driver.povDown().negate())
        .toggleOnTrue(superstructure.setGoalCommand(Superstructure.Goal.L1));

    driver
        .b()
        .and(driver.rightTrigger().or(driver.leftTrigger()))
        .onTrue(AutoScore.selectAndScoreCommand(superstructure, Superstructure.Goal.L2));

    driver
        .x()
        .and(driver.rightTrigger().or(driver.leftTrigger()))
        .onTrue(AutoScore.selectAndScoreCommand(superstructure, Superstructure.Goal.L3));

    driver
        .rightTrigger()
        .and(() -> superstructure.hasCoral())
        .whileTrue(
            AutoScore.coralAlignCommand(
                drive, driverX, driverY, driverOmega, false, superstructure));

    driver
        .leftTrigger()
        .and(() -> superstructure.hasCoral())
        .whileTrue(
            AutoScore.coralAlignCommand(
                drive, driverX, driverY, driverOmega, true, superstructure));

    // Scoring for L1
    driver
        .rightTrigger()
        .and(() -> superstructure.getGoal() == Superstructure.Goal.L1)
        .whileTrue(superstructure.scoreCommand(false));

    driver
        .leftTrigger()
        .and(() -> superstructure.getGoal() == Superstructure.Goal.L1)
        .whileTrue(superstructure.scoreCommand(true));

    // Misc
    driver
        .povRight()
        .and(() -> superstructure.hasCoral())
        .whileTrue(superstructure.setGoalCommand(Superstructure.Goal.SHUFFLE));

    /***************** Algae Triggers *****************/
    // Displacing
    driver.leftBumper().toggleOnTrue(superstructure.setGoalCommand(Superstructure.Goal.LO_ALGAE));

    driver.rightBumper().toggleOnTrue(superstructure.setGoalCommand(Superstructure.Goal.HI_ALGAE));

    /***************** Climbing Triggers *****************/
    driver
        .povUp()
        .toggleOnTrue(
            climber
                .climbCommand()
                .alongWith(hopper.setGoalCommand(Hopper.Goal.CLIMB))
                .alongWith(superstructure.setGoalCommand(Superstructure.Goal.CLIMB))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    driver
        .leftTrigger()
        .and(() -> climber.isClimbing())
        .whileTrue(climber.setGoalCommand(Climber.Goal.RAISE));

    driver
        .rightTrigger()
        .and(() -> climber.isClimbing())
        .whileTrue(climber.setGoalCommand(Climber.Goal.CLIMB));
  }

  private void configureTestBinds() {
    /***************** Drive Triggers *****************/
    // Drive suppliers
    DoubleSupplier driverX = () -> -driver.getLeftWithDeadband().y;
    DoubleSupplier driverY = () -> -driver.getLeftWithDeadband().x;
    DoubleSupplier driverOmega = () -> -driver.getRightWithDeadband().x;

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega));

    driver
        .a()
        .and(driver.povUp())
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
        .povUp()
        .toggleOnTrue(
            climber
                .climbCommand()
                .alongWith(hopper.setGoalCommand(Hopper.Goal.CLIMB))
                .alongWith(superstructure.setGoalCommand(Superstructure.Goal.CLIMB))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    driver
        .leftTrigger()
        .and(() -> climber.isClimbing())
        .whileTrue(climber.setGoalCommand(Climber.Goal.RAISE));

    driver
        .rightTrigger()
        .and(() -> climber.isClimbing())
        .whileTrue(climber.setGoalCommand(Climber.Goal.CLIMB));
  }

  public void configureUI() {
    ElasticUI.setAlignToggleSuppliers(
        () -> disableHeadingAutoAlign, () -> disableTranslationAutoAlign);
  }

  public void setToggles(boolean disableHeadingAutoAlign, boolean disableTranslationAutoAlign) {
    this.disableHeadingAutoAlign = disableHeadingAutoAlign;
    this.disableTranslationAutoAlign = disableTranslationAutoAlign;
  }

  private boolean prevArmCoastState = false;
  private Timer calibrationBuffer = new Timer();

  public void update() {
    updateAlerts();

    SimManager.periodic();

    if (armCoastOverride.get() == false && armCoastOverride.get() != prevArmCoastState) {
      calibrationBuffer.restart();
    }

    if (calibrationBuffer.isRunning() && calibrationBuffer.advanceIfElapsed(1.0)) {
      drive.calibrate();
      calibrationBuffer.stop();
      calibrationBuffer.reset();
    }

    prevArmCoastState = armCoastOverride.get();
  }

  private void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(
        Constants.isSim
            ? false
            : !DriverStation.isJoystickConnected(driver.getHID().getPort())
                || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected.set(
        Constants.isSim
            ? false
            : !DriverStation.isJoystickConnected(buttonBoard.getHID().getPort()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector
        .getCommand()
        .beforeStarting(
            () -> {
              vision.setCamerasEnabled(true, true, true);
              // drive.calibrate();
            })
        .finallyDo(() -> vision.setCamerasEnabled(true, true, true));
  }
}
