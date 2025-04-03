package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.field_calibration.FieldCalibrationUtil;
import frc.robot.autos.Autos;
import frc.robot.autos.Trailblazer;
import frc.robot.claw.ClawSubsystem;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_deploy.DeploySubsystem;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.ground_manager.GroundManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.ElasticLayoutUtil;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.game_piece_detection.CoralMap;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;

public class Robot extends TimedRobot {
  private Command autonomousCommand = Commands.none();
  private final FmsSubsystem fms = new FmsSubsystem();
  private final Hardware hardware = new Hardware();

  private final DeploySubsystem deploy = new DeploySubsystem(hardware.deployMotor);

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrainPigeon);
  private final Limelight leftBackLimelight =
      new Limelight("leftb", LimelightState.TAGS, LimelightModel.THREEG);
  private final Limelight leftFrontLimelight =
      new Limelight("leftf", LimelightState.TAGS, LimelightModel.THREEG);

  private final Limelight rightLimelight =
      new Limelight("right", LimelightState.TAGS, LimelightModel.FOUR);

  private final Limelight gamePieceDetectionLimelight =
      new Limelight("gp", LimelightState.CORAL, LimelightModel.THREE);

  private final VisionSubsystem vision =
      new VisionSubsystem(
          imu, leftBackLimelight, leftFrontLimelight, rightLimelight, gamePieceDetectionLimelight);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, vision, swerve);
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(hardware.elevatorLeftMotor, hardware.elevatorRightMotor);
  private final Trailblazer trailblazer = new Trailblazer(swerve, localization);
  private final RumbleControllerSubsystem rumbleController =
      new RumbleControllerSubsystem(hardware.driverController, true);

  private final ClawSubsystem claw = new ClawSubsystem(hardware.clawMotor, hardware.clawCandi);
  private final IntakeSubsystem intake =
      new IntakeSubsystem(hardware.intakeMotor, hardware.intakeCandi);

  private final ArmSubsystem arm = new ArmSubsystem(hardware.armMotor);
  private final LightsSubsystem lights = new LightsSubsystem(hardware.candle);
  private final ClimberSubsystem climber =
      new ClimberSubsystem(
          hardware.climberClimbMotor,
          hardware.climberCANcoder,
          hardware.climberGrabMotor,
          hardware.climberCanrange);
  private final AutoAlign autoAlign = new AutoAlign(vision, localization, swerve);
  private final CoralMap coralMap = new CoralMap(localization, swerve);
  private final GroundManager gm = new GroundManager(deploy, intake);
  private final RobotManager robotManager =
      new RobotManager(
          gm,
          claw,
          arm,
          elevator,
          vision,
          imu,
          swerve,
          localization,
          coralMap,
          lights,
          autoAlign,
          climber,
          rumbleController);
  private final FieldCalibrationUtil fieldCalibrationUtil =
      new FieldCalibrationUtil(elevator, arm, lights, localization);

  private final RobotCommands robotCommands = new RobotCommands(robotManager);
  private final Autos autos = new Autos(robotManager, trailblazer);

  public Robot() {
    System.out.println("roboRIO serial number: " + RobotConfig.SERIAL_NUMBER);

    DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

    SignalLogger.start();
    SignalLogger.setPath("/media/sda1/hoot/");

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withNtPublish(RobotConfig.IS_DEVELOPMENT)
            .withNtTunables(RobotConfig.IS_DEVELOPMENT));
    // DogLog.setPdh(hardware.pdh);

    // Record metadata
    DogLog.log("Metadata/ProjectName", BuildConstants.MAVEN_NAME);
    DogLog.log("Metadata/RoborioSerialNumber", RobotConfig.SERIAL_NUMBER);
    DogLog.log("Metadata/RobotName", RobotConfig.get().robotName());
    DogLog.log("Metadata/BuildDate", BuildConstants.BUILD_DATE);
    DogLog.log("Metadata/GitSHA", BuildConstants.GIT_SHA);
    DogLog.log("Metadata/GitDate", BuildConstants.GIT_DATE);
    DogLog.log("Metadata/GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0 -> DogLog.log("Metadata/GitDirty", "All changes committed");
      case 1 -> DogLog.log("Metadata/GitDirty", "Uncomitted changes");
      default -> DogLog.log("Metadata/GitDirty", "Unknown");
    }

    // This must be run before any commands are scheduled
    LifecycleSubsystemManager.ready();

    configureBindings();

    ElasticLayoutUtil.onBoot();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    DogLog.timeEnd("Scheduler/TimeSinceLastLoop");
    DogLog.time("Scheduler/TimeSinceLastLoop");

    DogLog.time("Scheduler/CommandSchedulerPeriodic");
    CommandScheduler.getInstance().run();
    DogLog.timeEnd("Scheduler/CommandSchedulerPeriodic");
    LifecycleSubsystemManager.log();

    if (RobotController.getBatteryVoltage() < 12.5) {
      DogLog.logFault("Battery voltage low", AlertType.kWarning);
    } else {
      DogLog.clearFault("Battery voltage low");
    }

    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      fieldCalibrationUtil.log();
    }
  }

  @Override
  public void disabledInit() {
    ElasticLayoutUtil.onDisable();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = autos.getAutoCommand();
    autonomousCommand.schedule();

    ElasticLayoutUtil.onEnable();
    autoAlign.clearReefState();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    autonomousCommand.cancel();

    ElasticLayoutUtil.onEnable();
    if (RobotConfig.IS_DEVELOPMENT) {
      autoAlign.clearReefState();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.run(
            () -> {
              if (DriverStation.isTeleop()) {
                swerve.driveTeleop(
                    hardware.driverController.getLeftX(),
                    hardware.driverController.getLeftY(),
                    hardware.driverController.getRightX());
              }
            }));

    hardware.driverController.rightTrigger().onTrue(robotCommands.confirmScoreCommand());
    hardware.driverController.leftTrigger().onTrue(robotCommands.floorIntakeCommand());
    hardware.driverController.leftBumper().onTrue(robotCommands.algaeIntakeGroundCommand());
    hardware.driverController.rightBumper().onTrue(robotCommands.stowCommand());
    hardware
        .driverController
        .y()
        .onTrue(robotCommands.highLineupCommand())
        .onFalse(robotCommands.scoringAlignOffCommand());
    hardware
        .driverController
        .x()
        .onTrue(robotCommands.l3LineupCommand())
        .onFalse(robotCommands.scoringAlignOffCommand());
    hardware
        .driverController
        .b()
        .onTrue(robotCommands.l2LineupCommand())
        .onFalse(robotCommands.scoringAlignOffCommand());
    hardware
        .driverController
        .a()
        .onTrue(robotCommands.lowLineupCommand())
        .onFalse(robotCommands.scoringAlignOffCommand());

    hardware.driverController.povUp().onTrue(robotCommands.climbUpCommand());
    hardware.driverController.povDown().onTrue(robotCommands.climbStopCommand());
    hardware
        .driverController
        .povRight()
        .onTrue(robotCommands.algaeReefIntakeCommand())
        .onFalse(robotCommands.scoringAlignOffCommand());

    hardware.driverController.start().onTrue(robotCommands.unjamCommand());
    hardware.driverController.back().onTrue(localization.getZeroCommand());

    hardware.operatorController.a().onTrue(robotCommands.rehomeElevatorCommand());
    hardware.operatorController.y().onTrue(robotCommands.rehomeDeployCommand());

    hardware.operatorController.back().onTrue(robotCommands.spinToWinCommand());

    // hardware.operatorController.rightTrigger().onTrue(robotCommands.waitAlgaeFlingCommand());
    hardware.operatorController.leftTrigger().onTrue(robotCommands.testNextLollipopCommand());
  }
}
