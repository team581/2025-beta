package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefAlignState;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefSide;
import frc.robot.claw.ClawState;
import frc.robot.claw.ClawSubsystem;
import frc.robot.climber.ClimberState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.FeatureFlags;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake_assist.IntakeAssistUtil;
import frc.robot.lights.LightsState;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.collision_avoidance.CollisionAvoidance;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.game_piece_detection.CoralMap;

import java.util.Optional;

public class RobotManager extends StateMachine<RobotState> {
  public final LocalizationSubsystem localization;

  public final VisionSubsystem vision;
  public final ImuSubsystem imu;

  public final CoralMap coralMap;
  private final SwerveSubsystem swerve;
  public final ClawSubsystem intake;
  public final ArmSubsystem arm;
  public final ElevatorSubsystem elevator;
  public final ClimberSubsystem climber;
  public final RumbleControllerSubsystem rumbleController;

  private final LightsSubsystem lights;

  public final AutoAlign autoAlign;

  private boolean algaeMode = false;

  public RobotManager(
      ClawSubsystem intake,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      VisionSubsystem vision,
      ImuSubsystem imu,
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      CoralMap coralMap,
      LightsSubsystem lights,
      AutoAlign autoAlign,
      ClimberSubsystem climber,
      RumbleControllerSubsystem rumbleController) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.intake = intake;
    this.arm = arm;
    this.elevator = elevator;
    this.vision = vision;
    this.imu = imu;
    this.swerve = swerve;
    this.localization = localization;
    this.coralMap = coralMap;
    this.lights = lights;
    this.climber = climber;
    this.autoAlign = autoAlign;
    this.rumbleController = rumbleController;
  }

  private double reefSnapAngle = 0.0;
  private double coralIntakeAssistAngle = 0.0;
  private Optional<Pose2d> maybeBestCoralMapTranslation = Optional.empty();
  private ReefSide nearestReefSide = ReefSide.SIDE_GH;
  private ReefPipeLevel scoringLevel = ReefPipeLevel.BASE;
  private boolean isRollHomed = false;
  private boolean confirmScoreActive = false;

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case IDLE_NO_GP,
              IDLE_ALGAE,
              IDLE_CORAL,
              PROCESSOR_WAITING,
              NET_BACK_WAITING,
              NET_FORWARD_WAITING,
              CORAL_L1_3_PLACE,
              CORAL_CENTERED_L2_3_PLACE,
              CORAL_CENTERED_L3_3_PLACE,
              CORAL_CENTERED_L4_3_PLACE,
              CORAL_DISPLACED_L2_3_PLACE,
              CORAL_DISPLACED_L3_3_PLACE,
              CORAL_DISPLACED_L4_3_PLACE,
              CLIMBING_2_HANGING,
              CLIMBING_3_HANGING_2,
              CLIMBING_4_HANGING_3,
              UNJAM_CORAL_STATION,
              UNJAM ->
          currentState;

      case REHOME_ELEVATOR ->
          elevator.getState() == ElevatorState.STOWED ? RobotState.IDLE_NO_GP : currentState;

      case PROCESSOR_PREPARE_TO_SCORE ->
          arm.atGoal() && elevator.atGoal() ? RobotState.PROCESSOR_SCORING : currentState;
      case NET_BACK_PREPARE_TO_SCORE ->
          arm.atGoal() && elevator.atGoal() ? RobotState.NET_BACK_SCORING : currentState;
      case NET_FORWARD_PREPARE_TO_SCORE ->
          arm.atGoal() && elevator.atGoal() ? RobotState.NET_FORWARD_SCORING : currentState;

      case CORAL_L2_1_APPROACH -> {
        var isClose =
            AutoAlign.isCloseToReefSide(
                localization.getPose(), nearestReefSide.getPose(), swerve.getFieldRelativeSpeeds());

        if (!isClose) {
          yield currentState;
        }

        yield RobotState.CORAL_CENTERED_L2_2_LINEUP;
      }
      case PREPARE_UNJAM_CORAL_STATION ->
          elevator.atGoal() && arm.atGoal() ? RobotState.UNJAM_CORAL_STATION : currentState;
      case CORAL_CENTERED_L2_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_CENTERED_L2_3_PLACE : currentState;
      case CORAL_CENTERED_L3_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_CENTERED_L3_3_PLACE : currentState;
      case CORAL_CENTERED_L4_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_CENTERED_L4_3_PLACE : currentState;

      case CORAL_DISPLACED_L2_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_DISPLACED_L2_3_PLACE : currentState;
      case CORAL_DISPLACED_L3_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_DISPLACED_L3_3_PLACE : currentState;
      case CORAL_DISPLACED_L4_2_LINEUP ->
          shouldProgressTeleopScore() ? RobotState.CORAL_DISPLACED_L4_3_PLACE : currentState;

      case CORAL_L3_1_APPROACH -> {
        var isClose =
            AutoAlign.isCloseToReefSide(
                localization.getPose(), nearestReefSide.getPose(), swerve.getFieldRelativeSpeeds());

        if (!isClose) {
          yield currentState;
        }

        yield RobotState.CORAL_CENTERED_L3_2_LINEUP;
      }
      case CORAL_L4_1_APPROACH -> {
        // Require explicit transition in auto
        if (FeatureFlags.EXPLICIT_L4_LINEUP.getAsBoolean() && DriverStation.isAutonomous()) {
          yield currentState;
        }
        var isClose =
            AutoAlign.isCloseToReefSide(
                localization.getPose(), nearestReefSide.getPose(), swerve.getFieldRelativeSpeeds());

        if (!isClose) {
          yield currentState;
        }

        // Temporary workaround for avoiding tipping into the reef during auto
        if (DriverStation.isAutonomous()) {
          yield RobotState.CORAL_CENTERED_L4_2_LINEUP;
        }

        yield RobotState.CORAL_CENTERED_L4_1_POINT_5_RAISE_ARM;
      }

      case CORAL_CENTERED_L4_1_POINT_5_RAISE_ARM ->
          arm.atGoal() && elevator.atGoal()
              ? RobotState.CORAL_CENTERED_L4_2_LINEUP
              : currentState;
      case CORAL_DISPLACED_L4_1_POINT_5_RAISE_ARM ->
          arm.atGoal() && elevator.atGoal()
              ? RobotState.CORAL_DISPLACED_L4_2_LINEUP
              : currentState;

      // Scoring
      case PROCESSOR_SCORING, NET_FORWARD_SCORING, ALGAE_OUTTAKE -> {
        if (timeout(0.5)) {
          algaeMode = false;
          yield RobotState.IDLE_NO_GP;
        }
        yield currentState;
      }
      case CORAL_L1_4_RELEASE,
          CORAL_CENTERED_L2_4_RELEASE,
          CORAL_CENTERED_L3_4_RELEASE,
          CORAL_CENTERED_L4_4_RELEASE,
          CORAL_DISPLACED_L2_4_RELEASE,
          CORAL_DISPLACED_L3_4_RELEASE,
          CORAL_DISPLACED_L4_4_RELEASE -> {
        var done =
            arm.atGoal()
                && elevator.atGoal()
                && (DriverStation.isTeleop() ? cameraOnlineAndFarEnoughFromReef() : timeout(0.5));

        if (done) {
          rumbleController.rumbleRequest();
          yield RobotState.IDLE_NO_GP;
        }

        yield currentState;
      }

      case CORAL_CENTERED_L4_3_PLACE_THEN_RELEASE ->
          arm.atGoal() && elevator.atGoal()
              ? RobotState.CORAL_CENTERED_L4_4_RELEASE
              : currentState;

      case CORAL_DISPLACED_L4_3_PLACE_THEN_RELEASE ->
          arm.atGoal() && elevator.atGoal()
              ? RobotState.CORAL_DISPLACED_L4_4_RELEASE
              : currentState;

      // Intaking
      case INTAKE_ALGAE_FLOOR -> {
        if (intake.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.IDLE_ALGAE;
        }

        yield currentState;
      }
      case INTAKE_ALGAE_L2, INTAKE_ALGAE_L3 -> {
        if (intake.getHasGP()) {
          rumbleController.rumbleRequest();
          if (cameraOnlineAndFarEnoughFromReef()) {
            yield RobotState.IDLE_ALGAE;
          }
        }

        yield currentState;
      }

      case INTAKE_CORAL_FLOOR_HORIZONTAL,
          INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL,
          INTAKE_CORAL_FLOOR_UPRIGHT -> {
        if (intake.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.IDLE_CORAL;
        }

        yield currentState;
      }
      case INTAKE_STATION_APPROACH -> {
        if (Units.radiansToDegrees(Math.abs(swerve.getTeleopSpeeds().omegaRadiansPerSecond))
            < 15.0) {
          if (AutoAlign.shouldIntakeStationFront(localization.getPose())) {
            yield RobotState.INTAKE_CORAL_STATION_FRONT;
          }
          yield RobotState.INTAKE_CORAL_STATION_BACK;
        }
        yield currentState;
      }
      case INTAKE_CORAL_STATION_FRONT -> {
        if (intake.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.IDLE_CORAL;
        }
        yield currentState;
      }
      case INTAKE_CORAL_STATION_BACK -> {
        if (intake.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.SMART_STOW_1;
        }
        yield currentState;
      }
      case SMART_STOW_1 -> elevator.atGoal() ? RobotState.SMART_STOW_2 : currentState;
      case SMART_STOW_2 -> arm.atGoal() ? RobotState.IDLE_CORAL : currentState;
      case NET_BACK_SCORING -> intake.getHasGP() ? currentState : RobotState.IDLE_NO_GP;
      case CLIMBING_1_LINEUP ->
          climber.holdingCage() ? RobotState.CLIMBING_2_HANGING : currentState;
      default -> throw new IllegalArgumentException("Unexpected value: " + currentState);
    };
  }

  private boolean shouldProgressTeleopScore() {
    return DriverStation.isTeleop()
        && confirmScoreActive
        && autoAlign.getReefAlignState() == ReefAlignState.HAS_TAGS_IN_POSITION;
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        intake.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CORAL_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(
            algaeMode ? LightsState.IDLE_NO_GP_ALGAE_MODE : LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case IDLE_ALGAE -> {
        intake.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.STOWED, ArmState.ALGAE_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case IDLE_CORAL -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CORAL_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_CORAL);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_ALGAE_FLOOR -> {
        intake.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.GROUND_ALGAE_INTAKE, ArmState.GROUND_ALGAE_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_ALGAE_L2 -> {
        intake.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_L2);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_ALGAE_L3 -> {
        intake.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_L3);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.IDLE_NO_GP_ALGAE_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_STATION_APPROACH -> {
        intake.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CORAL_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.STATION_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_CORAL_STATION_BACK -> {
        intake.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(
            ElevatorState.INTAKING_CORAL_STATION_BACK, ArmState.INTAKING_CORAL_STATION_BACK);
        swerve.snapsDriveRequest(SnapUtil.getCoralStationAngle(localization.getPose()), true);
        vision.setState(VisionState.STATION_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_CORAL_STATION_FRONT -> {
        intake.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(
            ElevatorState.INTAKING_CORAL_STATION_FRONT, ArmState.INTAKING_CORAL_STATION_FRONT);
        swerve.snapsDriveRequest(
            SnapUtil.getCoralStationAngle(localization.getPose()) - 180.0, true);
        vision.setState(VisionState.STATION_TAGS);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case SMART_STOW_1 -> {
        intake.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.STOWED, ArmState.SMART_STOW_1);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_CORAL);
        climber.setState(ClimberState.STOWED);
      }
      case SMART_STOW_2 -> {
        intake.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CORAL_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_CORAL);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_CORAL_FLOOR_UPRIGHT -> {
        intake.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.GROUND_CORAL_INTAKE, ArmState.GROUND_CORAL_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CORAL_DETECTION);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_CORAL_FLOOR_HORIZONTAL -> {
        intake.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.GROUND_CORAL_INTAKE, ArmState.GROUND_CORAL_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CORAL_DETECTION);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL -> {
        intake.setState(ClawState.CORAL_HANDOFF);
        moveSuperstructure(ElevatorState.GROUND_CORAL_INTAKE, ArmState.GROUND_CORAL_INTAKE);
        // Enable assist in periodic if there's coral in map
        swerve.normalDriveRequest();
        vision.setState(VisionState.CORAL_DETECTION);
        lights.setState(LightsState.IDLE_NO_GP_CORAL_MODE);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L2_1_APPROACH -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L2_APPROACH, ArmState.CORAL_STOWED);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L3_1_APPROACH -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L3_APPROACH, ArmState.CORAL_STOWED);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L4_1_APPROACH -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L4_APPROACH, ArmState.CORAL_STOWED);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_3_PLACE -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L1_PLACE, ArmState.CORAL_SCORE_PLACING_L1, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_4_RELEASE -> {
        intake.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(ElevatorState.CORAL_L1_RELEASE, ArmState.CORAL_SCORE_PLACING_L1, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L2_2_LINEUP -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L2_LINEUP, ArmState.CORAL_SCORE_CENTERED_LINEUP_L2);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L2_3_PLACE -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L2_PLACE,
            ArmState.CORAL_SCORE_CENTERED_PLACING_L2,
            true);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L2_4_RELEASE -> {
        intake.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L2_RELEASE,
            ArmState.CORAL_SCORE_CENTERED_PLACING_L2,
            true);
        autoAlign.markPipeScored();
        swerve.snapsDriveRequest(reefSnapAngle, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L3_2_LINEUP -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L3_LINEUP, ArmState.CORAL_SCORE_CENTERED_LINEUP_L3);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L3_3_PLACE -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L3_PLACE,
            ArmState.CORAL_SCORE_CENTERED_PLACING_L3,
            true);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L3_4_RELEASE -> {
        intake.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L3_RELEASE,
            ArmState.CORAL_SCORE_CENTERED_PLACING_L3,
            true);
        autoAlign.markPipeScored();
        swerve.snapsDriveRequest(reefSnapAngle, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L4_1_POINT_5_RAISE_ARM -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_CENTERED_L4_RAISE_ARM, ArmState.CORAL_STOWED);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L4_2_LINEUP -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L4_LINEUP, ArmState.CORAL_SCORE_CENTERED_LINEUP_L4);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L4_3_PLACE_THEN_RELEASE, CORAL_CENTERED_L4_3_PLACE -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L4_PLACE, ArmState.CORAL_SCORE_CENTERED_LINEUP_L4, true);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_CENTERED_L4_4_RELEASE -> {
        intake.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_CENTERED_L4_RELEASE,
            ArmState.CORAL_SCORE_CENTERED_PLACING_L4,
            true);
        swerve.snapsDriveRequest(reefSnapAngle, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        autoAlign.markPipeScored();
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L2_2_LINEUP -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L2_LINEUP, ArmState.CORAL_SCORE_DISPLACED_LINEUP_L2);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L2_3_PLACE -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L2_PLACE,
            ArmState.CORAL_SCORE_DISPLACED_PLACING_L2,
            true);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L2_4_RELEASE -> {
        intake.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L2_RELEASE,
            ArmState.CORAL_SCORE_DISPLACED_PLACING_L2,
            true);
        swerve.snapsDriveRequest(reefSnapAngle, true);
        autoAlign.markPipeScored();
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L3_2_LINEUP -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L3_LINEUP, ArmState.CORAL_SCORE_DISPLACED_LINEUP_L3);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L3_3_PLACE -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L3_PLACE,
            ArmState.CORAL_SCORE_DISPLACED_PLACING_L3,
            true);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L3_4_RELEASE -> {
        intake.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L3_RELEASE,
            ArmState.CORAL_SCORE_DISPLACED_PLACING_L3,
            true);
        swerve.snapsDriveRequest(reefSnapAngle, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        autoAlign.markPipeScored();
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L4_1_POINT_5_RAISE_ARM -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.CORAL_DISPLACED_L4_RAISE_ARM, ArmState.CORAL_STOWED);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L4_2_LINEUP -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L4_LINEUP, ArmState.CORAL_SCORE_DISPLACED_LINEUP_L4);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L4_3_PLACE_THEN_RELEASE, CORAL_DISPLACED_L4_3_PLACE -> {
        intake.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L4_PLACE,
            ArmState.CORAL_SCORE_DISPLACED_PLACING_L4,
            true);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_DISPLACED_L4_4_RELEASE -> {
        intake.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_DISPLACED_L4_RELEASE,
            ArmState.CORAL_SCORE_DISPLACED_PLACING_L4,
            true);
        swerve.snapsDriveRequest(reefSnapAngle, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        autoAlign.markPipeScored();
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case NET_BACK_WAITING, NET_BACK_PREPARE_TO_SCORE -> {
        intake.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_BACKWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getBackwardNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }

      case NET_FORWARD_WAITING, NET_FORWARD_PREPARE_TO_SCORE -> {
        intake.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_FORWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getForwardNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_WITH_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case NET_FORWARD_SCORING -> {
        intake.setState(ClawState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.NET, ArmState.ALGAE_FORWARD_NET);
        swerve.snapsDriveRequest(SnapUtil.getForwardNetDirection(localization.getPose()));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case PROCESSOR_WAITING, PROCESSOR_PREPARE_TO_SCORE -> {
        intake.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case PROCESSOR_SCORING -> {
        intake.setState(ClawState.SCORE_ALGAE_PROCESSOR);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_OUTTAKE -> {
        intake.setState(ClawState.SCORE_ALGAE_PROCESSOR);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      // TODO: Create special light states for climbing, unjam, and rehoming
      case CLIMBING_1_LINEUP -> {
        intake.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CORAL_STOWED);
        swerve.climbRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.LINEUP);
      }
      case CLIMBING_2_HANGING -> {
        intake.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.HANGING);
      }
      case CLIMBING_3_HANGING_2 -> {
        intake.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.HANGING_2);
      }
      case CLIMBING_4_HANGING_3 -> {
        intake.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.HANGING_3);
      }
      case PREPARE_UNJAM_CORAL_STATION -> {
        intake.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.INTAKING_CORAL_STATION_BACK, ArmState.INTAKING_CORAL_STATION_BACK);
        swerve.normalDriveRequest();
        vision.setState(VisionState.STATION_TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case UNJAM_CORAL_STATION -> {
        intake.setState(ClawState.OUTTAKING);
        moveSuperstructure(
            ElevatorState.INTAKING_CORAL_STATION_BACK, ArmState.INTAKING_CORAL_STATION_BACK);
        swerve.normalDriveRequest();
        vision.setState(VisionState.STATION_TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case UNJAM -> {
        intake.setState(ClawState.OUTTAKING);
        moveSuperstructure(ElevatorState.UNJAM, ArmState.UNJAM);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case REHOME_ELEVATOR -> {
        intake.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.MID_MATCH_HOMING, ArmState.CORAL_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case REHOME_ARM -> {
        intake.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.MID_MATCH_HOMING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
      case REHOME_ROLL -> {
        intake.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.CORAL_STOWED);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.PLACEHOLDER);
        climber.setState(ClimberState.STOWED);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("RobotManager/NearestReefSidePose", nearestReefSide.getPose());
    DogLog.log(
        "RobotManager/ShouldIntakeForward",
        AutoAlign.shouldIntakeStationFront(localization.getPose()));
    DogLog.log("CollisionAvoidance/latestUnsafe", latestUnsafe);

    // Continuous state actions
    moveSuperstructure(latestElevatorGoal, latestArmGoal, latestUnsafe);

    // Update snaps
    switch (getState()) {
      case INTAKE_ALGAE_L2, INTAKE_ALGAE_L3 -> {
        swerve.scoringAlignmentRequest(reefSnapAngle);
      }
      case CORAL_L2_1_APPROACH,
          CORAL_L3_1_APPROACH,
          CORAL_L4_1_APPROACH,
          CORAL_CENTERED_L2_2_LINEUP,
          CORAL_CENTERED_L2_3_PLACE,
          CORAL_CENTERED_L2_4_RELEASE,
          CORAL_CENTERED_L3_2_LINEUP,
          CORAL_CENTERED_L3_3_PLACE,
          CORAL_CENTERED_L3_4_RELEASE,
          CORAL_CENTERED_L4_1_POINT_5_RAISE_ARM,
          CORAL_CENTERED_L4_2_LINEUP,
          CORAL_CENTERED_L4_3_PLACE,
          CORAL_CENTERED_L4_4_RELEASE,
          CORAL_DISPLACED_L2_2_LINEUP,
          CORAL_DISPLACED_L2_3_PLACE,
          CORAL_DISPLACED_L2_4_RELEASE,
          CORAL_DISPLACED_L3_2_LINEUP,
          CORAL_DISPLACED_L3_3_PLACE,
          CORAL_DISPLACED_L3_4_RELEASE,
          CORAL_DISPLACED_L4_1_POINT_5_RAISE_ARM,
          CORAL_DISPLACED_L4_2_LINEUP,
          CORAL_DISPLACED_L4_3_PLACE,
          CORAL_DISPLACED_L4_4_RELEASE -> {
        swerve.scoringAlignmentRequest(reefSnapAngle);
      }
      case INTAKE_CORAL_STATION_BACK -> {
        swerve.snapsDriveRequest(SnapUtil.getCoralStationAngle(localization.getPose()), true);
      }
      case INTAKE_CORAL_STATION_FRONT -> {
        swerve.snapsDriveRequest(
            SnapUtil.getCoralStationAngle(localization.getPose()) - 180.0, true);
      }
      case INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL -> {
        if (FeatureFlags.CORAL_DETECTION.getAsBoolean()) {
          if (maybeBestCoralMapTranslation.isPresent()) {
            var bestCoralMapTranslation = maybeBestCoralMapTranslation.orElseThrow();

            if (DriverStation.isTeleop()) {
              Pose2d lookaheadPose = localization.getLookaheadPose(0.5);
              DogLog.log("IntakeAssist/LookaheadPose", lookaheadPose);
              swerve.setFieldRelativeCoralAssistSpeedsOffset(
                  IntakeAssistUtil.getAssistSpeedsFromPose(bestCoralMapTranslation, lookaheadPose));
              swerve.coralAlignmentDriveRequest();
            } else {
              var coralSnapAngle =
                  IntakeAssistUtil.getIntakeAssistAngle(
                      bestCoralMapTranslation.getTranslation(), localization.getPose());
              swerve.snapsDriveRequest(coralSnapAngle);
            }
          } else {
            swerve.normalDriveRequest();
          }
        }
      }
      default -> {}
    }

    // Update lights
    switch (getState()) {
      case CORAL_L2_1_APPROACH,
          CORAL_L3_1_APPROACH,
          CORAL_L4_1_APPROACH,
          CORAL_L1_3_PLACE,
          CORAL_L1_4_RELEASE,
          CORAL_CENTERED_L2_2_LINEUP,
          CORAL_CENTERED_L2_3_PLACE,
          CORAL_CENTERED_L2_4_RELEASE,
          CORAL_CENTERED_L3_2_LINEUP,
          CORAL_CENTERED_L3_3_PLACE,
          CORAL_CENTERED_L3_4_RELEASE,
          CORAL_CENTERED_L4_1_POINT_5_RAISE_ARM,
          CORAL_CENTERED_L4_2_LINEUP,
          CORAL_CENTERED_L4_3_PLACE,
          CORAL_CENTERED_L4_4_RELEASE,
          CORAL_DISPLACED_L2_2_LINEUP,
          CORAL_DISPLACED_L2_3_PLACE,
          CORAL_DISPLACED_L2_4_RELEASE,
          CORAL_DISPLACED_L3_2_LINEUP,
          CORAL_DISPLACED_L3_3_PLACE,
          CORAL_DISPLACED_L3_4_RELEASE,
          CORAL_DISPLACED_L4_1_POINT_5_RAISE_ARM,
          CORAL_DISPLACED_L4_2_LINEUP,
          CORAL_DISPLACED_L4_3_PLACE,
          CORAL_DISPLACED_L4_4_RELEASE -> {
        lights.setState(getLightStateForScoring());
      }
      default -> {}
    }

    // Prevent this from interfering with the lights for field calibration
    if (!FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      if (vision.isAnyCameraOffline()) {
        lights.setDisabledState(LightsState.ERROR);
      } else if (arm.getState() == ArmState.PRE_MATCH_HOMING && !arm.rangeOfMotionGood()) {
        lights.setDisabledState(LightsState.UNHOMED);
      } else {
        lights.setDisabledState(LightsState.HEALTHY);
      }
    }

    autoAlign.setDriverPoseOffset(swerve.getPoseOffset());
    switch (swerve.getState()) {
      case REEF_ALIGN_TELEOP -> {
        if (autoAlign.isTagAlignedDebounced()) {
          swerve.reefAlignTeleopFineAdjustRequest();
        }
      }
      default -> {}
    }
  }

  @Override
  protected void collectInputs() {
    super.collectInputs();
    nearestReefSide = autoAlign.getClosestReefSide();
    maybeBestCoralMapTranslation = coralMap.getBestCoral();

    reefSnapAngle = nearestReefSide.getPose().getRotation().getDegrees();
    scoringLevel =
        switch (getState()) {
          case CORAL_L1_3_PLACE, CORAL_L1_4_RELEASE -> ReefPipeLevel.L1;
          case CORAL_L2_1_APPROACH,
                  CORAL_CENTERED_L2_2_LINEUP,
                  CORAL_CENTERED_L2_3_PLACE,
                  CORAL_CENTERED_L2_4_RELEASE,
                  CORAL_DISPLACED_L2_2_LINEUP,
                  CORAL_DISPLACED_L2_3_PLACE,
                  CORAL_DISPLACED_L2_4_RELEASE ->
              ReefPipeLevel.L2;
          case CORAL_L3_1_APPROACH,
                  CORAL_CENTERED_L3_2_LINEUP,
                  CORAL_CENTERED_L3_3_PLACE,
                  CORAL_CENTERED_L3_4_RELEASE,
                  CORAL_DISPLACED_L3_2_LINEUP,
                  CORAL_DISPLACED_L3_3_PLACE,
                  CORAL_DISPLACED_L3_4_RELEASE ->
              ReefPipeLevel.L3;
          case CORAL_L4_1_APPROACH,
                  CORAL_CENTERED_L4_1_POINT_5_RAISE_ARM,
                  CORAL_CENTERED_L4_2_LINEUP,
                  CORAL_CENTERED_L4_3_PLACE,
                  CORAL_CENTERED_L4_4_RELEASE,
                  CORAL_CENTERED_L4_3_PLACE_THEN_RELEASE,
                  CORAL_DISPLACED_L4_1_POINT_5_RAISE_ARM,
                  CORAL_DISPLACED_L4_2_LINEUP,
                  CORAL_DISPLACED_L4_3_PLACE,
                  CORAL_DISPLACED_L4_4_RELEASE,
                  CORAL_DISPLACED_L4_3_PLACE_THEN_RELEASE ->
              ReefPipeLevel.L4;
          default -> ReefPipeLevel.BASE;
        };

    DogLog.log("AutoAlign/UsedPose", autoAlign.getUsedScoringPose());

    vision.setClosestScoringReefTag(nearestReefSide.getTagID());

    autoAlign.setScoringLevel(scoringLevel);
    autoAlign.setTeleopSpeeds(swerve.getTeleopSpeeds());
    if (vision.isAnyScoringTagLimelightOnline() || DriverStation.isAutonomous()) {
      var idealAlignSpeeds =
          switch (getState()) {
            case INTAKE_ALGAE_L2, INTAKE_ALGAE_L3 -> autoAlign.getAlgaeAlignSpeeds();
            default -> autoAlign.getTagAlignSpeeds();
          };
      swerve.setAutoAlignAutoSpeeds(idealAlignSpeeds);
      swerve.setAutoAlignSpeeds(autoAlign.calculateConstrainedAndWeightedSpeeds(idealAlignSpeeds));
    } else {
      swerve.setAutoAlignAutoSpeeds(new ChassisSpeeds());
      swerve.setAutoAlignSpeeds(swerve.getTeleopSpeeds());
    }

    swerve.setElevatorHeight(elevator.getHeight());
  }

  private boolean cameraOnlineAndFarEnoughFromReef() {
    var tagCameraOnline = vision.isAnyScoringTagLimelightOnline();

    if (!tagCameraOnline) {
      return timeout(0.5);
    }

    var isFarEnoughFromReefSide =
        !AutoAlign.isCloseToReefSide(localization.getPose(), nearestReefSide.getPose(), 0.75);

    return isFarEnoughFromReefSide;
  }

  public void setAlgaeMode(boolean newAlgaeActive) {
    switch (getState()) {
      // Can change from holding coral to algae, but not the other way around
      case IDLE_CORAL -> {
        algaeMode = newAlgaeActive;
        if (algaeMode) {
          forceIdleNoGp();
        }
      }
      // Update light states when staying in IDLE_NO_GP, but changing algae mode
      case IDLE_NO_GP -> {
        algaeMode = newAlgaeActive;
        forceIdleNoGp();
      }
      // Switch between floor intaking coral and algae
      case INTAKE_CORAL_FLOOR_HORIZONTAL,
          INTAKE_CORAL_FLOOR_UPRIGHT,
          INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL,
          INTAKE_ALGAE_FLOOR -> {
        algaeMode = newAlgaeActive;
        if (newAlgaeActive) {
          setStateFromRequest(RobotState.INTAKE_ALGAE_FLOOR);
        } else {
          setStateFromRequest(RobotState.INTAKE_CORAL_FLOOR_HORIZONTAL);
        }
      }
      // Switch straight from L4 coral scoring to intaking reef algae
      case CORAL_CENTERED_L4_4_RELEASE, CORAL_DISPLACED_L4_4_RELEASE -> {
        algaeMode = newAlgaeActive;
        if (algaeMode) {
          intakeReefAlgaeRequest();
        } else {
          l4CoralReleaseRequest();
        }
      }
      // Switch back and forth from station intaking and reef algae intaking
      case INTAKE_STATION_APPROACH, INTAKE_CORAL_STATION_BACK, INTAKE_CORAL_STATION_FRONT -> {
        algaeMode = newAlgaeActive;
        if (algaeMode) {
          intakeReefAlgaeRequest();
        } else {
          setStateFromRequest(RobotState.INTAKE_STATION_APPROACH);
        }
      }
      default -> {}
    }
  }

  public void forceIdleNoGp() {
    setStateFromRequest(RobotState.IDLE_NO_GP);
    lights.setState(LightsState.idleNoGp(algaeMode));
  }

  public void intakeReefAlgaeRequest() {
    algaeMode = true;
    if (nearestReefSide.algaeHeight == ReefPipeLevel.L3) {
      setStateFromRequest(RobotState.INTAKE_ALGAE_L3);
    } else {
      setStateFromRequest(RobotState.INTAKE_ALGAE_L2);
    }
  }

  public void stowRequest() {
    switch (getState()) {
      case INTAKE_ALGAE_FLOOR, INTAKE_ALGAE_L2, INTAKE_ALGAE_L3 -> {
        // We are cancelling intaking and want to stow instead
        // Because we are using algae, we can't rely on the sensors to detect a game piece

        forceIdleNoGp();
      }
      default -> {
        if (algaeMode) {
          // We know that if we're in this branch of the switch statement we are holding an algae
          setStateFromRequest(RobotState.IDLE_ALGAE);
        } else if (intake.getHasGP()) {
          // We are holding a coral
          setStateFromRequest(RobotState.IDLE_CORAL);
        } else {
          forceIdleNoGp();
        }
      }
    }
  }

  public void intakeFloorRequest() {
    if (algaeMode) {
      intakeFloorAlgaeRequest();
    } else if (!intake.getSensor()) {
      intakeFloorCoralHorizontalRequest();
    }
  }

  public void intakeAssistFloorRequest() {
    if (algaeMode) {
      // No intake assist for algae yet, so just do nothing
      // Require a press & release to trigger algae intake
      // intakeFloorAlgaeRequest();
    } else {
      intakeAssistFloorCoralHorizontalRequest();
    }
  }

  public void intakeFloorAlgaeRequest() {
    algaeMode = true;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ALGAE_FLOOR);
    }
  }

  public void intakeFloorCoralHorizontalRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.INTAKE_CORAL_FLOOR_HORIZONTAL);
    }
  }

  public void intakeAssistFloorCoralHorizontalRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL);
    }
  }

  public void intakeStationRequest() {
    if (algaeMode) {
      intakeReefAlgaeRequest();
    } else {
      switch (getState()) {
        case CLIMBING_1_LINEUP,
            CLIMBING_2_HANGING,
            CLIMBING_3_HANGING_2,
            CLIMBING_4_HANGING_3,
            REHOME_ELEVATOR,
            REHOME_ROLL,
            REHOME_ARM -> {}
        default -> setStateFromRequest(RobotState.INTAKE_STATION_APPROACH);
      }
    }
  }

  public void intakeStationBackRequest() {
    algaeMode = false;
    setStateFromRequest(RobotState.INTAKE_CORAL_STATION_BACK);
  }

  public void intakeStationFrontRequest() {
    algaeMode = false;
    setStateFromRequest(RobotState.INTAKE_CORAL_STATION_FRONT);
  }

  public void lowLineupRequest() {
    if (algaeMode) {
      processorWaitingRequest();
    } else {
      l1CoralLineupRequest();
    }
  }

  public void processorWaitingRequest() {
    algaeMode = true;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.PROCESSOR_WAITING);
    }
  }

  public void l1CoralLineupRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.CORAL_L1_3_PLACE);
    }
  }

  public void l2LineupRequest() {
    if (algaeMode) {
      intakeAlgaeL2Request();
    } else {
      l2CoralApproachRequest();
    }
  }

  private void intakeAlgaeL2Request() {
    algaeMode = true;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ALGAE_L2);
    }
  }

  public void l2CoralApproachRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.CORAL_L2_1_APPROACH);
    }
  }

  public void l3LineupRequest() {
    if (algaeMode) {
      intakeAlgaeL3Request();
    } else {
      l3CoralApproachRequest();
    }
  }

  private void intakeAlgaeL3Request() {
    algaeMode = true;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ALGAE_L3);
    }
  }

  public void l3CoralApproachRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.CORAL_L3_1_APPROACH);
    }
  }

  public void l4CoralScoreRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, CLIMBING_3_HANGING_2, CLIMBING_4_HANGING_3 -> {}
      default -> setStateFromRequest(RobotState.CORAL_CENTERED_L4_3_PLACE);
    }
  }

  public void l4coralPlaceAndReleaseRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, CLIMBING_3_HANGING_2, CLIMBING_4_HANGING_3 -> {}
      default -> setStateFromRequest(RobotState.CORAL_CENTERED_L4_3_PLACE_THEN_RELEASE);
    }
  }

  public void l4CoralReleaseRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING, CLIMBING_3_HANGING_2, CLIMBING_4_HANGING_3 -> {}
      default -> setStateFromRequest(RobotState.CORAL_CENTERED_L4_4_RELEASE);
    }
  }

  public void highApproachRequest() {
    if (algaeMode) {
      algaeNetRequest();
    } else {
      l4CoralApproachRequest();
    }
  }

  public void algaeNetRequest() {
    if (!vision.isAnyTagLimelightOnline()
        || AutoAlign.shouldNetScoreForwards(localization.getPose())) {
      algaeNetForwardRequest();
    } else {
      algaeNetBackRequest();
    }
  }

  private void algaeNetForwardRequest() {
    algaeMode = true;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.NET_FORWARD_WAITING);
    }
  }

  private void algaeNetBackRequest() {
    algaeMode = true;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.NET_BACK_WAITING);
    }
  }

  public void l4CoralApproachRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.CORAL_L4_1_APPROACH);
    }
  }

  public void l4CoralLineupRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.CORAL_CENTERED_L4_1_POINT_5_RAISE_ARM);
    }
  }

  public void preloadCoralRequest() {
    algaeMode = false;
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.IDLE_CORAL);
    }
  }

  public void confirmScoreRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          INTAKE_ALGAE_FLOOR,
          INTAKE_ALGAE_L2,
          INTAKE_ALGAE_L3,
          INTAKE_CORAL_FLOOR_HORIZONTAL,
          INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL,
          INTAKE_CORAL_FLOOR_UPRIGHT,
          INTAKE_CORAL_STATION_BACK,
          INTAKE_CORAL_STATION_FRONT,
          INTAKE_STATION_APPROACH,
          CORAL_CENTERED_L4_1_POINT_5_RAISE_ARM,
          CORAL_DISPLACED_L4_1_POINT_5_RAISE_ARM -> {}

      case IDLE_ALGAE -> {
        setStateFromRequest(RobotState.ALGAE_OUTTAKE);
      }
      case PROCESSOR_WAITING -> setStateFromRequest(RobotState.PROCESSOR_PREPARE_TO_SCORE);
      case NET_BACK_WAITING -> setStateFromRequest(RobotState.NET_BACK_PREPARE_TO_SCORE);
      case NET_FORWARD_WAITING -> setStateFromRequest(RobotState.NET_FORWARD_PREPARE_TO_SCORE);

      case CORAL_L1_3_PLACE -> setStateFromRequest(RobotState.CORAL_L1_4_RELEASE);

      case CORAL_L2_1_APPROACH -> setStateFromRequest(RobotState.CORAL_CENTERED_L2_2_LINEUP);
      case CORAL_CENTERED_L2_2_LINEUP -> setStateFromRequest(RobotState.CORAL_CENTERED_L2_3_PLACE);
      case CORAL_CENTERED_L2_3_PLACE -> setStateFromRequest(RobotState.CORAL_CENTERED_L2_4_RELEASE);
      case CORAL_DISPLACED_L2_2_LINEUP ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L2_3_PLACE);
      case CORAL_DISPLACED_L2_3_PLACE ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L2_4_RELEASE);

      case CORAL_L3_1_APPROACH -> setStateFromRequest(RobotState.CORAL_CENTERED_L3_2_LINEUP);
      case CORAL_CENTERED_L3_2_LINEUP -> setStateFromRequest(RobotState.CORAL_CENTERED_L3_3_PLACE);
      case CORAL_CENTERED_L3_3_PLACE -> setStateFromRequest(RobotState.CORAL_CENTERED_L3_4_RELEASE);
      case CORAL_DISPLACED_L3_2_LINEUP ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L3_3_PLACE);
      case CORAL_DISPLACED_L3_3_PLACE ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L3_4_RELEASE);

      case CORAL_L4_1_APPROACH -> setStateFromRequest(RobotState.CORAL_CENTERED_L4_2_LINEUP);
      case CORAL_CENTERED_L4_2_LINEUP -> setStateFromRequest(RobotState.CORAL_CENTERED_L4_3_PLACE);
      case CORAL_CENTERED_L4_3_PLACE -> setStateFromRequest(RobotState.CORAL_CENTERED_L4_4_RELEASE);
      case CORAL_DISPLACED_L4_2_LINEUP ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L4_3_PLACE);
      case CORAL_DISPLACED_L4_3_PLACE ->
          setStateFromRequest(RobotState.CORAL_DISPLACED_L4_4_RELEASE);

      default -> {}
    }
  }

  public void nextClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_3_HANGING_2);
      case CLIMBING_3_HANGING_2 -> setStateFromRequest(RobotState.CLIMBING_4_HANGING_3);
      case CLIMBING_4_HANGING_3 -> {}
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void previousClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_4_HANGING_3 -> setStateFromRequest(RobotState.CLIMBING_3_HANGING_2);
      case CLIMBING_3_HANGING_2 -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLIMBING_1_LINEUP -> {
        setStateFromRequest(RobotState.IDLE_NO_GP);
      }
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void unjamRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void unjamStationRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.PREPARE_UNJAM_CORAL_STATION);
    }
  }

  public void rehomeElevatorRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ROLL,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.REHOME_ELEVATOR);
    }
  }

  public void rehomeArmRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ROLL -> {}
      default -> setStateFromRequest(RobotState.REHOME_ARM);
    }
  }

  public void rehomeRollRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBING_3_HANGING_2,
          CLIMBING_4_HANGING_3,
          REHOME_ELEVATOR,
          REHOME_ARM -> {}
      default -> setStateFromRequest(RobotState.REHOME_ROLL);
    }
  }

  public Command waitForRollHomedCommand() {
    return Commands.waitUntil(() -> isRollHomed);
  }

  private ElevatorState latestElevatorGoal = ElevatorState.STOWED;
  private ArmState latestArmGoal = ArmState.PRE_MATCH_HOMING;
  private boolean latestUnsafe = false;

  private void moveSuperstructure(ElevatorState elevatorGoal, ArmState armGoal) {
    moveSuperstructure(elevatorGoal, armGoal, false);
  }

  private void moveSuperstructure(
      ElevatorState elevatorGoal, ArmState armGoal, boolean unsafe) {
    latestElevatorGoal = elevatorGoal;
    latestArmGoal = armGoal;
    latestUnsafe = unsafe;

    var maybeIntermediaryPosition =
        CollisionAvoidance.plan(
            new SuperstructurePosition(elevator.getHeight(), arm.getAngle()),
            new SuperstructurePosition(elevatorGoal.height, armGoal.angle));

    if (unsafe || maybeIntermediaryPosition.isEmpty()) {
      // No collision, go straight to goal state or unsafe mode
      elevator.setState(elevatorGoal);
      arm.setState(armGoal);
    } else {
      var intermediaryPosition = maybeIntermediaryPosition.orElseThrow();

      // A collision was detected, so we need to go to an intermediary point
      elevator.setCollisionAvoidanceGoal(intermediaryPosition.elevatorHeight());
      elevator.setState(ElevatorState.COLLISION_AVOIDANCE);

      arm.setCollisionAvoidanceGoal(intermediaryPosition.armAngle());
      arm.setState(ArmState.COLLISION_AVOIDANCE);
    }
  }

  private LightsState getLightStateForScoring() {
    return switch (autoAlign.getReefAlignState()) {
      case ALL_CAMERAS_DEAD -> LightsState.ERROR;
      case HAS_TAGS_IN_POSITION -> LightsState.SCORE_ALIGN_READY;
      default -> LightsState.SCORE_ALIGN_NOT_READY;
    };
  }

  public void setConfirmScoreActive(boolean newValue) {
    confirmScoreActive = newValue;
  }

  public boolean notSmartStowing() {
    return getState() != RobotState.SMART_STOW_1 && getState() != RobotState.SMART_STOW_2;
  }
}
