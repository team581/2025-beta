package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefSide;
import frc.robot.auto_align.ReefSideOffset;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.claw.ClawState;
import frc.robot.claw.ClawSubsystem;
import frc.robot.climber.ClimberState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.FeatureFlags;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.lights.LightsState;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.collision_avoidance.CollisionAvoidance;
import frc.robot.robot_manager.collision_avoidance.ObstructionKind;
import frc.robot.robot_manager.ground_manager.GroundManager;
import frc.robot.robot_manager.ground_manager.GroundState;
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
  public final ClawSubsystem claw;
  public final ArmSubsystem arm;
  public final ElevatorSubsystem elevator;
  public final ClimberSubsystem climber;
  public final RumbleControllerSubsystem rumbleController;

  public final GroundManager groundManager;

  private final LightsSubsystem lights;

  public final AutoAlign autoAlign;

  public RobotManager(
      GroundManager groundManager,
      ClawSubsystem claw,
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
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.CLAW_EMPTY);
    this.groundManager = groundManager;
    this.arm = arm;
    this.claw = claw;
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

    var stateCount = RobotState.values().length;

    DogLog.log("RobotManager/StateCount", stateCount);
  }

  private double reefSnapAngle = 0.0;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private ReefSide nearestReefSide = ReefSide.SIDE_GH;
  private ReefPipeLevel scoringLevel = ReefPipeLevel.L4;
  private Pose2d robotPose;
  private ObstructionKind shouldLoopAroundToScoreObstruction = ObstructionKind.NONE;
  private Optional<RobotState> afterIntakingCoralState = Optional.empty();
  private boolean scoringAlignActive = false;

  @Override
  protected RobotState getNextState(RobotState currentState) {
    if (afterIntakingCoralState.isPresent() && groundManager.hasCoral()) {
      return afterIntakingCoralState.orElseThrow();
    }

    return switch (currentState) {
      case CLAW_EMPTY,
              CLAW_ALGAE,
              CLAW_CORAL,
              ALGAE_INTAKE_FLOOR,
              ALGAE_PROCESSOR_WAITING,
              ALGAE_NET_LEFT_WAITING,
              ALGAE_NET_RIGHT_WAITING,
              CLIMBING_2_HANGING,
              CLIMBER_STOP,
              UNJAM,
              SPIN_TO_WIN,
              CORAL_INTAKE_LOLLIPOP_APPROACH,
              CORAL_INTAKE_LOLLIPOP_PUSH,
              PREPARE_HANDOFF_AFTER_INTAKE,
              ALGAE_FLING_WAIT ->
          currentState;
      case CORAL_L2_RIGHT_PLACE,
          CORAL_L2_LEFT_PLACE,
          CORAL_L3_RIGHT_PLACE,
          CORAL_L3_LEFT_PLACE,
          CORAL_L4_RIGHT_PLACE,
          CORAL_L4_LEFT_PLACE -> {
        if (FeatureFlags.AUTO_ALIGN_AUTO_SCORE.getAsBoolean()
            && autoAlign.isTagAlignedDebounced()
            && arm.atGoal()
            && elevator.atGoal()
            && scoringAlignActive) {
          yield currentState.getPlaceToReleaseState();
        }
        yield currentState;
      }
      case CORAL_L1_RIGHT_LINEUP,
          CORAL_L2_LEFT_LINEUP,
          CORAL_L2_RIGHT_LINEUP,
          CORAL_L3_LEFT_LINEUP,
          CORAL_L3_RIGHT_LINEUP,
          CORAL_L4_LEFT_LINEUP,
          CORAL_L4_RIGHT_LINEUP -> {
        if (FeatureFlags.AUTO_ALIGN_AUTO_SCORE.getAsBoolean()
            && autoAlign.isTagAlignedDebounced()
            && arm.atGoal()
            && elevator.atGoal()
            && scoringAlignActive) {
          yield currentState.getLineupToPlaceState();
        }
        yield currentState;
      }
      case REHOME_ELEVATOR ->
          elevator.getState() == ElevatorState.STOWED ? RobotState.CLAW_EMPTY : currentState;
      case PREPARE_SPIN_TO_WIN ->
          elevator.atGoal() && arm.atGoal() ? RobotState.SPIN_TO_WIN : currentState;

      case ALGAE_FLING_PREPARE -> arm.atGoal() ? RobotState.ALGAE_FLING_RELEASE : currentState;
      case ALGAE_FLING_RELEASE -> !claw.getHasGP() ? RobotState.CLAW_EMPTY : currentState;

      // handoff
      case CORAL_L1_PREPARE_HANDOFF,
              CORAL_L2_PREPARE_HANDOFF,
              CORAL_L3_PREPARE_HANDOFF,
              CORAL_L4_PREPARE_HANDOFF ->
          elevator.atGoal() && arm.atGoal() && groundManager.deploy.atGoal()
              ? currentState.getHandoffPrepareToReleaseState()
              : currentState;

      case CORAL_L1_RELEASE_HANDOFF,
              CORAL_L2_RELEASE_HANDOFF,
              CORAL_L3_RELEASE_HANDOFF,
              CORAL_L4_RELEASE_HANDOFF ->
          claw.getHasGP()
              ? currentState.getHandoffReleaseToApproachState(robotScoringSide)
              : currentState;

      // Approach
      case CORAL_L1_RIGHT_APPROACH ->
          elevator.nearGoal() && arm.nearGoal()
              ? currentState.getRightApproachToLineupState()
              : currentState;

      case CORAL_L2_LEFT_APPROACH, CORAL_L3_LEFT_APPROACH, CORAL_L4_LEFT_APPROACH -> {
        if (robotScoringSide == RobotScoringSide.RIGHT) {
          yield currentState.getLeftToRightApproachState();
        }

        yield elevator.nearGoal() && arm.nearGoal()
            ? currentState.getLeftApproachToLineupState()
            : currentState;
      }

      case CORAL_L2_RIGHT_APPROACH, CORAL_L3_RIGHT_APPROACH, CORAL_L4_RIGHT_APPROACH -> {
        if (robotScoringSide == RobotScoringSide.LEFT) {
          yield currentState.getRightToLeftApproachState();
        }

        yield elevator.nearGoal() && arm.nearGoal()
            ? currentState.getRightApproachToLineupState()
            : currentState;
      }

      case CORAL_L2_LEFT_RELEASE,
          CORAL_L3_LEFT_RELEASE,
          CORAL_L4_LEFT_RELEASE,
          CORAL_L1_RIGHT_RELEASE,
          CORAL_L2_RIGHT_RELEASE,
          CORAL_L3_RIGHT_RELEASE,
          CORAL_L4_RIGHT_RELEASE -> {
        if (DriverStation.isTeleop()) {
          // In teleop, we go to CLAW_EMPTY when you drive away or if we know the score succeeded
          if (cameraOnlineAndFarEnoughFromReef()) {
            yield RobotState.CLAW_EMPTY;
          }
        } else if (arm.atGoal() && elevator.atGoal() && (!claw.getHasGP() || timeout(0.5))) {
          // In auto, check if the score succeeded with a timeout
          yield RobotState.CLAW_EMPTY;
        }

        yield currentState;
      }

      // Algae scoring
      case ALGAE_PROCESSOR_RELEASE,
              ALGAE_NET_LEFT_RELEASE,
              ALGAE_NET_RIGHT_RELEASE,
              ALGAE_OUTTAKE ->
          timeout(0.5) || !claw.getHasGP() ? RobotState.CLAW_EMPTY : currentState;

      // Intaking

      case ALGAE_INTAKE_L2_LEFT_APPROACH,
              ALGAE_INTAKE_L2_RIGHT_APPROACH,
              ALGAE_INTAKE_L3_LEFT_APPROACH,
              ALGAE_INTAKE_L3_RIGHT_APPROACH ->
          arm.nearGoal() && elevator.nearGoal()
              ? currentState.getAlgaeApproachToIntakeState()
              : currentState;

      case ALGAE_INTAKE_L2_LEFT,
          ALGAE_INTAKE_L3_LEFT,
          ALGAE_INTAKE_L2_RIGHT,
          ALGAE_INTAKE_L3_RIGHT -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
          if (cameraOnlineAndFarEnoughFromReef()) {
            yield currentState.getAlgaeIntakeToHoldingState();
          }
        }

        yield currentState;
      }

      case ALGAE_INTAKE_L2_LEFT_HOLDING,
          ALGAE_INTAKE_L3_LEFT_HOLDING,
          ALGAE_INTAKE_L2_RIGHT_HOLDING,
          ALGAE_INTAKE_L3_RIGHT_HOLDING -> {
        if (cameraOnlineAndFarEnoughFromReef()) {
          yield RobotState.CLAW_ALGAE;
        }

        yield currentState;
      }

      case CORAL_INTAKE_LOLLIPOP_GRAB -> claw.getHasGP() ? RobotState.CLAW_CORAL : currentState;

      case CLIMBING_1_LINEUP ->
          climber.holdingCage() ? RobotState.CLIMBING_2_HANGING : currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case CLAW_EMPTY -> {
        claw.setState(ClawState.IDLE_NO_GP);
        groundManager.idleRequest();
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_EMPTY);
        climber.setState(ClimberState.STOPPED);
      }
      case CLAW_ALGAE -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        groundManager.idleRequest();
        moveSuperstructure(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case CLAW_CORAL -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        groundManager.idleRequest();
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_CORAL);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_FLOOR -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_GROUND, ArmState.ALGAE_INTAKE_FLOOR);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2_LEFT_APPROACH -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2_LEFT, ArmState.ALGAE_INTAKE_LEFT_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2_LEFT -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2_LEFT, ArmState.ALGAE_INTAKE_LEFT_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2_RIGHT_APPROACH -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2_RIGHT, ArmState.ALGAE_INTAKE_RIGHT_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2_RIGHT -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2_RIGHT, ArmState.ALGAE_INTAKE_RIGHT_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3_LEFT_APPROACH -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3_LEFT, ArmState.ALGAE_INTAKE_LEFT_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3_LEFT -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3_LEFT, ArmState.ALGAE_INTAKE_LEFT_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3_RIGHT_APPROACH -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3_RIGHT, ArmState.ALGAE_INTAKE_RIGHT_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3_RIGHT -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3_RIGHT, ArmState.ALGAE_INTAKE_RIGHT_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_INTAKE_LOLLIPOP_APPROACH -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            // TODO: We don't want to require unsafe mode here
            ElevatorState.LOLLIPOP_CORAL_INTAKE_PUSH, ArmState.LOLLIPOP_CORAL_INTAKE_PUSH, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.ALGAE_DETECTION);
        lights.setState(getLightsStateForLollipop());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_INTAKE_LOLLIPOP_PUSH -> {
        claw.setState(ClawState.OUTTAKING);
        moveSuperstructure(
            // TODO: We don't want to require unsafe mode here
            ElevatorState.LOLLIPOP_CORAL_INTAKE_PUSH, ArmState.LOLLIPOP_CORAL_INTAKE_PUSH, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(getLightsStateForLollipop());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_INTAKE_LOLLIPOP_GRAB -> {
        claw.setState(ClawState.LOLLIPOP_CORAL_INTAKE);
        moveSuperstructure(
            ElevatorState.LOLLIPOP_CORAL_INTAKE_INTAKE, ArmState.LOLLIPOP_CORAL_INTAKE_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.ALGAE_DETECTION);
        lights.setState(getLightsStateForLollipop());
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_NET_LEFT_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_LEFT);
        swerve.snapsDriveRequest(SnapUtil.getNetScoringAngle(RobotScoringSide.LEFT, robotPose));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_NET_RIGHT_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_RIGHT);
        swerve.snapsDriveRequest(SnapUtil.getNetScoringAngle(RobotScoringSide.RIGHT, robotPose));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_NET_LEFT_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_LEFT, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_NET_RIGHT_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_RIGHT, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_PROCESSOR_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_PROCESSOR_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_PROCESSOR);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR, true);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case PREPARE_HANDOFF_AFTER_INTAKE -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.HANDOFF);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOPPED);
      }
      // Handoff states
      case CORAL_L1_PREPARE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        afterIntakingCoralState = Optional.empty();
        groundManager.handoffWaitRequest();
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.HANDOFF);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_RELEASE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        groundManager.handoffReleaseRequest();
        moveSuperstructure(ElevatorState.CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.HANDOFF);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOPPED);
      }

      // L1
      case CORAL_L1_RIGHT_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L1, ArmState.CORAL_SCORE_RIGHT_LINEUP_L1);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_RIGHT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L1, ArmState.CORAL_SCORE_RIGHT_LINEUP_L1);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_RIGHT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L1,
            ArmState.CORAL_SCORE_RIGHT_RELEASE_L1,
            true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      // L2
      case CORAL_L2_LEFT_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_LEFT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_LEFT_PLACE -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_LEFT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_RIGHT_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L2, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2);
        swerve.snapsDriveRequest(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_RIGHT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L2, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2);
        swerve.snapsDriveRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_RIGHT_PLACE -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L2,
            ArmState.CORAL_SCORE_RIGHT_RELEASE_L2,
            true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_RIGHT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L2,
            ArmState.CORAL_SCORE_RIGHT_RELEASE_L2,
            true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      // L3
      case CORAL_L3_LEFT_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_LINEUP_L3, ArmState.CORAL_SCORE_LEFT_LINEUP_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_LEFT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_LINEUP_L3, ArmState.CORAL_SCORE_LEFT_LINEUP_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_LEFT_PLACE -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_RELEASE_L3, ArmState.CORAL_SCORE_LEFT_RELEASE_L3, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_LEFT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_RELEASE_L3, ArmState.CORAL_SCORE_LEFT_RELEASE_L3, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_RIGHT_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L3, ArmState.CORAL_SCORE_RIGHT_LINEUP_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_RIGHT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L3, ArmState.CORAL_SCORE_RIGHT_LINEUP_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_RIGHT_PLACE -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L3,
            ArmState.CORAL_SCORE_RIGHT_RELEASE_L3,
            true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_RIGHT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L3,
            ArmState.CORAL_SCORE_RIGHT_RELEASE_L3,
            true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      // L4
      case CORAL_L4_LEFT_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_LINEUP_L4, ArmState.CORAL_SCORE_LEFT_LINEUP_L4);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_LEFT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_LINEUP_L4, ArmState.CORAL_SCORE_LEFT_LINEUP_L4);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_LEFT_PLACE -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_RELEASE_L4, ArmState.CORAL_SCORE_LEFT_RELEASE_L4, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_LEFT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_RELEASE_L4, ArmState.CORAL_SCORE_LEFT_RELEASE_L4, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_RIGHT_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L4, ArmState.CORAL_SCORE_RIGHT_LINEUP_L4);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_RIGHT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L4, ArmState.CORAL_SCORE_RIGHT_LINEUP_L4);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_RIGHT_PLACE -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L4,
            ArmState.CORAL_SCORE_RIGHT_RELEASE_L4,
            true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_RIGHT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L4,
            ArmState.CORAL_SCORE_RIGHT_RELEASE_L4,
            true);
        vision.setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      // TODO: Create special light states for climbing, unjam, and rehoming
      case CLIMBING_1_LINEUP -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.CLIMBING, ArmState.CLIMBING);
        swerve.climbRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CLIMB_LINEUP);
        climber.setState(ClimberState.LINEUP_FORWARD);
      }
      case CLIMBING_2_HANGING -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.CLIMBING, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CLIMB_HANG);
        climber.setState(ClimberState.HANGING);
      }
      case CLIMBER_STOP -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.CLIMBING, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CLIMB_STOP);
        climber.setState(ClimberState.STOPPED);
      }
      case UNJAM -> {
        claw.setState(ClawState.OUTTAKING);
        groundManager.unjamRequest();
        moveSuperstructure(ElevatorState.UNJAM, ArmState.UNJAM);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.OTHER);
        climber.setState(ClimberState.STOPPED);
      }
      case REHOME_ELEVATOR -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.MID_MATCH_HOMING, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.OTHER);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_OUTTAKE -> {
        claw.setState(ClawState.OUTTAKING);
        moveSuperstructure(ElevatorState.ALGAE_OUTTAKE, ArmState.ALGAE_OUTTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case PREPARE_SPIN_TO_WIN -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_NO_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case SPIN_TO_WIN -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.SPIN_TO_WIN, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_FLING_WAIT -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_FLING, ArmState.ALGAE_FLING_WAIT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_NO_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_FLING_PREPARE -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_FLING, ArmState.ALGAE_FLING_SWING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_TAGS);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_FLING_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.ALGAE_FLING, ArmState.ALGAE_FLING_SWING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOPPED);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("RobotManager/NearestReefSidePose", nearestReefSide.getPose());
    DogLog.log("CollisionAvoidance/latestUnsafe", latestUnsafe);
    DogLog.log("AutoAlign/ScoringLoopAroundObstruction", shouldLoopAroundToScoreObstruction);
    // Continuous state actions
    moveSuperstructure(latestElevatorGoal, latestArmGoal, latestUnsafe);

    arm.setCoralTx(vision.getHandoffOffsetTx());

    switch (getState()) {
      case ALGAE_INTAKE_L2_LEFT_APPROACH,
          ALGAE_INTAKE_L3_LEFT_APPROACH,
          ALGAE_INTAKE_L2_RIGHT_APPROACH,
          ALGAE_INTAKE_L3_RIGHT_APPROACH,
          ALGAE_INTAKE_L2_LEFT,
          ALGAE_INTAKE_L3_LEFT,
          ALGAE_INTAKE_L2_RIGHT,
          ALGAE_INTAKE_L3_RIGHT,
          ALGAE_INTAKE_L2_LEFT_HOLDING,
          ALGAE_INTAKE_L3_LEFT_HOLDING,
          ALGAE_INTAKE_L2_RIGHT_HOLDING,
          ALGAE_INTAKE_L3_RIGHT_HOLDING -> {
        if (scoringAlignActive) {
          swerve.scoringAlignmentRequest(reefSnapAngle);
        } else {
          swerve.normalDriveRequest();
        }
      }
      case CORAL_L1_RIGHT_APPROACH,
          CORAL_L2_LEFT_APPROACH,
          CORAL_L2_RIGHT_APPROACH,
          CORAL_L3_LEFT_APPROACH,
          CORAL_L3_RIGHT_APPROACH,
          CORAL_L4_LEFT_APPROACH,
          CORAL_L4_RIGHT_APPROACH,
          CORAL_L2_LEFT_LINEUP,
          CORAL_L3_LEFT_LINEUP,
          CORAL_L4_LEFT_LINEUP,
          CORAL_L2_RIGHT_LINEUP,
          CORAL_L3_RIGHT_LINEUP,
          CORAL_L4_RIGHT_LINEUP,
          CORAL_L2_LEFT_PLACE,
          CORAL_L3_LEFT_PLACE,
          CORAL_L4_LEFT_PLACE,
          CORAL_L2_RIGHT_PLACE,
          CORAL_L3_RIGHT_PLACE,
          CORAL_L4_RIGHT_PLACE,
          CORAL_L2_LEFT_RELEASE,
          CORAL_L3_LEFT_RELEASE,
          CORAL_L4_LEFT_RELEASE,
          CORAL_L2_RIGHT_RELEASE,
          CORAL_L3_RIGHT_RELEASE,
          CORAL_L4_RIGHT_RELEASE -> {
        if (scoringAlignActive) {
          swerve.scoringAlignmentRequest(reefSnapAngle);
          lights.setState(getLightStateForScoring());
        } else {
          lights.setState(LightsState.HOLDING_CORAL);
          swerve.normalDriveRequest();
        }
      }
      case CORAL_INTAKE_LOLLIPOP_APPROACH,
          CORAL_INTAKE_LOLLIPOP_GRAB,
          CORAL_INTAKE_LOLLIPOP_PUSH -> {
        coralMap.updateLollipopResult(vision.getLollipopVisionResult());
      }
      default -> {}
    }

    // Update lights
    switch (getState()) {
      case CORAL_INTAKE_LOLLIPOP_APPROACH,
          CORAL_INTAKE_LOLLIPOP_GRAB,
          CORAL_INTAKE_LOLLIPOP_PUSH -> {
        lights.setState(getLightsStateForLollipop());
      }
      default -> {}
    }

    // Prevent this from interfering with the lights for field calibration
    if (!FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      if (vision.isAnyCameraOffline()) {
        lights.setDisabledState(LightsState.ERROR);
      } else if (arm.getState() == ArmState.PRE_MATCH_HOMING && !arm.rangeOfMotionGood()) {
        lights.setDisabledState(LightsState.UNHOMED);
      } else if (vision.seeingTagRecentlyDisabled()) {
        lights.setDisabledState(LightsState.HEALTHY);
      } else {
        lights.setDisabledState(LightsState.SCORE_ALIGN_NO_TAGS);
      }
    }

    switch (getState()) {
      case ALGAE_INTAKE_FLOOR -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
        }
      }
      default -> {}
    }
  }

  @Override
  protected void collectInputs() {
    super.collectInputs();
    nearestReefSide = autoAlign.getClosestReefSide();
    robotPose = localization.getPose();

    robotScoringSide =
        AutoAlign.getScoringSideFromRobotPose(
            robotPose,
            vision.isAnyLeftScoringTagLimelightOnline(),
            vision.isAnyRightScoringTagLimelightOnline());
    shouldLoopAroundToScoreObstruction = autoAlign.shouldArmGoAroundToScore();
    reefSnapAngle = autoAlign.getUsedScoringPose().getRotation().getDegrees();
    scoringLevel =
        switch (getState()) {
          case CORAL_L1_RIGHT_APPROACH,
                  CORAL_L2_LEFT_APPROACH,
                  CORAL_L2_RIGHT_APPROACH,
                  CORAL_L3_LEFT_APPROACH,
                  CORAL_L3_RIGHT_APPROACH,
                  CORAL_L4_LEFT_APPROACH,
                  CORAL_L4_RIGHT_APPROACH,
                  CORAL_L2_LEFT_RELEASE,
                  CORAL_L2_RIGHT_RELEASE,
                  CORAL_L3_LEFT_RELEASE,
                  CORAL_L3_RIGHT_RELEASE,
                  CORAL_L4_LEFT_RELEASE,
                  CORAL_L4_RIGHT_RELEASE ->
              ReefPipeLevel.RAISING;

          case CORAL_L1_RIGHT_LINEUP, CORAL_L1_RIGHT_RELEASE -> ReefPipeLevel.L1;
          case CORAL_L2_LEFT_LINEUP,
                  CORAL_L2_LEFT_PLACE,
                  CORAL_L2_RIGHT_LINEUP,
                  CORAL_L2_RIGHT_PLACE ->
              ReefPipeLevel.L2;
          case CORAL_L3_LEFT_LINEUP,
                  CORAL_L3_LEFT_PLACE,
                  CORAL_L3_RIGHT_LINEUP,
                  CORAL_L3_RIGHT_PLACE ->
              ReefPipeLevel.L3;
          case CORAL_L4_LEFT_LINEUP,
                  CORAL_L4_LEFT_PLACE,
                  CORAL_L4_RIGHT_LINEUP,
                  CORAL_L4_RIGHT_PLACE ->
              ReefPipeLevel.L4;
          default -> ReefPipeLevel.RAISING;
        };
    autoAlign.setScoringLevel(scoringLevel, robotScoringSide);

    var reefSideOffset =
        switch (getState()) {
          case ALGAE_INTAKE_L2_LEFT,
                  ALGAE_INTAKE_L3_LEFT,
                  ALGAE_INTAKE_L2_RIGHT,
                  ALGAE_INTAKE_L3_RIGHT ->
              ReefSideOffset.ALGAE_INTAKING;
          default -> ReefSideOffset.ALGAE_RAISING;
        };
    autoAlign.setAlgaeIntakingOffset(reefSideOffset);

    vision.setClosestScoringReefAndPipe(nearestReefSide.getTagID(), autoAlign.getBestReefPipe());
    vision.updateDistanceFromReef(
        robotPose.getTranslation().getDistance(nearestReefSide.getPose().getTranslation()));

    if (vision.isAnyTagLimelightOnline() && DriverStation.isTeleop()) {
      var idealAlignSpeeds =
          switch (getState()) {
            case ALGAE_INTAKE_L2_LEFT_APPROACH,
                    ALGAE_INTAKE_L3_LEFT_APPROACH,
                    ALGAE_INTAKE_L2_RIGHT_APPROACH,
                    ALGAE_INTAKE_L3_RIGHT_APPROACH,
                    ALGAE_INTAKE_L2_LEFT,
                    ALGAE_INTAKE_L3_LEFT,
                    ALGAE_INTAKE_L2_RIGHT,
                    ALGAE_INTAKE_L3_RIGHT ->
                autoAlign.getAlgaeAlignSpeeds();
            default -> autoAlign.getTagAlignSpeeds();
          };
      swerve.setAutoAlignSpeeds(idealAlignSpeeds);
    } else {
      swerve.setAutoAlignSpeeds(swerve.getTeleopSpeeds());
    }

    swerve.setElevatorHeight(elevator.getHeight());
  }

  private boolean cameraOnlineAndFarEnoughFromReef() {
    var tagCameraOnline = vision.isAnyTagLimelightOnline();

    if (!tagCameraOnline) {
      return timeout(0.5);
    }

    var isFarEnoughFromReefSide =
        !AutoAlign.isCloseToReefSide(robotPose, nearestReefSide.getPose(), 0.9);

    return isFarEnoughFromReefSide;
  }

  public void forceIdleNoGp() {
    setStateFromRequest(RobotState.CLAW_EMPTY);
  }

  public void stowRequest() {
    afterIntakingCoralState = Optional.empty();
    switch (getState()) {
      case ALGAE_INTAKE_L2_LEFT,
          ALGAE_INTAKE_L2_RIGHT,
          ALGAE_INTAKE_L3_LEFT,
          ALGAE_INTAKE_L3_RIGHT -> {
        groundManager.idleRequest();
        setStateFromRequest(RobotState.CLAW_EMPTY);
      }
      default -> {
        groundManager.idleRequest();
        if (claw.getHasGP()) {
          // Claw is maybe algae or coral
          if (getState().clawGp == ClawGamePiece.ALGAE) {
            setStateFromRequest(RobotState.CLAW_ALGAE);
          } else {
            setStateFromRequest(RobotState.CLAW_CORAL);
          }
        } else {
          setStateFromRequest(RobotState.CLAW_EMPTY);
        }
      }
    }
  }

  public void algaeFlingRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    setStateFromRequest(RobotState.ALGAE_FLING_WAIT);
  }

  public void algaeFlingConfirmRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    setStateFromRequest(RobotState.ALGAE_FLING_PREPARE);
  }

  public void intakeFloorAlgaeRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.ALGAE_INTAKE_FLOOR);
    }
  }

  public void lollipopIntakeApproachRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.CORAL_INTAKE_LOLLIPOP_APPROACH);
    }
  }

  public void lollipopIntakeGrabRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.CORAL_INTAKE_LOLLIPOP_GRAB);
    }
  }

  public void testingNextLollipopRequest() {
    if (!getState().climbingOrRehoming) {
      switch (getState()) {
        case CORAL_INTAKE_LOLLIPOP_APPROACH -> lollipopIntakeGrabRequest();
        case CORAL_INTAKE_LOLLIPOP_GRAB ->
            setStateFromRequest(RobotState.CORAL_INTAKE_LOLLIPOP_PUSH);
        case CORAL_INTAKE_LOLLIPOP_PUSH -> setStateFromRequest(RobotState.CLAW_CORAL);
        default -> lollipopIntakeApproachRequest();
      }
    }
  }

  public void processorWaitingRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.ALGAE_PROCESSOR_WAITING);
    }
  }

  public void scoringAlignOffRequest() {
    scoringAlignActive = false;
  }

  public void l4CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }
    scoringAlignActive = true;
    if (claw.getHasGP()) {
      switch (robotScoringSide) {
        case LEFT -> setStateFromRequest(RobotState.CORAL_L4_LEFT_APPROACH);
        case RIGHT -> setStateFromRequest(RobotState.CORAL_L4_RIGHT_APPROACH);
      }
    } else if (groundManager.getState() == GroundState.INTAKING) {
      afterIntakingCoralState = Optional.of(RobotState.CORAL_L4_PREPARE_HANDOFF);
      setStateFromRequest(RobotState.PREPARE_HANDOFF_AFTER_INTAKE);
    } else {
      setStateFromRequest(RobotState.CORAL_L4_PREPARE_HANDOFF);
    }
  }

  public void l3CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    scoringAlignActive = true;
    if (claw.getHasGP()) {
      switch (robotScoringSide) {
        case LEFT -> setStateFromRequest(RobotState.CORAL_L3_LEFT_APPROACH);
        case RIGHT -> setStateFromRequest(RobotState.CORAL_L3_RIGHT_APPROACH);
      }
    } else if (groundManager.getState() == GroundState.INTAKING) {
      afterIntakingCoralState = Optional.of(RobotState.CORAL_L3_PREPARE_HANDOFF);
      setStateFromRequest(RobotState.PREPARE_HANDOFF_AFTER_INTAKE);
    } else {
      setStateFromRequest(RobotState.CORAL_L3_PREPARE_HANDOFF);
    }
  }

  public void l2CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    scoringAlignActive = true;
    if (claw.getHasGP()) {
      switch (robotScoringSide) {
        case LEFT -> setStateFromRequest(RobotState.CORAL_L2_LEFT_APPROACH);
        case RIGHT -> setStateFromRequest(RobotState.CORAL_L2_RIGHT_APPROACH);
      }
    } else if (groundManager.getState() == GroundState.INTAKING) {
      afterIntakingCoralState = Optional.of(RobotState.CORAL_L2_PREPARE_HANDOFF);
      setStateFromRequest(RobotState.PREPARE_HANDOFF_AFTER_INTAKE);
    } else {
      setStateFromRequest(RobotState.CORAL_L2_PREPARE_HANDOFF);
    }
  }

  public void l1CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    if (claw.getHasGP()) {
      setStateFromRequest(RobotState.CORAL_L1_RIGHT_APPROACH);
    } else if (groundManager.getState() == GroundState.INTAKING) {
      afterIntakingCoralState = Optional.of(RobotState.CORAL_L1_PREPARE_HANDOFF);
      setStateFromRequest(RobotState.PREPARE_HANDOFF_AFTER_INTAKE);
    } else {
      setStateFromRequest(RobotState.CORAL_L1_PREPARE_HANDOFF);
    }
  }

  public void highLineupRequest() {
    if (!getState().climbingOrRehoming) {
      if (getState().clawGp == ClawGamePiece.ALGAE) {
        algaeNetRequest();
      } else {
        l4CoralApproachRequest();
      }
    }
  }

  public void lowLineupRequest() {
    if (!getState().climbingOrRehoming) {
      if (getState().clawGp == ClawGamePiece.ALGAE) {
        processorWaitingRequest();
      } else {
        l1CoralApproachRequest();
      }
    }
  }

  public void l4CoralLeftReleaseRequest() {
    if (!getState().climbingOrRehoming) {
      if (claw.getHasGP()) {
        setStateFromRequest(RobotState.CORAL_L4_LEFT_RELEASE);
      } else {
        setStateFromRequest(RobotState.CORAL_L4_PREPARE_HANDOFF);
      }
    }
  }

  public void algaeReefIntakeRequest() {
    if (!getState().climbingOrRehoming) {
      scoringAlignActive = true;
      if (robotScoringSide == RobotScoringSide.LEFT) {
        if (nearestReefSide.algaeHeight == ReefPipeLevel.L3) {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L3_LEFT_APPROACH);
        } else {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L2_LEFT_APPROACH);
        }
      } else {
        if (nearestReefSide.algaeHeight == ReefPipeLevel.L3) {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L3_RIGHT_APPROACH);
        } else {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L2_RIGHT_APPROACH);
        }
      }
    }
  }

  public void algaeNetRequest() {
    if (!vision.isAnyTagLimelightOnline()
        || AutoAlign.getNetScoringSideFromRobotPose(robotPose) == RobotScoringSide.LEFT) {
      algaeNetLeftRequest();
    } else {
      algaeNetRightRequest();
    }
  }

  private void algaeNetLeftRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.ALGAE_NET_LEFT_WAITING);
    }
  }

  private void algaeNetRightRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.ALGAE_NET_RIGHT_WAITING);
    }
  }

  public void preloadCoralRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.CLAW_EMPTY);
    }
  }

  public void confirmScoreRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBER_STOP,
          CORAL_L1_PREPARE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF,
          CORAL_L1_RELEASE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF,
          CORAL_L1_RIGHT_APPROACH,
          CORAL_L2_LEFT_APPROACH,
          CORAL_L2_RIGHT_APPROACH,
          CORAL_L3_LEFT_APPROACH,
          CORAL_L3_RIGHT_APPROACH,
          CORAL_L4_LEFT_APPROACH,
          CORAL_L4_RIGHT_APPROACH -> {}

      case ALGAE_FLING_WAIT -> setStateFromRequest(RobotState.ALGAE_FLING_PREPARE);

      case CORAL_L2_LEFT_LINEUP,
          CORAL_L3_LEFT_LINEUP,
          CORAL_L4_LEFT_LINEUP,
          CORAL_L1_RIGHT_LINEUP,
          CORAL_L2_RIGHT_LINEUP,
          CORAL_L3_RIGHT_LINEUP,
          CORAL_L4_RIGHT_LINEUP -> {
        setStateFromRequest(getState().getLineupToPlaceState());
      }

      case CORAL_L2_LEFT_PLACE,
          CORAL_L3_LEFT_PLACE,
          CORAL_L4_LEFT_PLACE,
          CORAL_L2_RIGHT_PLACE,
          CORAL_L3_RIGHT_PLACE,
          CORAL_L4_RIGHT_PLACE -> {
        setStateFromRequest(getState().getPlaceToReleaseState());
      }

      case CLAW_EMPTY, CLAW_ALGAE -> {
        if (groundManager.hasCoral()) {
          groundManager.l1Request();
        } else {
          setStateFromRequest(RobotState.ALGAE_OUTTAKE);
        }
      }

      case CLAW_CORAL -> l4CoralApproachRequest();
      case ALGAE_PROCESSOR_WAITING -> setStateFromRequest(RobotState.ALGAE_PROCESSOR_RELEASE);

      case ALGAE_NET_LEFT_WAITING -> setStateFromRequest(RobotState.ALGAE_NET_LEFT_RELEASE);

      case ALGAE_NET_RIGHT_WAITING -> setStateFromRequest(RobotState.ALGAE_NET_RIGHT_RELEASE);

      default -> {}
    }
  }

  public void nextClimbStateRequest() {
    switch (getState()) {
      case CLIMBER_STOP -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void stopClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBER_STOP);
      default -> {}
    }
  }

  public void unjamRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void rehomeElevatorRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.REHOME_ELEVATOR);
    }
  }

  public void spinToWinRequest() {
    if (!getState().climbingOrRehoming && FeatureFlags.SPIN_TO_WIN.getAsBoolean()) {
      setStateFromRequest(RobotState.PREPARE_SPIN_TO_WIN);
    }
  }

  private ElevatorState latestElevatorGoal = ElevatorState.STOWED;
  private ArmState latestArmGoal = ArmState.HOLDING_UPRIGHT;
  private boolean latestUnsafe = false;

  private void moveSuperstructure(ElevatorState elevatorGoal, ArmState armGoal) {
    moveSuperstructure(elevatorGoal, armGoal, false);
  }

  private void moveSuperstructure(ElevatorState elevatorGoal, ArmState armGoal, boolean unsafe) {
    latestElevatorGoal = elevatorGoal;
    latestArmGoal = armGoal;
    latestUnsafe = unsafe;

    var currentPosition = new SuperstructurePosition(elevator.getHeight(), arm.getAngle());
    var goal = new SuperstructurePosition(elevatorGoal.getHeight(), armGoal.getAngle());

    MechanismVisualizer.log(currentPosition, groundManager.deploy.getAngle());
    var obstructionKind =
        FeatureFlags.COLLISION_AVOIDANCE_OBSTRUCTION.getAsBoolean()
            ? shouldLoopAroundToScoreObstruction
            : ObstructionKind.NONE;
    var maybeCollisionAvoidanceResult =
        CollisionAvoidance.routePosition(currentPosition, goal, obstructionKind, arm.getRawAngle());

    if (unsafe || maybeCollisionAvoidanceResult.isEmpty()) {
      elevator.setState(elevatorGoal);
      arm.setState(armGoal);
    } else {
      var collisionAvoidanceResult = maybeCollisionAvoidanceResult.orElseThrow();

      elevator.setCollisionAvoidanceGoal(collisionAvoidanceResult.elevatorHeight());
      elevator.setState(ElevatorState.COLLISION_AVOIDANCE);

      arm.setCollisionAvoidanceGoal(
          collisionAvoidanceResult.armAngle());
      arm.setState(ArmState.COLLISION_AVOIDANCE);
    }
  }

  private LightsState getLightStateForScoring() {
    return switch (autoAlign.getReefAlignState()) {
      case ALL_CAMERAS_DEAD -> LightsState.ERROR;
      case HAS_TAGS_IN_POSITION, HAS_TAGS_WRONG_POSITION -> LightsState.SCORE_ALIGN_TAGS;
      default -> LightsState.SCORE_ALIGN_NO_TAGS;
    };
  }

  private LightsState getLightsStateForLollipop() {
    return vision.getLollipopVisionResult().isPresent()
        ? LightsState.HOLDING_ALGAE
        : LightsState.SCORE_ALIGN_NO_TAGS;
  }
}
