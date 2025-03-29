package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefSide;
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
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.game_piece_detection.CoralMap;

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
  private ReefPipeLevel scoringLevel = ReefPipeLevel.BASE;
  private Pose2d robotPose;
  private ObstructionKind shouldLoopAroundToScoreObstruction = ObstructionKind.NONE;

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case CLAW_EMPTY,
              CLAW_ALGAE,
              CLAW_CORAL,
              ALGAE_PROCESSOR_WAITING,
              ALGAE_NET_LEFT_WAITING,
              ALGAE_NET_RIGHT_WAITING,
              CLIMBING_2_HANGING,
              CLIMBER_STOP,
              UNJAM,
              ALGAE_OUTTAKE ->
          currentState;

      case REHOME_ELEVATOR ->
          elevator.getState() == ElevatorState.STOWED ? RobotState.CLAW_EMPTY : currentState;
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
          claw.getHasGP() && !groundManager.hasCoral()
              ? currentState.getHandoffReleaseToApproachState()
              : currentState;

      // Aproach
      case CORAL_L1_APPROACH, CORAL_L2_APPROACH, CORAL_L3_APPROACH, CORAL_L4_APPROACH ->
          AutoAlign.isCloseToReefSide(
                  robotPose, nearestReefSide.getPose(), swerve.getFieldRelativeSpeeds())
              ? currentState.getApproachToLineupState(robotScoringSide)
              : currentState;

      // Reef lineup states
      case CORAL_L1_LEFT_LINEUP,
              CORAL_L1_RIGHT_LINEUP,
              CORAL_L2_LEFT_LINEUP,
              CORAL_L2_RIGHT_LINEUP,
              CORAL_L3_LEFT_LINEUP,
              CORAL_L3_RIGHT_LINEUP,
              CORAL_L4_LEFT_LINEUP,
              CORAL_L4_RIGHT_LINEUP ->
          elevator.atGoal() && arm.atGoal() ? currentState.getLineupToReleaseState() : currentState;

      case CORAL_L1_LEFT_RELEASE,
          CORAL_L2_LEFT_RELEASE,
          CORAL_L3_LEFT_RELEASE,
          CORAL_L4_LEFT_RELEASE,
          CORAL_L1_RIGHT_RELEASE,
          CORAL_L2_RIGHT_RELEASE,
          CORAL_L3_RIGHT_RELEASE,
          CORAL_L4_RIGHT_RELEASE -> {
        if (arm.atGoal() && elevator.atGoal()) {
          if (DriverStation.isTeleop()) {
            if (cameraOnlineAndFarEnoughFromReef()) {
              rumbleController.rumbleRequest();
              yield RobotState.CLAW_EMPTY;
            }
          } else if (!claw.getHasGP() || timeout(0.5)) {
            yield RobotState.CLAW_EMPTY;
          }
        }

        yield currentState;
      }

      // Algae scoring
      case ALGAE_PROCESSOR_RELEASE, ALGAE_NET_LEFT_RELEASE, ALGAE_NET_RIGHT_RELEASE ->
          timeout(0.5) || !claw.getHasGP() ? RobotState.CLAW_EMPTY : currentState;

      // Intaking
      case ALGAE_INTAKE_FLOOR -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
          yield RobotState.CLAW_ALGAE;
        }
        yield currentState;
      }
      case ALGAE_INTAKE_L2_LEFT,
          ALGAE_INTAKE_L3_LEFT,
          ALGAE_INTAKE_L2_RIGHT,
          ALGAE_INTAKE_L3_RIGHT -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
          if (cameraOnlineAndFarEnoughFromReef()) {
            yield RobotState.CLAW_ALGAE;
          }
        }

        yield currentState;
      }

      case CORAL_INTAKE_LOLLIPOP -> {
        if (claw.getHasGP()) {
          yield RobotState.CLAW_CORAL;
        }

        yield currentState;
      }

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
        moveSuperstructure(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_EMPTY);
        climber.setState(ClimberState.STOWED);
      }
      case CLAW_ALGAE -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        groundManager.idleRequest();
        moveSuperstructure(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case CLAW_CORAL -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        groundManager.idleRequest();
        moveSuperstructure(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_CORAL);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_INTAKE_FLOOR -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.GROUND_ALGAE_INTAKE, ArmState.ALGAE_INTAKE_FLOOR);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_INTAKE_L2_LEFT -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2_LEFT, ArmState.ALGAE_INTAKE_LEFT_L2);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_INTAKE_L2_RIGHT -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L2_RIGHT, ArmState.ALGAE_INTAKE_RIGHT_L2);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_INTAKE_L3_LEFT -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3_LEFT, ArmState.ALGAE_INTAKE_LEFT_L3);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_INTAKE_L3_RIGHT -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_L3_RIGHT, ArmState.ALGAE_INTAKE_RIGHT_L3);
        swerve.scoringAlignmentRequest(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_INTAKE_LOLLIPOP -> {
        claw.setState(ClawState.LOLLIPOP_CORAL_INTAKE);
        moveSuperstructure(
            ElevatorState.GROUND_CORAL_INTAKE_UPRIGHT, ArmState.LOLLIPOP_CORAL_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CORAL_DETECTION);
        lights.setState(getLightsStateForLollipop());
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_LEFT_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_NET_LEFT, ArmState.ALGAE_NET_LEFT);
        swerve.snapsDriveRequest(SnapUtil.getLeftNetDirection(robotPose));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_RIGHT_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_NET_RIGHT, ArmState.ALGAE_NET_RIGHT);
        swerve.snapsDriveRequest(SnapUtil.getRightNetDirection(robotPose));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_LEFT_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.ALGAE_NET_LEFT, ArmState.ALGAE_NET_LEFT);
        swerve.snapsDriveRequest(SnapUtil.getLeftNetDirection(robotPose));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_NET_RIGHT_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.ALGAE_NET_RIGHT, ArmState.ALGAE_NET_RIGHT);
        swerve.snapsDriveRequest(SnapUtil.getRightNetDirection(robotPose));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_PROCESSOR_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_PROCESSOR_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_PROCESSOR);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      // Handoff states
      case CORAL_L1_PREPARE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        groundManager.handoffWaitRequest();
        moveSuperstructure(ElevatorState.CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_RELEASE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        groundManager.handoffReleaseRequest();
        moveSuperstructure(ElevatorState.CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOWED);
      }
      // Approach states
      case CORAL_L1_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_NOT_READY);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L2_APPROACH, CORAL_L3_APPROACH, CORAL_L4_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_NOT_READY);
        climber.setState(ClimberState.STOWED);
      }
      // L1
      case CORAL_L1_LEFT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_LINEUP_L1, ArmState.CORAL_SCORE_LEFT_LINEUP_L1);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_READY);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_LEFT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_RELEASE_L1, ArmState.CORAL_SCORE_LEFT_RELEASE_L1);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_RIGHT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L1, ArmState.CORAL_SCORE_RIGHT_LINEUP_L1);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_READY);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L1_RIGHT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L1, ArmState.CORAL_SCORE_RIGHT_RELEASE_L1);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      // L2
      case CORAL_L2_LEFT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_READY);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L2_LEFT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L2_RIGHT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L2, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_READY);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L2_RIGHT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L2, ArmState.CORAL_SCORE_RIGHT_RELEASE_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      // L3
      case CORAL_L3_LEFT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_READY);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L3_LEFT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L3_RIGHT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L2, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_READY);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L3_RIGHT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L2, ArmState.CORAL_SCORE_RIGHT_RELEASE_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      // L4
      case CORAL_L4_LEFT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_LINEUP_L4, ArmState.CORAL_SCORE_LEFT_LINEUP_L4);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_READY);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L4_LEFT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LEFT_RELEASE_L4, ArmState.CORAL_SCORE_LEFT_RELEASE_L4);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L4_RIGHT_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L4, ArmState.CORAL_SCORE_RIGHT_LINEUP_L4);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORE_ALIGN_READY);
        climber.setState(ClimberState.STOWED);
      }
      case CORAL_L4_RIGHT_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L4, ArmState.CORAL_SCORE_RIGHT_RELEASE_L4);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING);
        climber.setState(ClimberState.STOWED);
      }
      // TODO: Create special light states for climbing, unjam, and rehoming
      case CLIMBING_1_LINEUP -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.CLIMBING, ArmState.CLIMBING);
        swerve.climbRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CLIMB_LINEUP);
        climber.setState(ClimberState.LINEUP);
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
        climber.setState(ClimberState.STOWED);
      }
      case UNJAM -> {
        claw.setState(ClawState.OUTTAKING);
        groundManager.unjamRequest();
        moveSuperstructure(ElevatorState.UNJAM, ArmState.UNJAM);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.OTHER);
        climber.setState(ClimberState.STOWED);
      }
      case REHOME_ELEVATOR -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.MID_MATCH_HOMING, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.OTHER);
        climber.setState(ClimberState.STOWED);
      }
      case ALGAE_OUTTAKE -> {
        claw.setState(ClawState.OUTTAKING);
        moveSuperstructure(ElevatorState.ALGAE_OUTTAKE, ArmState.ALGAE_OUTTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOWED);
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

    // Update snaps
    switch (getState()) {
      case ALGAE_INTAKE_L2_LEFT,
          ALGAE_INTAKE_L3_LEFT,
          ALGAE_INTAKE_L2_RIGHT,
          ALGAE_INTAKE_L3_RIGHT -> {
        swerve.scoringAlignmentRequest(reefSnapAngle);
      }
      case CORAL_L1_LEFT_LINEUP,
          CORAL_L1_LEFT_RELEASE,
          CORAL_L2_LEFT_LINEUP,
          CORAL_L2_LEFT_RELEASE,
          CORAL_L3_LEFT_LINEUP,
          CORAL_L3_LEFT_RELEASE,
          CORAL_L4_LEFT_LINEUP,
          CORAL_L4_LEFT_RELEASE,
          CORAL_L1_RIGHT_LINEUP,
          CORAL_L1_RIGHT_RELEASE,
          CORAL_L2_RIGHT_LINEUP,
          CORAL_L2_RIGHT_RELEASE,
          CORAL_L3_RIGHT_LINEUP,
          CORAL_L3_RIGHT_RELEASE,
          CORAL_L4_RIGHT_LINEUP,
          CORAL_L4_RIGHT_RELEASE -> {
        swerve.scoringAlignmentRequest(reefSnapAngle);
      }
      case CORAL_INTAKE_LOLLIPOP -> {
        coralMap.updateLollipopResult(vision.getLollipopVisionResult());
      }
      default -> {}
    }

    // Update lights
    switch (getState()) {
      // TODO: Add lights state for scoring
      case CORAL_INTAKE_LOLLIPOP -> {
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
      } else if (vision.getTagResult().isEmpty()) {
        lights.setDisabledState(LightsState.SCORE_ALIGN_NOT_READY);
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
    robotPose = localization.getPose();
    robotScoringSide =
        AutoAlign.getScoringSideFromRobotPose(
            robotPose,
            vision.isAnyLeftScoringTagLimelightOnline(),
            vision.isAnyRightScoringTagLimelightOnline());
    autoAlign.setScoringLevel(scoringLevel, robotScoringSide);
    shouldLoopAroundToScoreObstruction = autoAlign.shouldArmGoAroundToScore();
    reefSnapAngle = autoAlign.getUsedScoringPose().getRotation().getDegrees();
    scoringLevel =
        switch (getState()) {
          case CORAL_L1_LEFT_LINEUP,
                  CORAL_L1_RIGHT_LINEUP,
                  CORAL_L1_LEFT_RELEASE,
                  CORAL_L1_RIGHT_RELEASE ->
              ReefPipeLevel.L1;
          case CORAL_L2_LEFT_LINEUP,
                  CORAL_L2_RIGHT_LINEUP,
                  CORAL_L2_LEFT_RELEASE,
                  CORAL_L2_RIGHT_RELEASE ->
              ReefPipeLevel.L2;
          case CORAL_L3_LEFT_LINEUP,
                  CORAL_L3_RIGHT_LINEUP,
                  CORAL_L3_LEFT_RELEASE,
                  CORAL_L3_RIGHT_RELEASE ->
              ReefPipeLevel.L3;
          case CORAL_L4_LEFT_LINEUP,
                  CORAL_L4_RIGHT_LINEUP,
                  CORAL_L4_LEFT_RELEASE,
                  CORAL_L4_RIGHT_RELEASE ->
              ReefPipeLevel.L4;
          default -> ReefPipeLevel.BASE;
        };

    DogLog.log("AutoAlign/UsedPose", autoAlign.getUsedScoringPose());

    vision.setClosestScoringReefAndPipe(nearestReefSide.getTagID(), autoAlign.getBestReefPipe());
    vision.updateDistanceFromReef(
        robotPose.getTranslation().getDistance(nearestReefSide.getPose().getTranslation()));

    autoAlign.setTeleopSpeeds(swerve.getTeleopSpeeds());
    if (vision.isAnyTagLimelightOnline() || DriverStation.isAutonomous()) {
      var idealAlignSpeeds =
          switch (getState()) {
            case ALGAE_INTAKE_L2_LEFT,
                    ALGAE_INTAKE_L3_LEFT,
                    ALGAE_INTAKE_L2_RIGHT,
                    ALGAE_INTAKE_L3_RIGHT ->
                autoAlign.getAlgaeAlignSpeeds();
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
    var tagCameraOnline = vision.isAnyTagLimelightOnline();

    if (!tagCameraOnline) {
      return timeout(0.5);
    }

    var isFarEnoughFromReefSide =
        !AutoAlign.isCloseToReefSide(robotPose, nearestReefSide.getPose(), 0.75);

    return isFarEnoughFromReefSide;
  }

  public void forceIdleNoGp() {
    setStateFromRequest(RobotState.CLAW_EMPTY);
  }

  public void stowRequest() {
    switch (getState()) {
      case ALGAE_INTAKE_FLOOR,
          ALGAE_INTAKE_L2_LEFT,
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

  public void intakeFloorAlgaeRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.ALGAE_INTAKE_FLOOR);
    }
  }

  public void intakeAssistFloorCoralHorizontalRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.CORAL_INTAKE_LOLLIPOP);
    }
  }

  public void processorWaitingRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.ALGAE_PROCESSOR_WAITING);
    }
  }

  public void l4CoralApproachRequest() {
    if (!getState().climbingOrRehoming) {
      if (claw.getHasGP()) {
        setStateFromRequest(RobotState.CORAL_L4_APPROACH);
      } else {
        setStateFromRequest(RobotState.CORAL_L4_PREPARE_HANDOFF);
      }
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

  public void l3LineupRequest() {
    if (!getState().climbingOrRehoming) {
      if (claw.getHasGP()) {
        setStateFromRequest(RobotState.CORAL_L3_APPROACH);
      } else {
        setStateFromRequest(RobotState.CORAL_L3_PREPARE_HANDOFF);
      }
    }
  }

  public void l2LineupRequest() {
    if (!getState().climbingOrRehoming) {
      if (claw.getHasGP()) {
        setStateFromRequest(RobotState.CORAL_L2_APPROACH);
      } else {
        setStateFromRequest(RobotState.CORAL_L2_PREPARE_HANDOFF);
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

  public void l1CoralApproachRequest() {
    if (!getState().climbingOrRehoming) {
      if (claw.getHasGP()) {
        setStateFromRequest(RobotState.CORAL_L1_APPROACH);
      } else {
        setStateFromRequest(RobotState.CORAL_L1_PREPARE_HANDOFF);
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
      if (robotScoringSide == RobotScoringSide.LEFT) {
        if (nearestReefSide.algaeHeight == ReefPipeLevel.L3) {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L3_LEFT);
        } else {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L2_LEFT);
        }
      } else {
        if (nearestReefSide.algaeHeight == ReefPipeLevel.L3) {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L3_RIGHT);
        } else {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L2_RIGHT);
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
      setStateFromRequest(RobotState.ALGAE_NET_RIGHT_WAITING);
    }
  }

  private void algaeNetRightRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.ALGAE_NET_LEFT_WAITING);
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
          CORAL_L1_APPROACH,
          CORAL_L2_APPROACH,
          CORAL_L3_APPROACH,
          CORAL_L4_APPROACH,
          CORAL_L1_LEFT_LINEUP,
          CORAL_L1_LEFT_RELEASE,
          CORAL_L2_LEFT_LINEUP,
          CORAL_L2_LEFT_RELEASE,
          CORAL_L3_LEFT_LINEUP,
          CORAL_L3_LEFT_RELEASE,
          CORAL_L4_LEFT_LINEUP,
          CORAL_L4_LEFT_RELEASE,
          CORAL_L1_RIGHT_LINEUP,
          CORAL_L1_RIGHT_RELEASE,
          CORAL_L2_RIGHT_LINEUP,
          CORAL_L2_RIGHT_RELEASE,
          CORAL_L3_RIGHT_LINEUP,
          CORAL_L3_RIGHT_RELEASE,
          CORAL_L4_RIGHT_LINEUP,
          CORAL_L4_RIGHT_RELEASE -> {}

      case CLAW_EMPTY, CLAW_ALGAE -> {
        if (groundManager.hasCoral()) {
          groundManager.l1Request();
        } else {
          setStateFromRequest(RobotState.ALGAE_OUTTAKE);
        }
      }

      case CLAW_CORAL -> l4CoralApproachRequest();
      case ALGAE_PROCESSOR_WAITING -> setStateFromRequest(RobotState.ALGAE_PROCESSOR_RELEASE);

      case ALGAE_NET_LEFT_WAITING -> setStateFromRequest(RobotState.ALGAE_NET_LEFT_WAITING);

      case ALGAE_NET_RIGHT_WAITING -> setStateFromRequest(RobotState.ALGAE_NET_RIGHT_WAITING);

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

    var maybeCollisionAvoidanceResult =
        CollisionAvoidance.route(
            new SuperstructurePosition(elevator.getHeight(), arm.getAngle()),
            new SuperstructurePosition(elevatorGoal.getHeight(), armGoal.getAngle()),
            shouldLoopAroundToScoreObstruction);

    if (unsafe || maybeCollisionAvoidanceResult.isEmpty()) {
      elevator.setState(elevatorGoal);
      arm.setState(armGoal);
    } else {
      var collisionAvoidanceResult = maybeCollisionAvoidanceResult.get();

      elevator.setCollisionAvoidanceGoal(collisionAvoidanceResult.position.elevatorHeight());
      elevator.setState(ElevatorState.COLLISION_AVOIDANCE);

      arm.setCollisionAvoidanceGoal(collisionAvoidanceResult.position.armAngle());
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

  private LightsState getLightsStateForLollipop() {
    return vision.getLollipopVisionResult().isPresent()
        ? LightsState.HOLDING_ALGAE
        : LightsState.SCORE_ALIGN_NOT_READY;
  }
}
