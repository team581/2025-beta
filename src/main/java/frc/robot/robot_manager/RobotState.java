package frc.robot.robot_manager;

import com.google.common.collect.ImmutableMap;
import frc.robot.auto_align.RobotScoringSide;
import java.util.Map;

public enum RobotState {
  // Idle states
  /** Idle without any game piece. */
  CLAW_EMPTY_DEPLOY_EMPTY(ClawGamePiece.EMPTY, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  /** Deploy is holding coral, claw isn't doing anything. */
  CLAW_EMPTY_DEPLOY_CORAL(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  /** Claw holding algae, deploy is holding coral. */
  CLAW_ALGAE_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  /** Claw holding algae, deploy is empty. */
  CLAW_ALGAE_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  /** Claw holding coral, deploy is empty. */
  CLAW_CORAL_DEPLOY_EMPTY(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),

  // Intake states
  CORAL_INTAKE_FLOOR_CLAW_EMPTY(ClawGamePiece.EMPTY, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_INTAKE_FLOOR_CLAW_ALGAE(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  // In theory we could have intake upright while holding algae but nobody is going to use that
  CORAL_INTAKE_LOLLIPOP_CLAW_EMPTY(ClawGamePiece.EMPTY, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  // Same for this, we only do this in auto, so no algae in claw
  CORAL_INTAKE_ASSIST_FLOOR_CLAW_EMPTY(ClawGamePiece.EMPTY, /* deployHasCoral= */false, /* climbingOrRehoming= */false),

  ALGAE_INTAKE_FLOOR_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_INTAKE_L2_LEFT_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_INTAKE_L3_LEFT_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_INTAKE_L2_RIGHT_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_INTAKE_L3_RIGHT_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),

  ALGAE_INTAKE_FLOOR_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  ALGAE_INTAKE_L2_LEFT_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  ALGAE_INTAKE_L3_LEFT_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  ALGAE_INTAKE_L2_RIGHT_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  ALGAE_INTAKE_L3_RIGHT_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),

  // L1 scoring using the ground intake
  CORAL_L1_DEPLOY_PREPARE_CLAW_ALGAE(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  CORAL_L1_DEPLOY_PREPARE_CLAW_EMPTY(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  CORAL_L1_DEPLOY_SCORE_CLAW_ALGAE(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  CORAL_L1_DEPLOY_SCORE_CLAW_EMPTY(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),

  // L1 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L1_PREPARE_HANDOFF(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  CORAL_L1_RELEASE_HANDOFF(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  /** Coral is in the claw, let's get ready to score L1. */
  CORAL_L1_APPROACH(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L1_LEFT_LINEUP(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L1_RIGHT_LINEUP(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L1_LEFT_RELEASE(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L1_RIGHT_RELEASE(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),

  // L2 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L2_PREPARE_HANDOFF(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  CORAL_L2_RELEASE_HANDOFF(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),

  /** Coral is in the claw, let's get ready to score L2. */
  CORAL_L2_APPROACH(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L2_LEFT_LINEUP(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L2_RIGHT_LINEUP(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L2_LEFT_RELEASE(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L2_RIGHT_RELEASE(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),

  // L3 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L3_PREPARE_HANDOFF(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  CORAL_L3_RELEASE_HANDOFF(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  /** Coral is in the claw, let's get ready to score L3. */
  CORAL_L3_APPROACH(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L3_LEFT_LINEUP(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L3_RIGHT_LINEUP(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L3_LEFT_RELEASE(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L3_RIGHT_RELEASE(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),

  // L4 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L4_PREPARE_HANDOFF(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  CORAL_L4_RELEASE_HANDOFF(ClawGamePiece.EMPTY, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  /** Coral is in the claw, let's get ready to score L4. */
  CORAL_L4_APPROACH(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L4_LEFT_LINEUP(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L4_RIGHT_LINEUP(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L4_LEFT_RELEASE(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  CORAL_L4_RIGHT_RELEASE(ClawGamePiece.CORAL, /* deployHasCoral= */false, /* climbingOrRehoming= */false),

  // Algae scoring states
  ALGAE_NET_LEFT_WAITING_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_NET_LEFT_RELEASE_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_NET_LEFT_WAITING_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  ALGAE_NET_LEFT_RELEASE_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),

  ALGAE_NET_RIGHT_WAITING_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_NET_RIGHT_RELEASE_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_NET_RIGHT_WAITING_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  ALGAE_NET_RIGHT_RELEASE_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),

  ALGAE_PROCESSOR_WAITING_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_PROCESSOR_RELEASE_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_PROCESSOR_WAITING_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  ALGAE_PROCESSOR_RELEASE_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),

  // Climbing states
  CLIMBING_1_LINEUP(ClawGamePiece.EMPTY, /* deployHasCoral= */false, /* climbingOrRehoming= */true),
  CLIMBING_2_HANGING(ClawGamePiece.EMPTY, /* deployHasCoral= */false, /* climbingOrRehoming= */true),
  CLIMBER_STOP(ClawGamePiece.EMPTY, /* deployHasCoral= */false, /* climbingOrRehoming= */true),

  // Misc states
  ALGAE_OUTTAKE_DEPLOY_EMPTY(ClawGamePiece.ALGAE, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  ALGAE_OUTTAKE_DEPLOY_CORAL(ClawGamePiece.ALGAE, /* deployHasCoral= */true, /* climbingOrRehoming= */false),
  UNJAM(ClawGamePiece.EMPTY, /* deployHasCoral= */false, /* climbingOrRehoming= */false),
  REHOME_DEPLOY(ClawGamePiece.EMPTY, /* deployHasCoral= */false, /* climbingOrRehoming= */true),
  REHOME_ELEVATOR(ClawGamePiece.EMPTY, /* deployHasCoral= */false, /* climbingOrRehoming= */true);

  public final ClawGamePiece clawGp;
  public final boolean deployHasCoral;
  public final boolean climbingOrRehoming;

  private RobotState(ClawGamePiece clawGp, boolean deployHasCoral, boolean climbingOrRehoming) {
    this.clawGp = clawGp;
    this.deployHasCoral = deployHasCoral;
    this.climbingOrRehoming = climbingOrRehoming;
  }

  private static final ImmutableMap<RobotState, RobotState> lineupToRelease =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(CORAL_L1_LEFT_LINEUP, CORAL_L1_LEFT_RELEASE),
          Map.entry(CORAL_L2_LEFT_LINEUP, CORAL_L2_LEFT_RELEASE),
          Map.entry(CORAL_L3_LEFT_LINEUP, CORAL_L3_LEFT_RELEASE),
          Map.entry(CORAL_L4_LEFT_LINEUP, CORAL_L4_LEFT_RELEASE),
          Map.entry(CORAL_L1_RIGHT_LINEUP, CORAL_L1_RIGHT_RELEASE),
          Map.entry(CORAL_L2_RIGHT_LINEUP, CORAL_L2_RIGHT_RELEASE),
          Map.entry(CORAL_L3_RIGHT_LINEUP, CORAL_L3_RIGHT_RELEASE),
          Map.entry(CORAL_L4_RIGHT_LINEUP, CORAL_L4_RIGHT_RELEASE)));

  private static final ImmutableMap<RobotState, RobotState> algaeAfterScore =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(ALGAE_PROCESSOR_RELEASE_DEPLOY_CORAL, CLAW_EMPTY_DEPLOY_CORAL),
          Map.entry(ALGAE_NET_LEFT_RELEASE_DEPLOY_CORAL, CLAW_EMPTY_DEPLOY_CORAL),
          Map.entry(ALGAE_NET_RIGHT_RELEASE_DEPLOY_CORAL, CLAW_EMPTY_DEPLOY_CORAL),
          Map.entry(ALGAE_PROCESSOR_RELEASE_DEPLOY_EMPTY, CLAW_EMPTY_DEPLOY_EMPTY),
          Map.entry(ALGAE_NET_LEFT_RELEASE_DEPLOY_EMPTY, CLAW_EMPTY_DEPLOY_EMPTY),
          Map.entry(ALGAE_NET_RIGHT_RELEASE_DEPLOY_EMPTY, CLAW_EMPTY_DEPLOY_EMPTY)));
  private static final ImmutableMap<RobotState, RobotState> algaeAfterIntake =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(ALGAE_INTAKE_L2_LEFT_DEPLOY_EMPTY, CLAW_ALGAE_DEPLOY_EMPTY),
          Map.entry(ALGAE_INTAKE_L3_LEFT_DEPLOY_EMPTY, CLAW_ALGAE_DEPLOY_EMPTY),
          Map.entry(ALGAE_INTAKE_L2_RIGHT_DEPLOY_EMPTY, CLAW_ALGAE_DEPLOY_EMPTY),
          Map.entry(ALGAE_INTAKE_L3_RIGHT_DEPLOY_EMPTY, CLAW_ALGAE_DEPLOY_EMPTY),
          Map.entry(ALGAE_INTAKE_L2_LEFT_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL),
          Map.entry(ALGAE_INTAKE_L3_LEFT_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL),
          Map.entry(ALGAE_INTAKE_L2_RIGHT_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL),
          Map.entry(ALGAE_INTAKE_L3_RIGHT_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL)));
  private static final ImmutableMap<RobotState, RobotState> algaeAfterFloorIntake =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(ALGAE_INTAKE_FLOOR_DEPLOY_EMPTY, CLAW_ALGAE_DEPLOY_EMPTY),
          Map.entry(ALGAE_INTAKE_FLOOR_DEPLOY_CORAL, CLAW_ALGAE_DEPLOY_CORAL)));
  private static final ImmutableMap<RobotState, RobotState> coralAfterIntake =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(CORAL_INTAKE_FLOOR_CLAW_EMPTY, CLAW_EMPTY_DEPLOY_CORAL),
          Map.entry(CORAL_INTAKE_LOLLIPOP_CLAW_EMPTY, CLAW_CORAL_DEPLOY_EMPTY),
          Map.entry(CORAL_INTAKE_ASSIST_FLOOR_CLAW_EMPTY, CLAW_EMPTY_DEPLOY_CORAL),
          Map.entry(CORAL_INTAKE_FLOOR_CLAW_ALGAE, CLAW_ALGAE_DEPLOY_CORAL)));
  private static final ImmutableMap<RobotState, RobotState> handoffPrepareToRelease =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(CORAL_L1_PREPARE_HANDOFF, CORAL_L1_RELEASE_HANDOFF),
          Map.entry(CORAL_L2_PREPARE_HANDOFF, CORAL_L2_RELEASE_HANDOFF),
          Map.entry(CORAL_L3_PREPARE_HANDOFF, CORAL_L3_RELEASE_HANDOFF),
          Map.entry(CORAL_L4_PREPARE_HANDOFF, CORAL_L4_RELEASE_HANDOFF)));
  private static final ImmutableMap<RobotState, RobotState> handoffReleaseToApproach =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(CORAL_L1_RELEASE_HANDOFF, CORAL_L1_APPROACH),
          Map.entry(CORAL_L2_RELEASE_HANDOFF, CORAL_L2_APPROACH),
          Map.entry(CORAL_L3_RELEASE_HANDOFF, CORAL_L3_APPROACH),
          Map.entry(CORAL_L4_RELEASE_HANDOFF, CORAL_L4_APPROACH)));
  private static final Map<RobotState, RobotState> approachToLineupLeftStates =
      Map.ofEntries(
          Map.entry(CORAL_L1_APPROACH, CORAL_L1_LEFT_LINEUP),
          Map.entry(CORAL_L2_APPROACH, CORAL_L2_LEFT_LINEUP),
          Map.entry(CORAL_L3_APPROACH, CORAL_L3_LEFT_LINEUP),
          Map.entry(CORAL_L4_APPROACH, CORAL_L4_LEFT_LINEUP));
  private static final Map<RobotState, RobotState> approachToLineupRightStates =
      Map.ofEntries(
          Map.entry(CORAL_L1_APPROACH, CORAL_L1_RIGHT_LINEUP),
          Map.entry(CORAL_L2_APPROACH, CORAL_L2_RIGHT_LINEUP),
          Map.entry(CORAL_L3_APPROACH, CORAL_L3_RIGHT_LINEUP),
          Map.entry(CORAL_L4_APPROACH, CORAL_L4_RIGHT_LINEUP));
  private static final ImmutableMap<RobotState, RobotState> deployScorePrepareToReleaseStates =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(CORAL_L1_DEPLOY_PREPARE_CLAW_EMPTY, CORAL_L1_DEPLOY_SCORE_CLAW_EMPTY),
          Map.entry(CORAL_L1_DEPLOY_PREPARE_CLAW_ALGAE, CORAL_L1_DEPLOY_SCORE_CLAW_ALGAE)));
  private static final ImmutableMap<RobotState, RobotState> afterDeployScore =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(CORAL_L1_DEPLOY_SCORE_CLAW_EMPTY, CLAW_EMPTY_DEPLOY_EMPTY),
          Map.entry(CORAL_L1_DEPLOY_SCORE_CLAW_ALGAE, CLAW_ALGAE_DEPLOY_EMPTY)));
  private static final ImmutableMap<RobotState, RobotState> idleToDeployPrepare =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(CLAW_EMPTY_DEPLOY_CORAL, CORAL_L1_DEPLOY_PREPARE_CLAW_EMPTY),
          Map.entry(CLAW_ALGAE_DEPLOY_CORAL, CORAL_L1_DEPLOY_PREPARE_CLAW_ALGAE)));
  private static final ImmutableMap<RobotState, RobotState> beforeAlgaeOuttake =
      ImmutableMap.copyOf(Map.ofEntries(
          Map.entry(CLAW_CORAL_DEPLOY_EMPTY, ALGAE_OUTTAKE_DEPLOY_EMPTY),
          Map.entry(CLAW_EMPTY_DEPLOY_EMPTY, ALGAE_OUTTAKE_DEPLOY_EMPTY),
          Map.entry(CLAW_ALGAE_DEPLOY_EMPTY, ALGAE_OUTTAKE_DEPLOY_EMPTY),
          Map.entry(CLAW_ALGAE_DEPLOY_CORAL, ALGAE_OUTTAKE_DEPLOY_CORAL),
          Map.entry(CLAW_EMPTY_DEPLOY_CORAL, ALGAE_OUTTAKE_DEPLOY_CORAL)));

  public RobotState getLineupToReleaseState() {
    return lineupToRelease.getOrDefault(this, this);
  }

  public RobotState getAlgaeAfterScore() {
    return algaeAfterScore.getOrDefault(this, this);
  }

  public RobotState getAlgaeAfterReefIntake() {
    return algaeAfterIntake.getOrDefault(this, this);
  }

  public RobotState getAlgaeAfterFloorIntake() {
    return algaeAfterFloorIntake.getOrDefault(this, this);
  }

  public RobotState getCoralAfterIntake() {
    return coralAfterIntake.getOrDefault(this, this);
  }

  public RobotState getHandoffPrepareToReleaseState() {
    return handoffPrepareToRelease.getOrDefault(this, this);
  }

  public RobotState getHandoffReleaseToApproachState() {
    return handoffReleaseToApproach.getOrDefault(this, this);
  }

  public RobotState getApproachToLineupState(RobotScoringSide scoringSide) {
    var map =
        scoringSide == RobotScoringSide.LEFT
            ? approachToLineupLeftStates
            : approachToLineupRightStates;
    return map.getOrDefault(this, this);
  }

  public RobotState getDeployScorePrepareToRelease() {
    return deployScorePrepareToReleaseStates.getOrDefault(this, this);
  }

  public RobotState getAfterDeployScore() {
    return afterDeployScore.getOrDefault(this, this);
  }

  public RobotState getAlgaeOuttakeState() {
    return beforeAlgaeOuttake.getOrDefault(this, this);
  }

  public RobotState getDeployScoreState() {
    return idleToDeployPrepare.getOrDefault(this, this);
  }
}
