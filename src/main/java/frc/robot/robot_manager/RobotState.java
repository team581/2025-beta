package frc.robot.robot_manager;

import com.google.common.collect.ImmutableMap;
import frc.robot.auto_align.RobotScoringSide;
import java.util.Map;

public enum RobotState {
  // Idle states
  /** Idle without any game piece. */
  CLAW_EMPTY(ClawGamePiece.EMPTY, false),
  /** Claw holding algae */
  CLAW_ALGAE(ClawGamePiece.ALGAE, false),
  /** Claw holding coral */
  CLAW_CORAL(ClawGamePiece.CORAL, false),

  // Intake states
  // In theory we could have intake upright while holding algae but nobody is going to use that
  CORAL_INTAKE_LOLLIPOP_APPROACH(ClawGamePiece.EMPTY, false),
  CORAL_INTAKE_LOLLIPOP_PUSH(ClawGamePiece.CORAL, false),
  CORAL_INTAKE_LOLLIPOP_GRAB(ClawGamePiece.EMPTY, false),

  ALGAE_INTAKE_FLOOR(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L2_LEFT_APPROACH(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_LEFT_APPROACH(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L2_RIGHT_APPROACH(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_RIGHT_APPROACH(ClawGamePiece.ALGAE, false),

  ALGAE_INTAKE_L2_LEFT_HOLDING(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_LEFT_HOLDING(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L2_RIGHT_HOLDING(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_RIGHT_HOLDING(ClawGamePiece.ALGAE, false),

  ALGAE_INTAKE_L2_LEFT(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_LEFT(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L2_RIGHT(ClawGamePiece.ALGAE, false),
  ALGAE_INTAKE_L3_RIGHT(ClawGamePiece.ALGAE, false),

  ALGAE_FLING_WAIT(ClawGamePiece.ALGAE, false),
  ALGAE_FLING_PREPARE(ClawGamePiece.ALGAE, false),
  ALGAE_FLING_RELEASE(ClawGamePiece.ALGAE, false),

  PREPARE_HANDOFF_AFTER_INTAKE(ClawGamePiece.EMPTY, false),

  // L1 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L1_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L1_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  /** Coral is in the claw, let's get ready to score L1. */
  CORAL_L1_RIGHT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L1_RIGHT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L1_RIGHT_RELEASE(ClawGamePiece.CORAL, false),

  // L2 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L2_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L2_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),

  /** Coral is in the claw, let's get ready to score L2. */
  CORAL_L2_LEFT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L2_RIGHT_APPROACH(ClawGamePiece.CORAL, false),

  CORAL_L2_LEFT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L2_RIGHT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L2_LEFT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L2_RIGHT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L2_LEFT_RELEASE(ClawGamePiece.CORAL, false),
  CORAL_L2_RIGHT_RELEASE(ClawGamePiece.CORAL, false),

  // L3 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L3_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L3_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  /** Coral is in the claw, let's get ready to score L3. */
  CORAL_L3_LEFT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L3_RIGHT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L3_LEFT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L3_RIGHT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L3_LEFT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L3_RIGHT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L3_LEFT_RELEASE(ClawGamePiece.CORAL, false),
  CORAL_L3_RIGHT_RELEASE(ClawGamePiece.CORAL, false),

  // L4 scoring using the claw
  /** Coral is in the ground intake, need to pass it to the claw. */
  CORAL_L4_PREPARE_HANDOFF(ClawGamePiece.EMPTY, false),
  CORAL_L4_RELEASE_HANDOFF(ClawGamePiece.EMPTY, false),
  /** Coral is in the claw, let's get ready to score L4. */
  CORAL_L4_LEFT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L4_RIGHT_APPROACH(ClawGamePiece.CORAL, false),
  CORAL_L4_LEFT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L4_RIGHT_LINEUP(ClawGamePiece.CORAL, false),
  CORAL_L4_LEFT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L4_RIGHT_PLACE(ClawGamePiece.CORAL, false),
  CORAL_L4_LEFT_RELEASE(ClawGamePiece.CORAL, false),
  CORAL_L4_RIGHT_RELEASE(ClawGamePiece.CORAL, false),

  // Algae scoring states
  ALGAE_NET_LEFT_WAITING(ClawGamePiece.ALGAE, false),
  ALGAE_NET_LEFT_RELEASE(ClawGamePiece.ALGAE, false),

  ALGAE_NET_RIGHT_WAITING(ClawGamePiece.ALGAE, false),
  ALGAE_NET_RIGHT_RELEASE(ClawGamePiece.ALGAE, false),

  ALGAE_PROCESSOR_WAITING(ClawGamePiece.ALGAE, false),
  ALGAE_PROCESSOR_RELEASE(ClawGamePiece.ALGAE, false),

  // Climbing states
  CLIMBING_1_LINEUP(ClawGamePiece.EMPTY, true),
  CLIMBING_2_HANGING(ClawGamePiece.EMPTY, true),
  CLIMBER_STOP(ClawGamePiece.EMPTY, true),

  // Misc states
  PREPARE_SPIN_TO_WIN(ClawGamePiece.EMPTY, false),
  SPIN_TO_WIN(ClawGamePiece.EMPTY, false),
  ALGAE_OUTTAKE(ClawGamePiece.ALGAE, false),
  UNJAM(ClawGamePiece.EMPTY, false),
  REHOME_ELEVATOR(ClawGamePiece.EMPTY, true),
  REHOME_ARM(ClawGamePiece.EMPTY, true);

  public final ClawGamePiece clawGp;
  public final boolean climbingOrRehoming;

  private RobotState(ClawGamePiece clawGp, boolean climbingOrRehoming) {
    this.clawGp = clawGp;
    this.climbingOrRehoming = climbingOrRehoming;
  }

  private static final ImmutableMap<RobotState, RobotState> lineupToPlace =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L2_LEFT_LINEUP, CORAL_L2_LEFT_PLACE),
          Map.entry(CORAL_L3_LEFT_LINEUP, CORAL_L3_LEFT_PLACE),
          Map.entry(CORAL_L4_LEFT_LINEUP, CORAL_L4_LEFT_PLACE),
          Map.entry(CORAL_L1_RIGHT_LINEUP, CORAL_L1_RIGHT_RELEASE),
          Map.entry(CORAL_L2_RIGHT_LINEUP, CORAL_L2_RIGHT_PLACE),
          Map.entry(CORAL_L3_RIGHT_LINEUP, CORAL_L3_RIGHT_PLACE),
          Map.entry(CORAL_L4_RIGHT_LINEUP, CORAL_L4_RIGHT_PLACE));
  private static final ImmutableMap<RobotState, RobotState> placeToRelease =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L2_LEFT_PLACE, CORAL_L2_LEFT_RELEASE),
          Map.entry(CORAL_L3_LEFT_PLACE, CORAL_L3_LEFT_RELEASE),
          Map.entry(CORAL_L4_LEFT_PLACE, CORAL_L4_LEFT_RELEASE),
          Map.entry(CORAL_L1_RIGHT_RELEASE, CORAL_L1_RIGHT_RELEASE),
          Map.entry(CORAL_L2_RIGHT_PLACE, CORAL_L2_RIGHT_RELEASE),
          Map.entry(CORAL_L3_RIGHT_PLACE, CORAL_L3_RIGHT_RELEASE),
          Map.entry(CORAL_L4_RIGHT_PLACE, CORAL_L4_RIGHT_RELEASE));

  private static final ImmutableMap<RobotState, RobotState> handoffPrepareToRelease =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L1_PREPARE_HANDOFF, CORAL_L1_RELEASE_HANDOFF),
          Map.entry(CORAL_L2_PREPARE_HANDOFF, CORAL_L2_RELEASE_HANDOFF),
          Map.entry(CORAL_L3_PREPARE_HANDOFF, CORAL_L3_RELEASE_HANDOFF),
          Map.entry(CORAL_L4_PREPARE_HANDOFF, CORAL_L4_RELEASE_HANDOFF));
  private static final ImmutableMap<RobotState, RobotState> handoffReleaseToLeftApproach =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L1_RELEASE_HANDOFF, CORAL_L1_RIGHT_APPROACH),
          Map.entry(CORAL_L2_RELEASE_HANDOFF, CORAL_L2_LEFT_APPROACH),
          Map.entry(CORAL_L3_RELEASE_HANDOFF, CORAL_L3_LEFT_APPROACH),
          Map.entry(CORAL_L4_RELEASE_HANDOFF, CORAL_L4_LEFT_APPROACH));
  private static final ImmutableMap<RobotState, RobotState> handoffReleaseToRightApproach =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L1_RELEASE_HANDOFF, CORAL_L1_RIGHT_APPROACH),
          Map.entry(CORAL_L2_RELEASE_HANDOFF, CORAL_L2_RIGHT_APPROACH),
          Map.entry(CORAL_L3_RELEASE_HANDOFF, CORAL_L3_RIGHT_APPROACH),
          Map.entry(CORAL_L4_RELEASE_HANDOFF, CORAL_L4_RIGHT_APPROACH));
  private static final ImmutableMap<RobotState, RobotState> approachToLineupLeftStates =
      ImmutableMap.ofEntries(
          // Go to right l1 since robot can't do left l1
          Map.entry(CORAL_L1_RIGHT_APPROACH, CORAL_L1_RIGHT_LINEUP),
          Map.entry(CORAL_L2_LEFT_APPROACH, CORAL_L2_LEFT_LINEUP),
          Map.entry(CORAL_L3_LEFT_APPROACH, CORAL_L3_LEFT_LINEUP),
          Map.entry(CORAL_L4_LEFT_APPROACH, CORAL_L4_LEFT_LINEUP));

  private static final ImmutableMap<RobotState, RobotState> algaeApproachToIntakeStates =
      ImmutableMap.ofEntries(
          // Go to right l1 since robot can't do left l1
          Map.entry(ALGAE_INTAKE_L2_LEFT_APPROACH, ALGAE_INTAKE_L2_LEFT),
          Map.entry(ALGAE_INTAKE_L3_LEFT_APPROACH, ALGAE_INTAKE_L3_LEFT),
          Map.entry(ALGAE_INTAKE_L2_RIGHT_APPROACH, ALGAE_INTAKE_L2_RIGHT),
          Map.entry(ALGAE_INTAKE_L3_RIGHT_APPROACH, ALGAE_INTAKE_L3_RIGHT));

  private static final ImmutableMap<RobotState, RobotState> algaeIntakeToHoldingStates =
      ImmutableMap.ofEntries(
          // Go to right l1 since robot can't do left l1
          Map.entry(ALGAE_INTAKE_L2_LEFT, ALGAE_INTAKE_L2_LEFT_HOLDING),
          Map.entry(ALGAE_INTAKE_L3_LEFT, ALGAE_INTAKE_L3_LEFT_HOLDING),
          Map.entry(ALGAE_INTAKE_L2_RIGHT, ALGAE_INTAKE_L2_RIGHT_HOLDING),
          Map.entry(ALGAE_INTAKE_L3_RIGHT, ALGAE_INTAKE_L3_RIGHT_HOLDING));
  private static final ImmutableMap<RobotState, RobotState> approachToLineupRightStates =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L1_RIGHT_APPROACH, CORAL_L1_RIGHT_LINEUP),
          Map.entry(CORAL_L2_RIGHT_APPROACH, CORAL_L2_RIGHT_LINEUP),
          Map.entry(CORAL_L3_RIGHT_APPROACH, CORAL_L3_RIGHT_LINEUP),
          Map.entry(CORAL_L4_RIGHT_APPROACH, CORAL_L4_RIGHT_LINEUP));

  private static final ImmutableMap<RobotState, RobotState> rightToLeftApproachStates =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L2_RIGHT_APPROACH, CORAL_L2_LEFT_APPROACH),
          Map.entry(CORAL_L3_RIGHT_APPROACH, CORAL_L3_LEFT_APPROACH),
          Map.entry(CORAL_L4_RIGHT_APPROACH, CORAL_L4_LEFT_APPROACH));

  private static final ImmutableMap<RobotState, RobotState> leftToRightApproachStates =
      ImmutableMap.ofEntries(
          Map.entry(CORAL_L2_LEFT_APPROACH, CORAL_L2_RIGHT_APPROACH),
          Map.entry(CORAL_L3_LEFT_APPROACH, CORAL_L3_RIGHT_APPROACH),
          Map.entry(CORAL_L4_LEFT_APPROACH, CORAL_L4_RIGHT_APPROACH));

  public RobotState getLineupToPlaceState() {
    return lineupToPlace.getOrDefault(this, this);
  }

  public RobotState getPlaceToReleaseState() {
    return placeToRelease.getOrDefault(this, this);
  }

  public RobotState getAlgaeApproachToIntakeState() {
    return algaeApproachToIntakeStates.getOrDefault(this, this);
  }

  public RobotState getAlgaeIntakeToHoldingState() {
    return algaeIntakeToHoldingStates.getOrDefault(this, this);
  }

  public RobotState getHandoffPrepareToReleaseState() {
    return handoffPrepareToRelease.getOrDefault(this, this);
  }

  public RobotState getHandoffReleaseToApproachState(RobotScoringSide scoringSide) {
    var map =
        scoringSide == RobotScoringSide.LEFT
            ? handoffReleaseToLeftApproach
            : handoffReleaseToRightApproach;
    return map.getOrDefault(this, this);
  }

  public RobotState getLeftApproachToLineupState() {

    return approachToLineupLeftStates.getOrDefault(this, this);
  }

  public RobotState getRightApproachToLineupState() {
    return approachToLineupRightStates.getOrDefault(this, this);
  }

  public RobotState getRightToLeftApproachState() {
    return rightToLeftApproachStates.getOrDefault(this, this);
  }

  public RobotState getLeftToRightApproachState() {
    return leftToRightApproachStates.getOrDefault(this, this);
  }
}
