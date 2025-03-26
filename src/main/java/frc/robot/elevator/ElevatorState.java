package frc.robot.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ElevatorState {
  /**
   * @deprecated This is a placeholder state.
   */
  @Deprecated
  UNTUNED(0),

  UNJAM(UNTUNED),
  PRE_MATCH_HOMING(UNTUNED),
  MID_MATCH_HOMING(UNTUNED),

  STOWED(UNTUNED),

  GROUND_CORAL_INTAKE_UPRIGHT(UNTUNED),
  GROUND_ALGAE_INTAKE(UNTUNED),

  ALGAE_OUTTAKE(UNTUNED),
  MULTICORAL_OUTTAKE(ALGAE_OUTTAKE),

  CORAL_HANDOFF(UNTUNED),

  ALGAE_INTAKE_L2_LEFT(UNTUNED),
  ALGAE_INTAKE_L3_LEFT(UNTUNED),
  ALGAE_INTAKE_L2_RIGHT(UNTUNED),
  ALGAE_INTAKE_L3_RIGHT(UNTUNED),

  ALGAE_NET_LEFT(UNTUNED),
  ALGAE_NET_RIGHT(UNTUNED),

  PROCESSOR(UNTUNED),

  // Left coral positions
  CORAL_SCORE_LEFT_LINEUP_L1(UNTUNED),
  CORAL_SCORE_LEFT_RELEASE_L1(UNTUNED),

  CORAL_SCORE_LEFT_LINEUP_L2(UNTUNED),
  CORAL_SCORE_LEFT_RELEASE_L2(UNTUNED),

  CORAL_SCORE_LEFT_LINEUP_L3(UNTUNED),
  CORAL_SCORE_LEFT_RELEASE_L3(UNTUNED),

  CORAL_SCORE_LEFT_LINEUP_L4(UNTUNED),
  CORAL_SCORE_LEFT_RELEASE_L4(UNTUNED),

  // Right coral positions
  CORAL_SCORE_RIGHT_LINEUP_L1(UNTUNED),
  CORAL_SCORE_RIGHT_RELEASE_L1(UNTUNED),

  CORAL_SCORE_RIGHT_LINEUP_L2(UNTUNED),
  CORAL_SCORE_RIGHT_RELEASE_L2(UNTUNED),

  CORAL_SCORE_RIGHT_LINEUP_L3(UNTUNED),
  CORAL_SCORE_RIGHT_RELEASE_L3(UNTUNED),

  CORAL_SCORE_RIGHT_LINEUP_L4(UNTUNED),
  CORAL_SCORE_RIGHT_RELEASE_L4(UNTUNED),

  CLIMBING(UNTUNED),
  COLLISION_AVOIDANCE(UNTUNED);

  private final double defaultHeight;
  private final DoubleSubscriber tunableHeight;

  private ElevatorState(double height) {
    this.defaultHeight = height;
    this.tunableHeight = DogLog.tunable("Elevator/State/" + name(), height);
  }

  ElevatorState(ElevatorState other) {
    this(other.defaultHeight);
  }

  public double getHeight() {
    return tunableHeight.get();
  }
}
