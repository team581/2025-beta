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

  STOWED(0),

  LOLLIPOP_CORAL_INTAKE_PUSH(8.0),
  LOLLIPOP_CORAL_INTAKE_INTAKE(0.0),

  ALGAE_INTAKE_GROUND(12.0),

  ALGAE_OUTTAKE(UNTUNED),

  ALGAE_FLING(UNTUNED),

  CORAL_HANDOFF(41.12),

  ALGAE_INTAKE_L2_LEFT(24.0),
  ALGAE_INTAKE_L3_LEFT(39.0),
  ALGAE_INTAKE_L2_RIGHT(24.0),
  ALGAE_INTAKE_L3_RIGHT(39.0),

  ALGAE_NET_LEFT(56.0),
  ALGAE_NET_RIGHT(56.0),

  PROCESSOR(20.0),

  // Left coral positions
  CORAL_SCORE_LEFT_LINEUP_L2(17.0),
  CORAL_SCORE_LEFT_RELEASE_L2(17.0),

  CORAL_SCORE_LEFT_LINEUP_L3(33.0),
  CORAL_SCORE_LEFT_RELEASE_L3(33.0),

  CORAL_SCORE_LEFT_LINEUP_L4(53.0),
  CORAL_SCORE_LEFT_RELEASE_L4(53.0),

  // Right coral positions
  CORAL_SCORE_RIGHT_LINEUP_L1(0.0),
  CORAL_SCORE_RIGHT_RELEASE_L1(0.0),

  CORAL_SCORE_RIGHT_LINEUP_L2(17.0),
  CORAL_SCORE_RIGHT_RELEASE_L2(17.0),

  CORAL_SCORE_RIGHT_LINEUP_L3(33.0),
  CORAL_SCORE_RIGHT_RELEASE_L3(33.0),

  CORAL_SCORE_RIGHT_LINEUP_L4(53.0),
  CORAL_SCORE_RIGHT_RELEASE_L4(53.0),

  CLIMBING(0.0),
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
