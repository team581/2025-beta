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

  GROUND_CORAL_INTAKE_UPRIGHT(0.0),
  GROUND_ALGAE_INTAKE(10.95),

  ALGAE_OUTTAKE(UNTUNED),

  CORAL_HANDOFF(42.12),

  ALGAE_INTAKE_L2_LEFT(24.0),
  ALGAE_INTAKE_L3_LEFT(39.0),
  ALGAE_INTAKE_L2_RIGHT(24.0),
  ALGAE_INTAKE_L3_RIGHT(39.0),

  ALGAE_NET_LEFT(UNTUNED),
  ALGAE_NET_RIGHT(UNTUNED),

  PROCESSOR(UNTUNED),

  // Left coral positions
  CORAL_SCORE_LEFT_LINEUP_L2(7.2),
  CORAL_SCORE_LEFT_RELEASE_L2(4.5),

  CORAL_SCORE_LEFT_LINEUP_L3(22.0),
  CORAL_SCORE_LEFT_RELEASE_L3(19.0),

  CORAL_SCORE_LEFT_LINEUP_L4(46.5),
  CORAL_SCORE_LEFT_RELEASE_L4(50.9),

  // Right coral positions
  CORAL_SCORE_RIGHT_LINEUP_L1(0.0),
  CORAL_SCORE_RIGHT_RELEASE_L1(0.0),

  CORAL_SCORE_RIGHT_LINEUP_L2(7.2),
  CORAL_SCORE_RIGHT_RELEASE_L2(4.5),

  CORAL_SCORE_RIGHT_LINEUP_L3(22.0),
  CORAL_SCORE_RIGHT_RELEASE_L3(19.0),

  CORAL_SCORE_RIGHT_LINEUP_L4(46.5),
  CORAL_SCORE_RIGHT_RELEASE_L4(50.9),

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
