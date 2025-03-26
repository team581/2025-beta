package frc.robot.arm;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ArmState {
  /**
   * @deprecated This is a placeholder state.
   */
  @Deprecated
  UNTUNED(0),

  UNJAM(UNTUNED),
  // Idle positions
  HOLDING_UPRIGHT(UNTUNED),

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

  // Handoffs
  CORAL_HANDOFF(UNTUNED),

  // Algae positions
  ALGAE_INTAKE_FLOOR(UNTUNED),

  ALGAE_INTAKE_LEFT_L2(UNTUNED),
  ALGAE_INTAKE_LEFT_L3(UNTUNED),

  ALGAE_INTAKE_RIGHT_L2(UNTUNED),
  ALGAE_INTAKE_RIGHT_L3(UNTUNED),

  ALGAE_NET_LEFT(UNTUNED),

  ALGAE_NET_RIGHT(UNTUNED),

  ALGAE_PROCESSOR(UNTUNED),

  ALGAE_OUTTAKE(UNTUNED),
  MULTICORAL_OUTTAKE(ALGAE_OUTTAKE),

  COLLISION_AVOIDANCE(UNTUNED),
  CLIMBING(UNTUNED),

  // For auto
  LOLLIPOP_CORAL_INTAKE(UNTUNED);

  private final double defaultAngle;
  private final DoubleSubscriber tunableAngle;

  ArmState(double angle) {
    this.defaultAngle = angle;
    this.tunableAngle = DogLog.tunable("Arm/State/" + name(), angle);
  }

  ArmState(ArmState other) {
    this(other.defaultAngle);
  }

  public double getAngle() {
    return tunableAngle.get();
  }
}
