package frc.robot.arm;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ArmState {
  /**
   * @deprecated This is a placeholder state.
   */
  @Deprecated
  UNTUNED(90),

  PRE_MATCH_HOMING(0),

  UNJAM(UNTUNED),
  // Idle positions
  HOLDING_UPRIGHT(90),

  // Left coral positions
  CORAL_SCORE_LEFT_LINEUP_L2(180 - 25.0),
  CORAL_SCORE_LEFT_RELEASE_L2(180 - 5.0),

  CORAL_SCORE_LEFT_LINEUP_L3(180 - 25.0),
  CORAL_SCORE_LEFT_RELEASE_L3(180 - 5.0),

  CORAL_SCORE_LEFT_LINEUP_L4(180 - 32.0),
  CORAL_SCORE_LEFT_RELEASE_L4(180.0 - 5.0),

  // Right coral positions
  CORAL_SCORE_RIGHT_LINEUP_L1(35.0),
  CORAL_SCORE_RIGHT_RELEASE_L1(35.0),

  CORAL_SCORE_RIGHT_LINEUP_L2(25.0),
  CORAL_SCORE_RIGHT_RELEASE_L2(5.0),

  CORAL_SCORE_RIGHT_LINEUP_L3(25.0),
  CORAL_SCORE_RIGHT_RELEASE_L3(5.0),

  CORAL_SCORE_RIGHT_LINEUP_L4(32.0),
  CORAL_SCORE_RIGHT_RELEASE_L4(5.0),

  // Handoffs
  CORAL_HANDOFF(-90.0),

  // Algae positions
  ALGAE_INTAKE_FLOOR(-21.18),

  ALGAE_INTAKE_LEFT_L2(180.0),
  ALGAE_INTAKE_LEFT_L3(180.0),

  ALGAE_INTAKE_RIGHT_L2(0.0),
  ALGAE_INTAKE_RIGHT_L3(0.0),

  ALGAE_NET_LEFT(180 - 75.0),

  ALGAE_NET_RIGHT(75.0),

  ALGAE_PROCESSOR(0.0),

  ALGAE_OUTTAKE(UNTUNED),

  COLLISION_AVOIDANCE(UNTUNED),
  CLIMBING(UNTUNED),

  SPIN_TO_WIN(90),

  ALGAE_FLING_WAIT(UNTUNED),
  ALGAE_FLING_SWING(UNTUNED),

  // For auto
  LOLLIPOP_CORAL_INTAKE_APPROACH(20),
  LOLLIPOP_CORAL_INTAKE_INTAKE(-8),
  LOLLIPOP_CORAL_INTAKE_TILT(-15);

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
