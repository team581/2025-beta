package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.arm.ArmState;
import frc.robot.elevator.ElevatorState;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {

  private static void assertNoCollision(
      ObstructionKind obstruction, SuperstructurePosition start, SuperstructurePosition end) {
    assertPath(
        obstruction, start, end, List.of(Waypoint.getClosest(start), Waypoint.getClosest(end)));
  }

  private static void assertNoCollision(ObstructionKind obstruction, Waypoint start, Waypoint end) {
    assertPath(obstruction, start, end, List.of(start, end));
  }

  @Test
  void assertNoCollision() {
    assertNoCollision(
        ObstructionKind.NONE,
        new SuperstructurePosition(
            ElevatorState.CORAL_SCORE_RELEASE_L4, ArmState.CORAL_SCORE_RIGHT_RELEASE_L4),
        new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_RIGHT_L3));
  }

  private static void assertPath(
      ObstructionKind obstruction, Waypoint start, Waypoint end, List<Waypoint> expected) {
    assertEquals(Optional.of(expected), CollisionAvoidance.aStar(start, end, obstruction));
  }

  private static void assertPath(
      ObstructionKind obstruction,
      SuperstructurePosition start,
      Waypoint end,
      List<Waypoint> expected) {
    assertEquals(Optional.of(expected), CollisionAvoidance.aStar(start, end, obstruction));
  }

  private static void assertPath(
      ObstructionKind obstruction,
      Waypoint start,
      SuperstructurePosition end,
      List<Waypoint> expected) {
    assertEquals(Optional.of(expected), CollisionAvoidance.aStar(start, end, obstruction));
  }

  private static void assertPath(
      ObstructionKind obstruction,
      SuperstructurePosition start,
      SuperstructurePosition end,
      List<Waypoint> expected) {
    assertEquals(Optional.of(expected), CollisionAvoidance.aStar(start, end, obstruction));
  }

  @Test
  public void hectorTest() {
    var currentRawAngle = 720;
    var normalizedGoalAngle = 90;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(810.0, 450.0);
    assertEquals(expected, result);
  }

  @Test
  public void hectorTest1() {
    var currentRawAngle = -720;
    var normalizedGoalAngle = 90;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(-630.0, -990.0);
    assertEquals(expected, result);
  }

  @Test
  public void hectorTest4() {
    var normalizedGoalAngle = -90;
    var currentRawAngle = -179;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(-90.0, -450.0);
    assertEquals(expected, result);
  }

  @Test
  public void hectorTest5() {
    var normalizedGoalAngle = -90;
    var currentRawAngle = 147.7;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(270.0, -90.0);
    assertEquals(expected, result);
  }

  @Test
  public void hectorTest6() {
    var normalizedGoalAngle = -90;
    var currentRawAngle = -147.7;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(-90.0, -450.0);
    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalTest() {
    double goalAngle = -90.0;
    boolean climberRisky = false;
    double currentAngle = -211.0;
    ObstructionKind obstructionKind = ObstructionKind.NONE;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -90.0;

    assertEquals(expected, result);
  }

  @Test
  public void solutionsTest() {
    var normalizedGoalAngle = 160;
    var currentRawAngle = 269.0;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(160.0, 520.0);
    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalTestL3Lineup() {
    double goalAngle = 160.0;
    boolean climberRisky = false;
    double currentAngle = 269.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = 520.0;

    assertEquals(expected, result);
  }

  @Test
  public void positivearmSetCollisionAvoidanceGoalTest() {
    double goalAngle = -90.0;
    boolean climberRisky = false;
    double currentAngle = 140.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -90.0;

    assertEquals(expected, result);
  }

  @Test
  public void swingsaroundtwicewhenstowing() { // TODO: make this pass
    double goalAngle = -90.0;
    boolean climberRisky = false;
    double currentAngle = -147.7;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -450.0;

    assertEquals(expected, result);
  }

  @Test
  public void swingsaroundtwicewhenstowin324g() { // TODO: make this pass
    double goalAngle = -90.0;
    boolean climberRisky = false;
    double currentAngle = 147.7;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -90.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalTest3() {
    double goalAngle = 180.0;
    boolean climberRisky = false;
    double currentAngle = -90.0;
    ObstructionKind obstructionKind = ObstructionKind.RIGHT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.RIGHT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = 180.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalBackwardTest() {
    double goalAngle = -25.0;
    boolean climberRisky = true;
    double currentAngle = 0.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -25.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalGoLongWayTest() {
    double goalAngle = 200.0;
    boolean climberRisky = true;
    double currentAngle = 360.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = 560.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalGoLongWayNegativeTest() {
    double goalAngle = -200.0;
    boolean climberRisky = true;
    double currentAngle = -360.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -200.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalNegativeTest() {
    double goalAngle = 180.0;
    boolean climberRisky = true;
    double currentAngle = -360.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -180.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalPositiveTest() {
    double goalAngle = 180.0;
    boolean climberRisky = true;
    double currentAngle = 360.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = 540.0;

    assertEquals(expected, result);
  }

  @Test
  public void rightObstructedAstarTest() {
    // Moves elevator first, only extends arm at the end
    assertPath(
        ObstructionKind.RIGHT_OBSTRUCTED,
        new SuperstructurePosition(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT),
        new SuperstructurePosition(
            ElevatorState.CORAL_SCORE_LINEUP_L4, ArmState.CORAL_SCORE_RIGHT_LINEUP_L4),
        List.of(Waypoint.L1_UPRIGHT, Waypoint.L4_UPRIGHT, Waypoint.L4_RIGHT_LINEUP));
  }

  @Test
  public void getClosestNodeStowedTest() {
    var result = Waypoint.getClosest(new SuperstructurePosition(43, -90));
    Waypoint expected = Waypoint.HANDOFF;

    assertEquals(expected, result);
  }

  @Test
  void stowsSafelyAfterRightNetScoreTest() {
    assertPath(
        ObstructionKind.RIGHT_OBSTRUCTED,
        new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_RIGHT),
        new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF),
        List.of(
            Waypoint.ALGAE_NET_OUT_RIGHT,
            Waypoint.ALGAE_NET_UP,
            Waypoint.L3_UPRIGHT,
            Waypoint.HANDOFF_CLEARS_CLIMBER,
            Waypoint.HANDOFF));
  }

  @Test
  void stowsSafelyAfterLeftNetScoreTest() {
    assertPath(
        ObstructionKind.LEFT_OBSTRUCTED,
        new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_LEFT),
        new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF),
        List.of(
            Waypoint.ALGAE_NET_OUT_LEFT,
            Waypoint.ALGAE_NET_UP,
            Waypoint.L3_UPRIGHT,
            Waypoint.HANDOFF_CLEARS_CLIMBER,
            Waypoint.HANDOFF));
  }

  @Test
  void handoffToRightL2ReefIntake() {
    assertPath(
        ObstructionKind.RIGHT_OBSTRUCTED,
        new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF),
        new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_RIGHT_L2),
        List.of(Waypoint.HANDOFF, Waypoint.HANDOFF_CLEARS_CLIMBER, Waypoint.REEF_ALGAE_L2_RIGHT));
  }

  @Test
  void handoffToLeftL2ReefIntake() {
    assertPath(
        ObstructionKind.LEFT_OBSTRUCTED,
        new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF),
        new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_LEFT_L2),
        List.of(Waypoint.HANDOFF, Waypoint.HANDOFF_CLEARS_CLIMBER, Waypoint.REEF_ALGAE_L2_LEFT));
  }

  @Test
  void l4PlaceToAlgaeLeftl2() {
    assertNoCollision(
        ObstructionKind.LEFT_OBSTRUCTED,
        new SuperstructurePosition(
            ElevatorState.CORAL_SCORE_RELEASE_L4, ArmState.CORAL_SCORE_LEFT_RELEASE_L4),
        new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_LEFT_L2));
  }

  @Test
  void l4PlaceToAlgaeLeftl3() {
    assertNoCollision(
        ObstructionKind.LEFT_OBSTRUCTED,
        new SuperstructurePosition(
            ElevatorState.CORAL_SCORE_RELEASE_L4, ArmState.CORAL_SCORE_LEFT_RELEASE_L4),
        new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_LEFT_L3));
  }

  @Test
  void l4PlaceToAlgaeRightl2() {
    assertNoCollision(
        ObstructionKind.RIGHT_OBSTRUCTED,
        new SuperstructurePosition(
            ElevatorState.CORAL_SCORE_RELEASE_L4, ArmState.CORAL_SCORE_RIGHT_RELEASE_L4),
        new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_RIGHT_L2));
  }

  @Test
  void rightAlgael2ToHandoff() {
    assertPath(
        ObstructionKind.RIGHT_OBSTRUCTED,
        new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_RIGHT_L2),
        new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF),
        List.of(Waypoint.REEF_ALGAE_L2_RIGHT, Waypoint.HANDOFF_CLEARS_CLIMBER, Waypoint.HANDOFF));
  }

  @Test
  void rightAlgael3ToHandoff() {
    assertPath(
        ObstructionKind.RIGHT_OBSTRUCTED,
        new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_RIGHT_L3),
        new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF),
        List.of(Waypoint.REEF_ALGAE_L3_RIGHT, Waypoint.HANDOFF_CLEARS_CLIMBER, Waypoint.HANDOFF));
  }

  @Test
  void stowedUprightToL3RightAlgae() {
    assertNoCollision(
        ObstructionKind.LEFT_OBSTRUCTED, Waypoint.L1_UPRIGHT, Waypoint.REEF_ALGAE_L3_RIGHT);
    assertNoCollision(
        ObstructionKind.RIGHT_OBSTRUCTED, Waypoint.L1_UPRIGHT, Waypoint.REEF_ALGAE_L3_RIGHT);
    assertNoCollision(
        ObstructionKind.RIGHT_OBSTRUCTED, Waypoint.L1_UPRIGHT, Waypoint.REEF_ALGAE_L3_RIGHT);
  }

  // Test the full lollipop auto
  @Test
  void lollipopAutoTest() {
    // Lineup for preload
    assertPath(
        ObstructionKind.NONE,
        Waypoint.L1_UPRIGHT,
        Waypoint.L4_LEFT_LINEUP,
        List.of(Waypoint.L1_UPRIGHT, Waypoint.L4_UPRIGHT, Waypoint.L4_LEFT_LINEUP));
    // Score preload L4
    assertNoCollision(
        ObstructionKind.LEFT_OBSTRUCTED, Waypoint.L4_LEFT_LINEUP, Waypoint.L4_LEFT_PLACE);

    // Lollipop intake approach
    assertPath(
        ObstructionKind.LEFT_OBSTRUCTED,
        Waypoint.L4_LEFT_PLACE,
        Waypoint.LOLLIPOP_INTAKE_RIGHT_APPROACH,
        List.of(
            Waypoint.L4_LEFT_PLACE, Waypoint.L4_UPRIGHT, Waypoint.LOLLIPOP_INTAKE_RIGHT_APPROACH));

    // Lollipop intake
    assertNoCollision(
        ObstructionKind.NONE,
        Waypoint.LOLLIPOP_INTAKE_RIGHT_APPROACH,
        Waypoint.LOLLIPOP_INTAKE_RIGHT);
    assertNoCollision(
        ObstructionKind.NONE, Waypoint.LOLLIPOP_INTAKE_RIGHT, Waypoint.LOLLIPOP_INTAKE_RIGHT_PUSH);

    // Stow after intake
    assertNoCollision(
        ObstructionKind.NONE, Waypoint.LOLLIPOP_INTAKE_RIGHT_PUSH, Waypoint.PROCESSOR);
    assertNoCollision(
        ObstructionKind.NONE, Waypoint.PROCESSOR, Waypoint.L1_UPRIGHT);

    assertPath(
        ObstructionKind.LEFT_OBSTRUCTED,
        Waypoint.L1_UPRIGHT,
        Waypoint.L4_LEFT_LINEUP,
        List.of(Waypoint.L1_UPRIGHT, Waypoint.L4_UPRIGHT, Waypoint.L4_LEFT_LINEUP));
  }

  @Test
  void coralScoreLeftToReefAlgae() {
    var scoreWaypoints =
        List.of(
            Waypoint.L2_LEFT_LINEUP,
            Waypoint.L2_LEFT_PLACE,
            Waypoint.L3_LEFT_LINEUP,
            Waypoint.L3_LEFT_PLACE,
            Waypoint.L4_LEFT_LINEUP,
            Waypoint.L4_LEFT_PLACE);
    for (var start : scoreWaypoints) {
      for (var obstruction : ObstructionKind.values()) {
        assertNoCollision(obstruction, start, Waypoint.REEF_ALGAE_L2_LEFT);
        assertNoCollision(obstruction, start, Waypoint.REEF_ALGAE_L3_LEFT);
      }
    }
  }

  @Test
  void coralScoreRightToReefAlgae() {
    var scoreWaypoints =
        List.of(
            Waypoint.L1_RIGHT_LINEUP,
            Waypoint.L2_RIGHT_LINEUP,
            Waypoint.L2_RIGHT_PLACE,
            Waypoint.L3_RIGHT_LINEUP,
            Waypoint.L3_RIGHT_PLACE,
            Waypoint.L4_RIGHT_LINEUP,
            Waypoint.L4_RIGHT_PLACE);
    for (var start : scoreWaypoints) {
      for (var obstruction : ObstructionKind.values()) {
        assertNoCollision(obstruction, start, Waypoint.REEF_ALGAE_L2_RIGHT);
        assertNoCollision(obstruction, start, Waypoint.REEF_ALGAE_L3_RIGHT);
      }
    }
  }

  @Test
  void handoffToScoreUnobstructed() {
    assertNoCollision(ObstructionKind.NONE, Waypoint.HANDOFF_CLEARS_CLIMBER, Waypoint.L2_LEFT_LINEUP);
    assertNoCollision(ObstructionKind.NONE, Waypoint.HANDOFF_CLEARS_CLIMBER, Waypoint.L3_LEFT_LINEUP);
    assertNoCollision(ObstructionKind.NONE, Waypoint.HANDOFF_CLEARS_CLIMBER, Waypoint.L4_LEFT_LINEUP);

    // TODO: Figure out optimal L1 scoring motion
    // assertNoCollision(ObstructionKind.NONE, Waypoint.HANDOFF, Waypoint.L1_RIGHT_LINEUP);
    assertNoCollision(ObstructionKind.NONE, Waypoint.HANDOFF_CLEARS_CLIMBER, Waypoint.L2_RIGHT_LINEUP);
    assertNoCollision(ObstructionKind.NONE, Waypoint.HANDOFF_CLEARS_CLIMBER, Waypoint.L3_RIGHT_LINEUP);
    assertNoCollision(ObstructionKind.NONE, Waypoint.HANDOFF_CLEARS_CLIMBER, Waypoint.L4_RIGHT_LINEUP);
  }
}
