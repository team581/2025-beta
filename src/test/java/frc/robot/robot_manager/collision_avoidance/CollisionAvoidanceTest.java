package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.arm.ArmState;
import frc.robot.elevator.ElevatorState;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.List;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  //  @Test
  // public void routePositionTest() {
  //   SuperstructurePosition current = new SuperstructurePosition(ElevatorState.CORAL_HANDOFF,
  // ArmState.CORAL_HANDOFF);
  //   SuperstructurePosition goal = new
  // SuperstructurePosition(ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L4,
  // ArmState.CORAL_SCORE_RIGHT_LINEUP_L4);

  //   boolean climberRisky = false;
  //   double currentAngle = -90.0;
  //   ObstructionKind obstructionKind = ObstructionKind.RIGHT_OBSTRUCTED;
  //   ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
  //   ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;

  //   var result =
  //       CollisionAvoidance.routePosition(
  //           current, goal, obstructionKind, currentAngle);

  //   double expected = -360.0;

  //   assertEquals(expected, result);
  // }
  @Test
  public void armSetCollisionAvoidanceGoalTest() {
    double goalAngle = 0.0;
    boolean climberRisky = false;
    double currentAngle = -90.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -360.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalTest2() {
    double goalAngle = 180.0;
    boolean climberRisky = true;
    double currentAngle = -90.0;
    ObstructionKind obstructionKind = ObstructionKind.RIGHT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = 180.0;

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

  // @Test
  // public void leftObstructedAstarTest() {
  //   var result =
  //       CollisionAvoidance.aStar(
  //           new SuperstructurePosition(0, 90),
  //           new SuperstructurePosition(50, 180),
  //           ObstructionKind.LEFT_OBSTRUCTED);
  //   var expected =
  //       List.of(
  //           Waypoint.ELEVATOR_0_ARM_UP,
  //           Waypoint.CLIMBER_SAFE_ARM_UP,
  //           Waypoint.L4_LEFT,
  //           Waypoint.L4_LEFT_PLACE);
  //   assertEquals(expected, result.get());
  // }

  @Test
  public void rightObstructedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT),
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L4, ArmState.CORAL_SCORE_RIGHT_LINEUP_L4),
            ObstructionKind.RIGHT_OBSTRUCTED);
    // Moves elevator first, only extends arm at the end
    var expected =
        List.of(Waypoint.ELEVATOR_0_ARM_UP, Waypoint.L4_UPRIGHT, Waypoint.L4_RIGHT_LINEUP);

    assertEquals(expected, result.get());
  }

  @Test
  public void getClosestNodeStowedTest() {
    var result = Waypoint.getClosest(new SuperstructurePosition(43, -90));
    Waypoint expected = Waypoint.HANDOFF;

    assertEquals(expected, result);
  }

  @Test
  void stowsSafelyAfterRightNetScoreTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_RIGHT),
            new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF),
            ObstructionKind.NONE);
    var expected =
        List.of(
            Waypoint.ALGAE_NET_OUT_RIGHT,
            Waypoint.ALGAE_NET_UP,
            Waypoint.ELEVATOR_0_ARM_UP,
            Waypoint.HANDOFF);

    assertEquals(expected, result.orElseThrow());
  }
}
