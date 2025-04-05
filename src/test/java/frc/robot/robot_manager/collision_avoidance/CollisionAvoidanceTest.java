package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.robot_manager.SuperstructurePosition;
import java.util.List;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  // TODO: add good tests

  @Test
  public void aStarTest() {
    var current = new SuperstructurePosition(0, 90);
    var goal = new SuperstructurePosition(50, -90);

    ObstructionKind obstructionKind = ObstructionKind.NONE;
    var result = CollisionAvoidance.aStar(current, goal, obstructionKind);

    var expected = List.of(Waypoint.ELEVATOR_0_ARM_UP, Waypoint.L4_RIGHT, Waypoint.HANDOFF_BUT_HIGHER);

    assertEquals(expected, result.get());
  }

  @Test
  public void armSetCollisionAvoidanceGoalTest() {
    double goalAngle = 0.0;
    boolean climberRisky = false;
    double currentAngle = -90.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, edgeObstructionKind, currentAngle);

    double expected = -360.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalTest2() {
    double goalAngle = 180.0;
    boolean climberRisky = false;
    double currentAngle = -90.0;
    ObstructionKind obstructionKind = ObstructionKind.RIGHT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, edgeObstructionKind, currentAngle);

    double expected = -180.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalTest3() {
    double goalAngle = 180.0;
    boolean climberRisky = false;
    double currentAngle = -90.0;
    ObstructionKind obstructionKind = ObstructionKind.RIGHT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.RIGHT_OBSTRUCTED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, edgeObstructionKind, currentAngle);

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
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, edgeObstructionKind, currentAngle);

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
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, edgeObstructionKind, currentAngle);

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
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, edgeObstructionKind, currentAngle);

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
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, edgeObstructionKind, currentAngle);

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
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, edgeObstructionKind, currentAngle);

    double expected = 540.0;

    assertEquals(expected, result);
  }

  @Test
  public void leftObstructedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(50, 180),
            ObstructionKind.LEFT_OBSTRUCTED);
    var expected =
        List.of(
            Waypoint.ELEVATOR_0_ARM_UP,
            Waypoint.CLIMBER_SAFE_ARM_UP,
            Waypoint.L4_LEFT,
            Waypoint.L4_LEFT_PLACE);
    assertEquals(expected, result.get());
  }

  @Test
  public void rightObstructedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(50, 0),
            ObstructionKind.RIGHT_OBSTRUCTED);
    var expected = List.of(Waypoint.ELEVATOR_0_ARM_UP, Waypoint.L4_RIGHT, Waypoint.L4_RIGHT_PLACE);

    assertEquals(expected, result.get());
  }

  @Test
  public void getClosestNodeStowedTest() {
    var result = Waypoint.getClosest(new SuperstructurePosition(43, -90));
    Waypoint expected = Waypoint.HANDOFF;

    assertEquals(expected, result);
  }
}
