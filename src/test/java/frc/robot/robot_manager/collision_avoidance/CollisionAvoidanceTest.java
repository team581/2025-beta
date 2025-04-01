package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.robot_manager.SuperstructurePosition;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  public void alreadyThereAstar() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(0, 90),
            ObstructionKind.NONE);

    assertEquals(Optional.empty(), result);
  }
  @Test
  public void rightObstructedL4toL1() {
    var result =
        CollisionAvoidance.aStar(
            Waypoint.L4_RIGHT.position,
            Waypoint.L1_RIGHT.position,
            ObstructionKind.RIGHT_OBSTRUCTED);
var expected = List.of(Waypoint.L4_RIGHT, Waypoint.L4_LEFT, Waypoint.LEFT_SAFE_STOWED_UP, Waypoint.STOWED_UP, Waypoint.L1_RIGHT);
    assertEquals(expected, result.get());
  }

  @Test
  public void stowedUpToUpRightAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(50, 0),
            ObstructionKind.NONE);
    var expected = List.of(Waypoint.STOWED_UP, Waypoint.L3_RIGHT);

    assertEquals(expected, result.get());
  }

  @Test
  public void lowRightToStowedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 0),
            new SuperstructurePosition(50, -90),
            ObstructionKind.NONE);

    var expected = List.of(Waypoint.LOLLIPOP_INTAKE_RIGHT, Waypoint.L4_RIGHT, Waypoint.STOWED);

    assertEquals(expected, result.get());
  }

  @Test
  public void leftObstructedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(50, 180),
            ObstructionKind.LEFT_OBSTRUCTED);
    var expected = List.of(Waypoint.STOWED_UP, Waypoint.L3_RIGHT, Waypoint.L3_LEFT);

    assertEquals(expected, result.get());
  }

  @Test
  public void rightObstructedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(50, 0),
            ObstructionKind.RIGHT_OBSTRUCTED);
    var expected =
        List.of(
            Waypoint.STOWED_UP, Waypoint.LEFT_SAFE_STOWED_UP, Waypoint.L3_LEFT, Waypoint.L3_RIGHT);

    assertEquals(expected, result.get());
  }

  @Test
  public void leftObstructedAlgaeRightAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(60, 45),
            ObstructionKind.LEFT_OBSTRUCTED);
    var expected = List.of(Waypoint.STOWED_UP, Waypoint.ALGAE_RIGHT);

    assertEquals(expected, result.get());
  }

  @Test
  public void rightObstructedAlgaeLeftAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(60, 45),
            ObstructionKind.RIGHT_OBSTRUCTED);
    var expected =
        List.of(
            Waypoint.STOWED_UP,
            Waypoint.LEFT_SAFE_STOWED_UP,
            Waypoint.ALGAE_LEFT,
            Waypoint.ALGAE_RIGHT);

    assertEquals(expected, result.get());
  }

  @Test
  public void rightObstructedHandoffToL4RightAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(40, -90),
            new SuperstructurePosition(50, 0),
            ObstructionKind.RIGHT_OBSTRUCTED);
    var expected =
        List.of(
            Waypoint.HANDOFF,
            Waypoint.STOWED,
            Waypoint.L4_LEFT,
            Waypoint.L3_LEFT,
            Waypoint.L3_RIGHT);

    assertEquals(expected, result.get());
  }

  @Test
  public void getClosestNodeStowedTest() {
    var result = Waypoint.getClosest(new SuperstructurePosition(43, -90));
    Waypoint expected = Waypoint.HANDOFF;

    assertEquals(expected, result);
  }
}
