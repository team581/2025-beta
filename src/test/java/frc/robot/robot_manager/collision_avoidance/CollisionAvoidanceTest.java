package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  // @Test
  // public void routeStowedUpToUpRightAstarTest() {
  //   var result =
  //       CollisionAvoidance.route(
  //           new SuperstructurePosition(0, 90),
  //           new SuperstructurePosition(50, 0),
  //           ObstructionKind.NONE);
  //   var expected = Waypoint.STOWED_UP;

  //   assertEquals(expected, result);
  // }
  // @Test
  // public void stowedUpToUpRightAstarTest() {
  //   var result =
  //       CollisionAvoidance.aStar(
  //           new SuperstructurePosition(0, 90),
  //           new SuperstructurePosition(50, 0),
  //           ObstructionKind.NONE);
  //   var expected = List.of(Waypoint.STOWED_UP, Waypoint.L4_RIGHT);

  //   assertEquals(expected, new ArrayList<>(result.orElseThrow()));
  // }

  @Test
  public void lowRightToStowedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 0),
            new SuperstructurePosition(50, -90),
            ObstructionKind.NONE);

    var expected = List.of(Waypoint.LOLLIPOP_INTAKE_RIGHT, Waypoint.L4_RIGHT, Waypoint.STOWED);

    assertEquals(expected, new ArrayList<>(result.orElseThrow()));
  }

  @Test
  public void leftObstructedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(50, 180),
            ObstructionKind.LEFT_OBSTRUCTED);

    assertEquals(Optional.empty(), result);
  }

  @Test
  public void rightObstructedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90),
            new SuperstructurePosition(50, 0),
            ObstructionKind.RIGHT_OBSTRUCTED);

    assertEquals(Optional.empty(), result);
  }

  @Test
  public void getClosestNodeStowedTest() {
    var result = Waypoint.getClosest(new SuperstructurePosition(43, -90));
    Waypoint expected = Waypoint.HANDOFF;

    assertEquals(expected, result);
  }
}
