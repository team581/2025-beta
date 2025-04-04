package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.arm.ArmSubsystem;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.List;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {

  @Test
  public void armSetCollisionAvoidanceGoalTest() {
    double goalAngle = 25.0;
    boolean climberRisky = true;
    double currentAngle = 0.0;
    double result = ArmSubsystem.getCollisionAvoidanceGoal(goalAngle, climberRisky, currentAngle);

    double expected = 25.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalBackwardTest() {
    double goalAngle = -25.0;
    boolean climberRisky = true;
    double currentAngle = 0.0;
    double result = ArmSubsystem.getCollisionAvoidanceGoal(goalAngle, climberRisky, currentAngle);

    double expected = -25.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalGoLongWayTest() {
    double goalAngle = 200.0;
    boolean climberRisky = true;
    double currentAngle = 360.0;
    double result = ArmSubsystem.getCollisionAvoidanceGoal(goalAngle, climberRisky, currentAngle);

    double expected = 560.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalGoLongWayNegativeTest() {
    double goalAngle = -200.0;
    boolean climberRisky = true;
    double currentAngle = -360.0;
    double result = ArmSubsystem.getCollisionAvoidanceGoal(goalAngle, climberRisky, currentAngle);

    double expected = -200.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalNegativeTest() {
    double goalAngle = 180.0;
    boolean climberRisky = true;
    double currentAngle = -360.0;
    double result = ArmSubsystem.getCollisionAvoidanceGoal(goalAngle, climberRisky, currentAngle);

    double expected = -180.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalPositiveTest() {
    double goalAngle = 180.0;
    boolean climberRisky = true;
    double currentAngle = 360.0;
    double result = ArmSubsystem.getCollisionAvoidanceGoal(goalAngle, climberRisky, currentAngle);

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
