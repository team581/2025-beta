package frc.robot.robot_manager.collision_avoidance;

import java.util.Optional;

public record WaypointEdge(
    /** The cost associated with the motion connecting the nodes on this edge. */
    double cost,
    boolean hitsClimber,
    /** Whether this motion is safe when the left side of the robot is obstructed. */
    ObstructionStrategy leftSideStrategy,
    /** Whether this motion is safe when the right side of the robot is obstructed. */
    ObstructionStrategy rightSideStrategy) {
  public WaypointEdge(
      Waypoint from,
      Waypoint to,
      ObstructionStrategy leftSideStrategy,
      ObstructionStrategy rightSideStrategy) {
    this(from.costFor(to), false, leftSideStrategy, rightSideStrategy);
  }

  public WaypointEdge avoidClimber() {
    return new WaypointEdge(cost, true, leftSideStrategy, rightSideStrategy);
  }

  public double getCost(ObstructionKind obstruction) {
    return switch (obstruction) {
      // TODO: The cost of the long way != the short way, need to account for that
      case LEFT_OBSTRUCTED -> leftSideStrategy == ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED ? Double.MAX_VALUE : cost;
      // TODO: The cost of the long way != the short way, need to account for that
      case RIGHT_OBSTRUCTED -> rightSideStrategy == ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED ? Double.MAX_VALUE : cost;
      case NONE -> cost;
    };
  }
}
