package frc.robot.robot_manager.collision_avoidance;

public record WaypointEdge(
    double shortCost,
    double longCost,
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
    this(from.costFor(to), from.costForLongWay(to), false, leftSideStrategy, rightSideStrategy);
  }

  public WaypointEdge avoidClimber() {
    return new WaypointEdge(shortCost, longCost, true, leftSideStrategy, rightSideStrategy);
  }

  public double getCost(ObstructionKind obstruction) {
    return switch (obstruction) {
      case LEFT_OBSTRUCTED ->
          switch (leftSideStrategy) {
            case IMPOSSIBLE_IF_BLOCKED -> Double.MAX_VALUE;
            case IGNORE_BLOCKED -> shortCost;
            case LONG_WAY_IF_BLOCKED -> longCost;
          };
      case RIGHT_OBSTRUCTED ->
          switch (rightSideStrategy) {
            case IMPOSSIBLE_IF_BLOCKED -> Double.MAX_VALUE;
            case IGNORE_BLOCKED -> shortCost;
            case LONG_WAY_IF_BLOCKED -> longCost;
          };
      case NONE -> shortCost;
    };
  }
}
