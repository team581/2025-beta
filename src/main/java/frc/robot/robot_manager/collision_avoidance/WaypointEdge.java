package frc.robot.robot_manager.collision_avoidance;

public record WaypointEdge(
    /** The cost associated with the motion connecting the nodes on this edge. */
    double cost,
    double costForLongWay,
    boolean hitsClimber,
    /** Whether this motion is safe when the left side of the robot is obstructed. */
    ObstructionStrategy leftSideStrategy,
    /** Whether this motion is safe when the right side of the robot is obstructed. */
    ObstructionStrategy rightSideStrategy) {
  private static final double STATIC_COST = 2.0;

  public WaypointEdge(
      Waypoint from,
      Waypoint to,
      ObstructionStrategy leftSideStrategy,
      ObstructionStrategy rightSideStrategy) {
    this(
        from.costFor(to),
        from.costFor(to) + STATIC_COST,
        false,
        leftSideStrategy,
        rightSideStrategy);
  }

  public WaypointEdge avoidClimber() {
    return new WaypointEdge(cost, costForLongWay, true, leftSideStrategy, rightSideStrategy);
  }

  public double getCost(ObstructionKind obstruction) {
    return switch (obstruction) {
      case LEFT_OBSTRUCTED ->
          switch (leftSideStrategy) {
            case IMPOSSIBLE_IF_BLOCKED -> Double.MAX_VALUE;
            case IGNORE_BLOCKED -> cost;
            case LONG_WAY_IF_BLOCKED -> costForLongWay;
          };
      case RIGHT_OBSTRUCTED ->
          switch (rightSideStrategy) {
            case IMPOSSIBLE_IF_BLOCKED -> Double.MAX_VALUE;
            case IGNORE_BLOCKED -> cost;
            case LONG_WAY_IF_BLOCKED -> costForLongWay;
          };
      case NONE -> cost;
    };
  }
}
