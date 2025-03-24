package frc.robot.robot_manager.collision_avoidance;

public record WaypointEdge(
    /** The cost associated with the motion connecting the nodes on this edge. */
    double cost,
    /** Whether this motion is safe when the left side of the robot is obstructed. */
    boolean safeWhenLeftBlocked,
    /** Whether this motion is safe when the right side of the robot is obstructed. */
    boolean safeWhenRightBlocked) {

  public static WaypointEdge rightUnblocked(Waypoint from, Waypoint to) {
    return new WaypointEdge(
        from, to, /* safeForLeftBlocked= */ true, /* safeForRightBlocked= */ false);
  }

  public static WaypointEdge leftUnblocked(Waypoint from, Waypoint to) {
    return new WaypointEdge(
        from, to, /* safeForLeftBlocked= */ false, /* safeForRightBlocked= */ true);
  }

  public static WaypointEdge alwaysSafe(Waypoint from, Waypoint to) {
    return new WaypointEdge(
        from, to, /* safeForLeftBlocked= */ true, /* safeForRightBlocked= */ true);
  }

  private WaypointEdge(
      Waypoint from, Waypoint to, boolean safeForLeftBlocked, boolean safeForRightBlocked) {
    this(from.costFor(to), safeForLeftBlocked, safeForRightBlocked);
  }

  public double getCost(ObstructionKind obstruction) {
    return switch (obstruction) {
      case LEFT_OBSTRUCTED -> safeWhenLeftBlocked ? cost : Double.MAX_VALUE;
      case RIGHT_OBSTRUCTED -> safeWhenRightBlocked ? cost : Double.MAX_VALUE;
      case NONE -> cost;
    };
  }
}
