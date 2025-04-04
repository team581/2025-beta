package frc.robot.robot_manager.collision_avoidance;

import java.util.Optional;

public record WaypointEdge(
    /** The cost associated with the motion connecting the nodes on this edge. */
    double cost,
    /** Whether this motion is safe when the left side of the robot is obstructed. */
    boolean safeWhenLeftBlocked,
    /** Whether this motion is safe when the right side of the robot is obstructed. */
    boolean safeWhenRightBlocked,
    /** Whether this motion will pass through the climber area. */
    boolean climberAtRisk,
    /**
     * The constraints for superstructure motion to use when moving along this edge. If empty,
     * regular motion magic parameters are used.
     */
    Optional<SuperstructureLimits> limits) {

  public static WaypointEdge rightUnblocked(Waypoint from, Waypoint to) {
    return new WaypointEdge(from, to, true, false, true, Optional.empty());
  }

  public static WaypointEdge rightUnblocked(
      Waypoint from, Waypoint to, SuperstructureLimits limits) {
    return new WaypointEdge(from, to, true, false, true, Optional.of(limits));
  }

  public static WaypointEdge rightUnblocked(Waypoint from, Waypoint to, boolean climberAtRisk) {
    return new WaypointEdge(from, to, true, false, climberAtRisk, Optional.empty());
  }

  public static WaypointEdge rightUnblocked(
      Waypoint from, Waypoint to, SuperstructureLimits limits, boolean climberAtRisk) {
    return new WaypointEdge(from, to, true, false, climberAtRisk, Optional.of(limits));
  }

  public static WaypointEdge leftUnblocked(Waypoint from, Waypoint to) {
    return new WaypointEdge(from, to, false, true, true, Optional.empty());
  }

  public static WaypointEdge leftUnblocked(
      Waypoint from, Waypoint to, SuperstructureLimits limits) {
    return new WaypointEdge(from, to, false, true, true, Optional.of(limits));
  }

  public static WaypointEdge leftUnblocked(Waypoint from, Waypoint to, boolean climberAtRisk) {
    return new WaypointEdge(from, to, false, true, climberAtRisk, Optional.empty());
  }

  public static WaypointEdge leftUnblocked(
      Waypoint from, Waypoint to, SuperstructureLimits limits, boolean climberAtRisk) {
    return new WaypointEdge(from, to, false, true, climberAtRisk, Optional.of(limits));
  }

  public static WaypointEdge alwaysSafe(Waypoint from, Waypoint to) {
    return new WaypointEdge(from, to, true, true, true, Optional.empty());
  }

  public static WaypointEdge alwaysSafe(Waypoint from, Waypoint to, SuperstructureLimits limits) {
    return new WaypointEdge(from, to, true, true, true, Optional.of(limits));
  }

  public static WaypointEdge alwaysSafe(Waypoint from, Waypoint to, boolean climberAtRisk) {
    return new WaypointEdge(from, to, true, true, climberAtRisk, Optional.empty());
  }

  public static WaypointEdge alwaysSafe(
      Waypoint from, Waypoint to, SuperstructureLimits limits, boolean climberAtRisk) {
    return new WaypointEdge(from, to, true, true, climberAtRisk, Optional.of(limits));
  }

  private WaypointEdge(
      Waypoint from,
      Waypoint to,
      boolean safeForLeftBlocked,
      boolean safeForRightBlocked,
      boolean climberAtRisk,
      Optional<SuperstructureLimits> limits) {
    this(from.costFor(to), safeForLeftBlocked, safeForRightBlocked, climberAtRisk, limits);
  }

  public double getCost(ObstructionKind obstruction) {
    return cost;
  }
}
