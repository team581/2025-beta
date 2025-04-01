package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.MutableValueGraph;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.arm.ArmState;
import frc.robot.elevator.ElevatorState;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.stream.Stream;

/**
 * These represent "waypoints" for collision avoidance to route through. These are NOT setpoints
 * that the robot uses, even though they may share a name and/or superstructure position. Collision
 * avoidance uses these as nodes within a graph to route from a current position to a goal position.
 */
public enum Waypoint {
  // TODO Here's an example of using setpoints for waypoints. Don't do this 100% of the time! Only
  // where it makes sense - Jonah
  ALGAE_INTAKE_RIGHT(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_GROUND, ArmState.ALGAE_INTAKE_FLOOR)),
  LOLLIPOP_INTAKE_RIGHT(new SuperstructurePosition(0.0, 0.0)),
  STOWED(new SuperstructurePosition(55.0, -90.0)),
  LEFT_SAFE_STOWED_UP(new SuperstructurePosition(10.0, 90.0)),
  STOWED_UP(new SuperstructurePosition(0.0, 90.0)),
  HANDOFF(new SuperstructurePosition(41.12, -90.0)),
  L1_RIGHT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L1, ArmState.CORAL_SCORE_RIGHT_LINEUP_L1)),
  L2_RIGHT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L2, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2)),
  L3_RIGHT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L3, ArmState.CORAL_SCORE_RIGHT_LINEUP_L3)),
  L4_RIGHT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L4, ArmState.CORAL_SCORE_RIGHT_LINEUP_L4)),
  L2_LEFT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LEFT_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2)),
  L3_LEFT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LEFT_LINEUP_L3, ArmState.CORAL_SCORE_LEFT_LINEUP_L3)),
  L4_LEFT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LEFT_LINEUP_L4, ArmState.CORAL_SCORE_LEFT_LINEUP_L4)),
          L1_RIGHT_RELEASE(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L1, ArmState.CORAL_SCORE_RIGHT_RELEASE_L1)),
        L2_RIGHT_RELEASE(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L2, ArmState.CORAL_SCORE_RIGHT_RELEASE_L2)),
        L3_RIGHT_RELEASE(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L3, ArmState.CORAL_SCORE_RIGHT_RELEASE_L3)),
        L4_RIGHT_RELEASE(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L4, ArmState.CORAL_SCORE_RIGHT_RELEASE_L4)),
        L2_LEFT_RELEASE(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L2, ArmState.CORAL_SCORE_RIGHT_RELEASE_L2)),
        L3_LEFT_RELEASE(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L3, ArmState.CORAL_SCORE_RIGHT_RELEASE_L3)),
        L4_LEFT_RELEASE(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L4, ArmState.CORAL_SCORE_RIGHT_RELEASE_L4)),
  ALGAE_RIGHT(new SuperstructurePosition(ElevatorState.ALGAE_NET_RIGHT, ArmState.ALGAE_NET_RIGHT)),
  ALGAE_LEFT(new SuperstructurePosition(ElevatorState.ALGAE_NET_LEFT, ArmState.ALGAE_NET_LEFT)),
  ALGAE_L2_RIGHT(
      new SuperstructurePosition(
          ElevatorState.ALGAE_INTAKE_L2_RIGHT, ArmState.ALGAE_INTAKE_RIGHT_L2)),
  ALGAE_L2_LEFT(
      new SuperstructurePosition(
          ElevatorState.ALGAE_INTAKE_L2_LEFT, ArmState.ALGAE_INTAKE_LEFT_L2)),
  ALGAE_L3_RIGHT(
      new SuperstructurePosition(
          ElevatorState.ALGAE_INTAKE_L3_RIGHT, ArmState.ALGAE_INTAKE_RIGHT_L3)),
  ALGAE_L3_LEFT(
      new SuperstructurePosition(
          ElevatorState.ALGAE_INTAKE_L3_LEFT, ArmState.ALGAE_INTAKE_LEFT_L3));

  public final SuperstructurePosition position;

  Waypoint(SuperstructurePosition position) {
    this.position = position;
  }

  public double costFor(Waypoint other) {
    return position.costFor(other.position);
  }

  public static void log() {
    for (var waypoint : values()) {
      DogLog.log(
          "CollisionAvoidance/Waypoints/" + waypoint.toString(),
          waypoint.position.getTranslation());
    }
    DogLog.log(
        "CollisionAvoidance/Waypoints/All",
        Stream.of(values())
            .map(waypoint -> waypoint.position.getTranslation())
            .toArray(Translation2d[]::new));
  }

  /**
   * Find the closest waypoint to the given superstructure position.
   *
   * @param position The position of the superstructure.
   */
  public static Waypoint getClosest(SuperstructurePosition position) {
    Waypoint closestWaypoint = STOWED;
    double closestDistance = Double.MAX_VALUE;
    Translation2d point = position.getTranslation();

    for (int i = 0; Waypoint.values().length > i; i++) {

      Translation2d nodePoint = Waypoint.values()[i].position.getTranslation();

      double distanceFromNodeToPoint =
          Math.hypot(
              Math.pow(nodePoint.getX() - point.getX(), 2),
              Math.pow(nodePoint.getY() - point.getY(), 2));

      if (distanceFromNodeToPoint < closestDistance) {
        closestDistance = distanceFromNodeToPoint;
        closestWaypoint = Waypoint.values()[i];
      }
    }
    return closestWaypoint;
  }

  public void canMoveToAlways(Waypoint other, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    canMoveToAlways(other, true, graph);
  }

  public void canMoveToAlways(
      Waypoint other, boolean climberAtRisk, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    var existingEdge = graph.putEdgeValue(this, other, WaypointEdge.alwaysSafe(this, other));

    if (existingEdge != null) {
      throw new IllegalStateException("Redundant edge connecting " + this + " to " + other);
    }
  }

  public void canMoveToWhenLeftSafe(
      Waypoint other, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    canMoveToWhenLeftSafe(other, true, graph);
  }

  public void canMoveToWhenLeftSafe(
      Waypoint other, boolean climberAtRisk, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    var existingEdge = graph.putEdgeValue(this, other, WaypointEdge.leftUnblocked(this, other));

    if (existingEdge != null) {
      throw new IllegalStateException("Redundant edge connecting " + this + " to " + other);
    }
  }

  public void canMoveToWhenRightSafe(
      Waypoint other, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    canMoveToWhenRightSafe(other, true, graph);
  }

  public void canMoveToWhenRightSafe(
      Waypoint other, boolean climberAtRisk, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    var existingEdge = graph.putEdgeValue(this, other, WaypointEdge.rightUnblocked(this, other));

    if (existingEdge != null) {
      throw new IllegalStateException("Redundant edge connecting " + this + " to " + other);
    }
  }
}
