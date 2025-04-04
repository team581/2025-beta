package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.MutableValueGraph;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.arm.ArmState;
import frc.robot.elevator.ElevatorState;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;

/**
 * These represent "waypoints" for collision avoidance to route through. These are NOT setpoints
 * that the robot uses, even though they may share a name and/or superstructure position. Collision
 * avoidance uses these as nodes within a graph to route from a current position to a goal position.
 */
public enum Waypoint {
  ALGAE_INTAKE_RIGHT(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_GROUND, ArmState.ALGAE_INTAKE_FLOOR)),
  LOLLIPOP_INTAKE_RIGHT(
      new SuperstructurePosition(
          ElevatorState.LOLLIPOP_CORAL_INTAKE_INTAKE, ArmState.LOLLIPOP_CORAL_INTAKE_INTAKE)),
  STOWED(new SuperstructurePosition(55.0, ArmState.CORAL_HANDOFF)),
  LEFT_SAFE_STOWED_UP(new SuperstructurePosition(10.0, ArmState.HOLDING_UPRIGHT)),
  STOWED_UP(new SuperstructurePosition(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT)),
  HANDOFF(new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF)),
  L1_RIGHT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L1, ArmState.CORAL_SCORE_RIGHT_LINEUP_L1)),
  L2_RIGHT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L2, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2)),
  L2_RIGHT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L2, ArmState.CORAL_SCORE_RIGHT_RELEASE_L2)),
  L3_RIGHT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L3, ArmState.CORAL_SCORE_RIGHT_LINEUP_L3)),
  L3_RIGHT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L3, ArmState.CORAL_SCORE_RIGHT_RELEASE_L3)),
  L4_RIGHT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L4, ArmState.CORAL_SCORE_RIGHT_LINEUP_L4)),
  L4_RIGHT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L4, ArmState.CORAL_SCORE_RIGHT_RELEASE_L4)),
  L2_LEFT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LEFT_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2)),
  L2_LEFT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LEFT_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2)),
  L3_LEFT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LEFT_LINEUP_L3, ArmState.CORAL_SCORE_LEFT_LINEUP_L3)),
  L3_LEFT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LEFT_RELEASE_L3, ArmState.CORAL_SCORE_LEFT_RELEASE_L3)),
  L4_LEFT(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LEFT_LINEUP_L4, ArmState.CORAL_SCORE_LEFT_LINEUP_L4)),
  L4_LEFT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LEFT_RELEASE_L4, ArmState.CORAL_SCORE_LEFT_RELEASE_L4)),
  ALGAE_NET_LEFT_UP(new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.HOLDING_UPRIGHT)),
  ALGAE_NET_LEFT_OUT(new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_LEFT)),
  ALGAE_NET_RIGHT_SWING_1(new SuperstructurePosition(ElevatorState.ALGAE_NET, 230.0)),
  ALGAE_NET_RIGHT_SWING_2(new SuperstructurePosition(ElevatorState.ALGAE_NET, 180.0)),
  ALGAE_NET_RIGHT_UP(new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.HOLDING_UPRIGHT)),
  ALGAE_NET_RIGHT_OUT(
      new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_RIGHT)),
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

  private static final List<Waypoint> ALL_WAYPOINTS = List.of(values());

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
    return ALL_WAYPOINTS.stream()
        .min(Comparator.comparingDouble(waypoint -> position.costFor(waypoint.position)))
        .orElseThrow();
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
