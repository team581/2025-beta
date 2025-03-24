package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.ElementOrder;
import com.google.common.graph.ImmutableValueGraph;
import com.google.common.graph.MutableValueGraph;
import com.google.common.graph.ValueGraph;
import com.google.common.graph.ValueGraphBuilder;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.Deque;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class CollisionAvoidance {

  private static final ValueGraph<Waypoint, WaypointEdge> graph = createGraph();

  /**
   * Returns an {@link Optional} containing the next {@link Waypoint} in the graph to go to. Returns
   * an empty Optional if there is no possible routing (impossible to avoid a collision or you are
   * at final waypoint).
   *
   * @param currentPosition The current position of the superstructure.
   * @param desiredPosition The desired position of the superstructure.
   * @param obstructionKind Additional constraints based on robot position.
   */
  public static Optional<Waypoint> route(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {

    var maybeResult = aStar(currentPosition, desiredPosition, obstructionKind);
    if (maybeResult.isPresent()) {
      return Optional.of(
          maybeResult.orElseThrow().peek()); // go to the first waypoint in the list to goal
    }
    return Optional.empty();
  }

  private static ImmutableValueGraph<Waypoint, WaypointEdge> createGraph() {
    // Create an undirected value graph to represent safe motion between waypoints. Undirected
    // because if you can go from A to B, you can also go from B to A. Value graph because we want
    // to associate a cost with motion between different waypoints.
    MutableValueGraph<Waypoint, WaypointEdge> graph =
        ValueGraphBuilder.undirected().incidentEdgeOrder(ElementOrder.stable()).build();

    // Middle
    Waypoint.STOWED.canMoveToAlways(Waypoint.HANDOFF, graph);

    Waypoint.STOWED.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.STOWED.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.STOWED.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.STOWED.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.STOWED.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.STOWED.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    Waypoint.HANDOFF.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.HANDOFF.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.HANDOFF.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.HANDOFF.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.HANDOFF.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);
    Waypoint.HANDOFF.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.LOLLIPOP_INTAKE_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_INTAKE_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L1_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L2_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L1_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L2_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    // Left side

    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToAlways(Waypoint.LOLLIPOP_INTAKE_RIGHT, graph);
    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L1_RIGHT, graph);
    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L2_RIGHT, graph);
    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.LOLLIPOP_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L1_RIGHT, graph);
    Waypoint.LOLLIPOP_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L2_RIGHT, graph);
    Waypoint.LOLLIPOP_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.LOLLIPOP_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.LOLLIPOP_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.L1_LEFT.canMoveToWhenLeftSafe(Waypoint.L2_LEFT, graph);
    Waypoint.L1_LEFT.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.L1_LEFT.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.L1_LEFT.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    Waypoint.L2_LEFT.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.L2_LEFT.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.L2_LEFT.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    Waypoint.L3_LEFT.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.L3_LEFT.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    Waypoint.L4_LEFT.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    // Right side
    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.L2_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.L2_RIGHT.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.L3_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.L4_RIGHT.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    // Create an immutable copy of the graph now that we've added all the nodes
    var immutableGraph = ImmutableValueGraph.copyOf(graph);

    // Visualize the generated graph
    Waypoint.log();

    return immutableGraph;
  }

  /**
   * Returns an {@link Optional} containing the next {@link Waypoint} in the graph to go to. Returns
   * an empty Optional if there is no possible routing (impossible to avoid a collision or you are
   * at final waypoint).
   */
  private static Deque<Waypoint> reconstructPath(
      Map<Waypoint, Waypoint> cameFrom, Waypoint endWaypoint) {
    Deque<Waypoint> totalPath = new ArrayDeque<Waypoint>();
    totalPath.add(endWaypoint);
    Waypoint current = endWaypoint;
    while (cameFrom.containsKey(current)) {
      current = cameFrom.get(current);
      totalPath.addFirst(current);
    }

    return totalPath;
  }

  static Optional<Deque<Waypoint>> aStar(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    var openSet = EnumSet.of(Waypoint.getClosest(currentPosition));

    Map<Waypoint, Waypoint> cameFrom = new EnumMap<Waypoint, Waypoint>(Waypoint.class);

    Map<Waypoint, Double> gscore = new EnumMap<Waypoint, Double>(Waypoint.class);

    for (var waypoint : Waypoint.values()) {
      gscore.put(waypoint, Double.MAX_VALUE);
    }

    Waypoint startWaypoint = Waypoint.getClosest(currentPosition);
    Waypoint goalWaypoint = Waypoint.getClosest(desiredPosition);

    gscore.put(startWaypoint, 0.0);
    Waypoint current = Waypoint.STOWED;
    while (!openSet.isEmpty()) {
      // current is equal to the waypoint in openset that has the smallest gscore
      var maybeCurrent =
          openSet.stream()
              .min(
                  Comparator.comparingDouble(
                      waypoint -> gscore.getOrDefault(waypoint, Double.MAX_VALUE)));
      if (maybeCurrent.isPresent()) {
        current = maybeCurrent.get();
      }

      if (current == goalWaypoint) {
        var totalPath = reconstructPath(cameFrom, current);
        return Optional.of(totalPath);
      }
      openSet.remove(current);
      Set<Waypoint> options = graph.adjacentNodes(current);

      for (Waypoint neighbor : options) {
        var edge = graph.edgeValue(current, neighbor);
        double tentativeGScore =
            gscore.getOrDefault(current, Double.MAX_VALUE)
                + edge.orElseThrow().getCost(obstructionKind);
        if (tentativeGScore < gscore.getOrDefault(neighbor, Double.MAX_VALUE)) {
          cameFrom.put(neighbor, current);
          gscore.put(neighbor, tentativeGScore);
          openSet.add(neighbor);
        }
      }
    }
    return Optional.empty();
  }

  /** Don't use this. */
  static ValueGraph<Waypoint, WaypointEdge> getRawGraph() {
    return graph;
  }

  private CollisionAvoidance() {}
}
