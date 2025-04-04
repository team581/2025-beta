package frc.robot.robot_manager.collision_avoidance;

import com.google.common.collect.ImmutableList;
import com.google.common.graph.ElementOrder;
import com.google.common.graph.ImmutableValueGraph;
import com.google.common.graph.MutableValueGraph;
import com.google.common.graph.ValueGraphBuilder;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.Deque;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class CollisionAvoidance {
  private static final double ELEVATOR_TOLERANCE = 25.0;
  private static final double ARM_TOLERANCE = 40.0;

  private static final ImmutableValueGraph<Waypoint, WaypointEdge> graph = createGraph();

  private static final Map<CollisionAvoidanceQuery, Optional<ImmutableList<Waypoint>>> aStarCache =
      new HashMap<>();

  private static CollisionAvoidanceQuery lastQuery =
      new CollisionAvoidanceQuery(Waypoint.STOWED_UP, Waypoint.STOWED_UP, ObstructionKind.NONE);
  private static Deque<Waypoint> lastPath = new ArrayDeque<>();

  private static boolean hasGeneratedPath = false;

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
    DogLog.log("CollisionAvoidance/ClawPos", currentPosition.getTranslation());

    if (DriverStation.isDisabled()) {
      return Optional.empty();
    }
    DogLog.log("CollisionAvoidance/DesiredWaypoint", Waypoint.getClosest(desiredPosition));
    // Check if the desired position and obstruction is the same, then use the same path
    if (!lastQuery.goalWaypoint().equals(Waypoint.getClosest(desiredPosition))
        || !lastQuery.obstructionKind().equals(obstructionKind)) {
      var currentWaypoint = Waypoint.getClosest(currentPosition);
      lastQuery =
          new CollisionAvoidanceQuery(
              currentWaypoint, Waypoint.getClosest(desiredPosition), obstructionKind);

      var maybePath = cachedAStar(lastQuery).map(ArrayDeque::new);
      if (maybePath.isPresent()) {
        hasGeneratedPath = true;
        lastPath = maybePath.orElseThrow();
      } else {
        return Optional.empty();
      }
    }

    if (lastPath.isEmpty()) {
      return Optional.empty();
    }

    var currentWaypoint = lastPath.getFirst();

    DogLog.log(
        "CollisionAvoidance/CurrentWaypoint/ElevatorHeight",
        currentWaypoint.position.elevatorHeight());
    DogLog.log("CollisionAvoidance/CurrentWaypoint/ArmAngle", currentWaypoint.position.armAngle());
    DogLog.log("CollisionAvoidance/CurrentWaypoint", currentWaypoint);
    DogLog.log("CollisionAvoidance/ClosestWaypoint", Waypoint.getClosest(currentPosition));
    DogLog.log("CollisionAvoidance/AstarPath", lastPath.toArray(Waypoint[]::new));

    DogLog.log(
        "CollisionAvoidance/CurrentPosition/ElevatorHeight", currentPosition.elevatorHeight());
    DogLog.log("CollisionAvoidance/CurrentPosition/ArmAngle", currentPosition.armAngle());

    boolean near =
        SuperstructurePosition.near(
            currentWaypoint.position, currentPosition, ELEVATOR_TOLERANCE, ARM_TOLERANCE);
    DogLog.log("CollisionAvoidance/Near", near);

    // Check if our current position is close to the current waypoint in path
    if (near) {
      // If it's close, return the next waypoint
      if (lastPath.isEmpty()) {
        return Optional.empty();
      }
      return Optional.of(lastPath.pop());
    }
    // If it's not close, return the same waypoint
    return Optional.of(currentWaypoint);
  }

  public static boolean isClimberAtRisk(
      SuperstructurePosition current, SuperstructurePosition goal) {
    Waypoint currentwWaypoint = Waypoint.getClosest(current);
    Waypoint goalWaypoint = Waypoint.getClosest(goal);

    var edge = graph.edgeValue(currentwWaypoint, goalWaypoint);
    if (edge.isEmpty()) {
      return true;
    }
    if (edge.get().climberAtRisk()) {
      return true;
    }
    return false;
  }

  private static Optional<ImmutableList<Waypoint>> cachedAStar(CollisionAvoidanceQuery query) {
    DogLog.log("CollisionAvoidance/AStarCacheSize", aStarCache.size());

    return aStarCache.computeIfAbsent(
        query,
        (k) ->
            aStar(
                query.currentWaypoint().position,
                query.goalWaypoint().position,
                query.obstructionKind()));
  }

  private static ImmutableValueGraph<Waypoint, WaypointEdge> createGraph() {
    // Create an undirected value graph to represent safe motion between waypoints. Undirected
    // because if you can go from A to B, you can also go from B to A. Value graph because we want
    // to associate a cost with motion between different waypoints.
    MutableValueGraph<Waypoint, WaypointEdge> graph =
        ValueGraphBuilder.undirected().incidentEdgeOrder(ElementOrder.stable()).build();

    // Middle
    Waypoint.STOWED.canMoveToAlways(Waypoint.HANDOFF, graph);

    Waypoint.STOWED.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.STOWED.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.STOWED.canMoveToWhenRightSafe(Waypoint.ALGAE_NET_UP, graph);

    Waypoint.STOWED_UP.canMoveToAlways(Waypoint.LEFT_SAFE_STOWED_UP, graph);

    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.LOLLIPOP_INTAKE_RIGHT, false, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.ALGAE_INTAKE_RIGHT, false, graph);
    Waypoint.LEFT_SAFE_STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L2_LEFT, graph);
    Waypoint.LEFT_SAFE_STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.LEFT_SAFE_STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.LEFT_SAFE_STOWED_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_NET_UP, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L1_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L2_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.ALGAE_NET_UP, graph);

    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_L2_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_L2_RIGHT, false, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_L3_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_L3_RIGHT, false, graph);

    Waypoint.ALGAE_NET_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_OUT_LEFT, graph);
    Waypoint.ALGAE_NET_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_OUT_RIGHT, graph);
    // Left side
    Waypoint.HANDOFF.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);

    Waypoint.L2_LEFT.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.L2_LEFT.canMoveToAlways(Waypoint.L2_LEFT_PLACE, graph);
    Waypoint.L2_LEFT.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);

    Waypoint.L3_LEFT.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.L3_LEFT.canMoveToAlways(Waypoint.L3_LEFT_PLACE, graph);
    Waypoint.L4_LEFT.canMoveToAlways(Waypoint.L4_LEFT_PLACE, graph);

    // Right side
    Waypoint.HANDOFF.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);

    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToAlways(Waypoint.LOLLIPOP_INTAKE_RIGHT, graph);
    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L1_RIGHT, graph);
    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L2_RIGHT, graph);
    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);

    Waypoint.LOLLIPOP_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L1_RIGHT, graph);
    Waypoint.LOLLIPOP_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L2_RIGHT, graph);
    Waypoint.LOLLIPOP_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.LOLLIPOP_INTAKE_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);

    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.L2_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);

    Waypoint.L2_RIGHT.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveToAlways(Waypoint.L2_RIGHT_PLACE, graph);

    Waypoint.L2_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);

    Waypoint.L3_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveToAlways(Waypoint.L3_RIGHT_PLACE, graph);
    Waypoint.L4_RIGHT.canMoveToAlways(Waypoint.L4_RIGHT_PLACE, graph);

    Waypoint.ALGAE_L2_RIGHT.canMoveToWhenLeftSafe(Waypoint.ALGAE_NET_UP, graph);
    Waypoint.ALGAE_L3_RIGHT.canMoveToWhenLeftSafe(Waypoint.ALGAE_NET_UP, graph);

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
  private static ImmutableList<Waypoint> reconstructPath(
      Map<Waypoint, Waypoint> cameFrom, Waypoint endWaypoint) {
    Deque<Waypoint> totalPath = new ArrayDeque<Waypoint>();
    totalPath.add(endWaypoint);
    Waypoint current = endWaypoint;
    while (cameFrom.containsKey(current)) {
      current = cameFrom.get(current);
      totalPath.addFirst(current);
    }

    return ImmutableList.copyOf(totalPath);
  }

  static Optional<ImmutableList<Waypoint>> aStar(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    var openSet = EnumSet.of(Waypoint.getClosest(currentPosition));

    Map<Waypoint, Waypoint> cameFrom = new EnumMap<Waypoint, Waypoint>(Waypoint.class);

    Map<Waypoint, Double> gscore = new EnumMap<Waypoint, Double>(Waypoint.class);

    Waypoint startWaypoint = Waypoint.getClosest(currentPosition);
    Waypoint goalWaypoint = Waypoint.getClosest(desiredPosition);
    if (startWaypoint.equals(goalWaypoint)) {
      DogLog.timestamp("CollisionAvoidance/StartAndEndSame");
      return Optional.empty();
    }

    gscore.put(startWaypoint, 0.0);
    Waypoint current = Waypoint.STOWED_UP;
    while (!openSet.isEmpty()) {
      // current is equal to the waypoint in openset that has the smallest gscore
      var maybeCurrent =
          openSet.stream()
              .min(
                  Comparator.comparingDouble(
                      waypoint -> gscore.getOrDefault(waypoint, Double.MAX_VALUE)));
      if (maybeCurrent.isPresent()) {
        current = maybeCurrent.orElseThrow();
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
    return Optional.of(ImmutableList.of(Waypoint.getClosest(currentPosition)));
  }

  /** Don't use this. */
  static ImmutableValueGraph<Waypoint, WaypointEdge> getRawGraph() {
    return graph;
  }

  private CollisionAvoidance() {}
}
