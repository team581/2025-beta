package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.ValueGraph;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.MathHelpers;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public class GraphVisualizer {
  private static final boolean HIDE_INFINITE_COSTS = false;

  public static void visualize() {
    var graph = CollisionAvoidance.getRawGraph();

    log(graph);
  }

  private static void log(ValueGraph<Waypoint, WaypointEdge> graph) {
    if (RobotBase.isReal()) {
      return;
    }

    var noObstructionContent =
        toMermaidGraphDiagram("Collision avoidance (unobstructed)", graph, ObstructionKind.NONE);
    var leftObstructedContent =
        toMermaidGraphDiagram(
            "Collision avoidance (left obstructed)", graph, ObstructionKind.LEFT_OBSTRUCTED);
    var rightObstructedContent =
        toMermaidGraphDiagram(
            "Collision avoidance (right obstructed)", graph, ObstructionKind.RIGHT_OBSTRUCTED);

    // Write to files
    var rootDir = System.getProperty("user.dir");

    var docsDirPath = Path.of(rootDir, "docs", "collision_avoidance");
    var docsDir = docsDirPath.toFile();
    docsDir.mkdirs();

    var noObstructionPath = docsDirPath.resolve(Path.of("no_obstruction.md"));
    var leftObstructedPath = docsDirPath.resolve(Path.of("left_obstructed.md"));
    var rightObstructedPath = docsDirPath.resolve(Path.of("right_obstructed.md"));

    try {
      Files.writeString(noObstructionPath, noObstructionContent);
    } catch (IOException e) {
      e.printStackTrace();
      DogLog.logFault("Couldn't write collision avoidance graph (unobstructed)");
    }

    try {
      Files.writeString(leftObstructedPath, leftObstructedContent);
    } catch (IOException e) {
      e.printStackTrace();
      DogLog.logFault("Couldn't write collision avoidance graph (left obstructed)");
    }

    try {
      Files.writeString(rightObstructedPath, rightObstructedContent);
    } catch (IOException e) {
      e.printStackTrace();
      DogLog.logFault("Couldn't write collision avoidance graph (right obstructed)");
    }
  }

  private static String toMermaidGraphDiagram(
      String title, ValueGraph<Waypoint, WaypointEdge> graph, ObstructionKind obstruction) {
    var sb = new StringBuilder();

    sb.append("# ");
    sb.append(title);
    sb.append(
        "\n\n```mermaid\n%%{ init: { \"flowchart\": { \"defaultRenderer\": \"elk\" } } }%%\ngraph TD\n\n");

    for (var pair : graph.edges()) {
      var edge = graph.edgeValue(pair).orElseThrow();
      var cost = edge.getCost(obstruction);

      if (cost == Double.MAX_VALUE) {
        if (HIDE_INFINITE_COSTS) {
          // Just ignore edges with infinite cost
          continue;
        }

        // Easier to view in the diagram this way
        cost = 999;
      }

      sb.append(pair.nodeU().toString());
      sb.append(" <-- ");
      sb.append(MathHelpers.roundTo(cost, 3));
      sb.append(" --> ");
      sb.append(pair.nodeV().toString());
      sb.append('\n');
    }

    sb.append("```\n");

    return sb.toString();
  }

  private GraphVisualizer() {}
}
