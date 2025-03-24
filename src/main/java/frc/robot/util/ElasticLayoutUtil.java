package frc.robot.util;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Make Elastic layout available via HTTP
 * https://frc-elastic.gitbook.io/docs/additional-features-and-references/remote-layout-downloading#on-robot-configuration
 *
 * <p>Use the "merge" strategy for downloading the layout
 */
public final class ElasticLayoutUtil {
  private static final int PORT = 5800;
  private static boolean isServerRunning = false;

  public static void onBoot() {
    startServer();
  }

  public static void onEnable() {
    stopServer();
  }

  public static void onDisable() {
    startServer();
  }

  public static final void startServer() {
    if (!isServerRunning) {
      isServerRunning = true;
      WebServer.start(PORT, Filesystem.getDeployDirectory().getPath());
    }
  }

  private static final void stopServer() {
    WebServer.stop(PORT);
    isServerRunning = false;
  }

  private ElasticLayoutUtil() {}
}
