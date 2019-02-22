package frc.robot;

import java.util.Objects;

/**
 * RuntimeDetector
 */
public class RuntimeDetector {
  public static boolean isSimulation() {
    return Objects.equals(System.getProperty("sun.java.command"), "com.snobot.simulator.Main");
  }
}
