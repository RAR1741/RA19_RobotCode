package frc.robot.configuration;

import com.moandjiezana.toml.Toml;

public interface Configurable {
  /**
   * Apply configuration.
   * @param config
   */
  void configure(Toml config);
}