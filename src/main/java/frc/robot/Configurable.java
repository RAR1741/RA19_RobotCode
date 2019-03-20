package frc.robot;

import com.moandjiezana.toml.Toml;

/**
 * An interface for a TOML-configurable system.
 */
public interface Configurable {
  /**
   * Configure the system with a supplied TOML config object.
   * 
   * @param config a TOML configuration object
   */
  public void configure(Toml config);
}