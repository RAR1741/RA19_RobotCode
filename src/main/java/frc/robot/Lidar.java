package frc.robot;

/**
 * LIDAR-based range finder (AndyMark am-3829)
 * @see https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf
 */

import edu.wpi.first.wpilibj.I2C;
import java.nio.ByteBuffer;

public class Lidar {
  private I2C device;

  public Lidar(I2C.Port port) {
    this(port, 0x62);
  }
  public Lidar(I2C.Port port, int deviceAddress) {
    device = new I2C(port, deviceAddress);
  }

  /**
   * Utility method for debugging 
   */
  void debug() {
    // This *should* be the simple way to get distance
    device.write(0x00, 0x04);
    boolean done = false;
    // These should be instance variables to avoid constantly allocating
    // tiny arrays.
    ByteBuffer statusBuffer = ByteBuffer.allocate(1);
    ByteBuffer measurementBuffer = ByteBuffer.allocate(2);

    while (!done) {
      device.read(0x01, 1, statusBuffer);
      
      if (statusBuffer.get(0) == 0) {
        done = true;
      }

      // TODO: Sleep or something
    }

    device.read(0x8f, 2, measurementBuffer);
    System.out.printf("Distance = %d cm", measurementBuffer.getShort());
  }
}