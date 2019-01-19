package frc.robot;

/**
 * LIDAR-based range finder (AndyMark am-3829)
 * @see https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf
 */

import edu.wpi.first.wpilibj.I2C;
import java.nio.ByteBuffer;

public class Lidar {
  // I2C device handle
  private I2C device;

  // Places to receive I2C responses to transactions
  private ByteBuffer statusBuffer = ByteBuffer.allocate(1);
  private ByteBuffer measurementBuffer = ByteBuffer.allocate(2);

  public Lidar(I2C handle) {
    this.device = handle;
  }

  private static final byte REGISTER_ACQ_COMMAND = 0x00;
  private static final byte REGISTER_STATUS = 0x01;
  private static final byte REGISTER_FULL_DELAY_HIGH = 0x0f;

  // Setting the most significant bit of the address byte turns on 
  // auto-incrementing of addresses for reads and writes in a single transfer.
  private static final byte AUTO_INCREMENT_MASK = (byte) 0x80;

  /**
   * Query LIDAR and measure the distance to the target in cm.
   * @return distance to target in centimeters
   */
  public int getDistanceInCentimeters() {
    // 1. Write command to take a new measurement to the command register.
    device.write(REGISTER_ACQ_COMMAND, 0x04);

    // 2. Read the status register, and poll it until it bit 0 "goes low"
    boolean done = false;

    while (!done) {
      device.read(REGISTER_STATUS, 1, statusBuffer);

      byte result = statusBuffer.get(0);
      statusBuffer.rewind();

      if ((result & 0x01) == 0) {
        done = true;
      }
    }

    // 3. Read two bytes from the measurement register and treat them as a 16-bit CM distance.
    measurementBuffer.rewind();
    device.read(REGISTER_FULL_DELAY_HIGH | AUTO_INCREMENT_MASK & 0xff, 2, measurementBuffer);
    return measurementBuffer.getShort();
  }
}