package frc.robot;

import org.junit.Test;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;

import edu.wpi.first.wpilibj.I2C;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.*;

import java.nio.ByteBuffer;

public class LidarTest {
  @Test
  public void testGetDistanceInCentimeters() {
    // Fake an I2C conversation.
    I2C fakeI2C = mock(I2C.class);

    byte[] waiting = { 0x01 };
    byte[] done = { 0x00 };
    byte[] result = { 0x01, 0x03 }; // 259 cm: 1 * 256 + 3 * 1

    when(fakeI2C.write(eq(0x00), eq(0x04))).thenReturn(false);

    // Simulate a few cycles without the status register showing "ready"
    // before returning 0.
    when(fakeI2C.read(eq(0x01), eq(1), any(ByteBuffer.class)))
      .thenAnswer(parrotRead(waiting, false))
      .thenAnswer(parrotRead(waiting, false))
      .thenAnswer(parrotRead(done, false));

    // Finally provide the result.
    when(fakeI2C.read(eq(0x8f), eq(2), any(ByteBuffer.class)))
      .thenAnswer(parrotRead(result, false));

    Lidar lidar = new Lidar(fakeI2C);
    assertEquals(259, lidar.getDistanceInCentimeters());
  }

  /**
   * Shorthand to return a canned response from the I2C response to a read request.
   * @param toBeRead byte array containing the response.
   * @param retval whether or not to indicate the transaction was aborted
   * @return a Mockito Answer to provide a stubbed response
   */
  private static Answer<Boolean> parrotRead(byte[] toBeRead, boolean retval) {
    return new Answer<Boolean>() {
      public Boolean answer(InvocationOnMock invocation) {
        // This modifies the underlying array of the buffer handed to us,
        // just like the real I2C class will.
        byte[] backer = ((ByteBuffer)invocation.getArgument(2)).array();

        for (int i = 0; i < toBeRead.length; ++i) {
          backer[i] = toBeRead[i];
        }
        return retval;
      }
    };
  }
}