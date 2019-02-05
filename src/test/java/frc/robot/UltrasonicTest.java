package frc.robot;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.Test;

import edu.wpi.first.wpilibj.*;

public class UltrasonicTest{
  // Floating point math is inexeact, agreeing to 1x10^-3 should be
  // good enough.
  // In addition, the ultrasonic calculation is not exact, so we'll
  // ignore an error within 1 cm.
  private static final double TOLERANCE = 1;

  private AnalogInput getFakeInput(double voltageToReturn) {
    AnalogInput fakeInput = mock(AnalogInput.class);
    when(fakeInput.getVoltage()).thenReturn(voltageToReturn);
    return fakeInput;
  }

  @Test
  public void testReference1() {
    AnalogInput fakeInput = getFakeInput(0.293);
    UltrasonicSensor sensor = new UltrasonicSensor(fakeInput);
    assertEquals(30.00, sensor.getDistance(), TOLERANCE);
  }

  @Test
  public void testReference2() {
    AnalogInput fakeInput = getFakeInput(4.885);
    UltrasonicSensor sensor = new UltrasonicSensor(fakeInput);
    assertEquals(500.0, sensor.getDistance(), TOLERANCE);
  }
}
