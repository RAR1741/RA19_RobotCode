package frc.robot;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.Test;

import edu.wpi.first.wpilibj.*;

public class UltrasonicTest{
  // Floating point math is inexeact, agreeing to 1x10^-3 should be
  // good enough.
  private static final double TOLERANCE = 1e-3;

  private AnalogInput getFakeInput(double voltageToReturn) {
    AnalogInput fakeInput = mock(AnalogInput.class);
    when(fakeInput.getVoltage()).thenReturn(voltageToReturn);
    return fakeInput;
  }

  @Test
  public void testGetDistanceNone() {
    AnalogInput fakeInput = getFakeInput(0);
    UltrasonicSensor sensor = new UltrasonicSensor(fakeInput);
    assertEquals(0.0, sensor.getDistance(), TOLERANCE);
  }

  @Test
  public void testGetDistanceMid() {
    AnalogInput fakeInput = getFakeInput(2.5);
    UltrasonicSensor sensor = new UltrasonicSensor(fakeInput);
    assertEquals(1.25, sensor.getDistance(), TOLERANCE);
  }

  @Test
  public void testGetDistanceFull() {
    AnalogInput fakeInput = getFakeInput(5.0);
    UltrasonicSensor sensor = new UltrasonicSensor(fakeInput);
    assertEquals(2.5, sensor.getDistance(), TOLERANCE);
  }
}