package frc.robot;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.Test;

import edu.wpi.first.wpilibj.AnalogInput;

public class PressureSensorTest {
  // Floating point math is inexeact, agreeing to 1x10^-3 should be
  // good enough.
  private static final double TOLERANCE = 1e-3;

  private AnalogInput getFakeInput(double voltageToReturn) {
    AnalogInput fakeInput = mock(AnalogInput.class);
    when(fakeInput.getVoltage()).thenReturn(voltageToReturn);
    return fakeInput;
  }

  @Test
  public void testGetPressureEmpty() {
    AnalogInput fakeInput = getFakeInput(0.5);
    PressureSensor sensor = new PressureSensor(fakeInput);
    assertEquals(0.0, sensor.getPressure(), TOLERANCE);
  }

  @Test
  public void testGetPressureFull() {
    AnalogInput fakeInput = getFakeInput(2.70);
    PressureSensor sensor = new PressureSensor(fakeInput);
    assertEquals(110.0, sensor.getPressure(), TOLERANCE);
  }

  @Test
  public void testGetPressureHalf() {
    AnalogInput fakeInput = getFakeInput(1.6);
    PressureSensor sensor = new PressureSensor(fakeInput);
    assertEquals(55.0, sensor.getPressure(), TOLERANCE);
  }
}