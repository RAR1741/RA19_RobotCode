package frc.robot;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.Test;

import edu.wpi.first.wpilibj.AnalogInput;

public class PressureSensorTest {
  private static final double TOLERANCE = 1e-3;

  @Test
  public void testGetPressureEmpty() {
    AnalogInput fakeInput = mock(AnalogInput.class);
    when(fakeInput.getVoltage()).thenReturn(0.5);

    PressureSensor sensor = new PressureSensor(fakeInput);
    assertEquals(0.0, sensor.getPressure(), TOLERANCE);
  }

  @Test
  public void testGetPressureFull() {
    AnalogInput fakeInput = mock(AnalogInput.class);
    when (fakeInput.getVoltage()).thenReturn(2.70);

    PressureSensor sensor = new PressureSensor(fakeInput);
    assertEquals(120.0, sensor.getPressure(), TOLERANCE);
  }
}