package frc.robot;

import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.junit.Test;

import frc.robot.loggable.LoggableTalonSRX;

public class ManipulationTest {
  @Test
  public void testLift() {
    LoggableTalonSRX fakeTalon = mock(LoggableTalonSRX.class);
    Manipulation manipulation = new Manipulation(fakeTalon);
    manipulation.lift(0.75);
    verify(fakeTalon, times(1)).set(eq(ControlMode.PercentOutput), eq(0.75));
  }
}