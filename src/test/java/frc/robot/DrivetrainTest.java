package frc.robot;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.Test;

import edu.wpi.first.wpilibj.*;

public class DrivetrainTest {

  private DigitalInput getFakeInput(Boolean booleanToReturn) {
    DigitalInput fakeInput = mock(DigitalInput.class);
    when(fakeInput.get()).thenReturn(booleanToReturn);
    return fakeInput;
  }

  @Test
  public void testReference1() {
    DigitalInput fakeLeftInput = getFakeInput(false);
    DigitalInput fakeRightInput = getFakeInput(false);

    Drivetrain fakeDrive = new Drivetrain(4, 5, 6, 7, 1, 2, 3);

    // assertEquals(30.00, fakeDrive.getDistance());
  }
}
