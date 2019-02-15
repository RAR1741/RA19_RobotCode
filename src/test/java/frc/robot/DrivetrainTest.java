package frc.robot;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.junit.Test;
import org.mockito.Mockito;

import edu.wpi.first.wpilibj.DigitalInput;

public class DrivetrainTest {

  private Drivetrain getFakeDrivetrain(boolean left, boolean middle, boolean right) {
    DigitalInput fakeLeftInput = getFakeInput(left);
    DigitalInput fakeMiddleInput = getFakeInput(middle);
    DigitalInput fakeRightInput = getFakeInput(right);

    return Mockito.spy(new Drivetrain(getFakeTalon(), getFakeTalon(), getFakeTalon(), getFakeTalon(), fakeLeftInput,
        fakeMiddleInput, fakeRightInput));
  }

  private DigitalInput getFakeInput(Boolean booleanToReturn) {
    DigitalInput fakeInput = mock(DigitalInput.class);
    when(fakeInput.get()).thenReturn(booleanToReturn);
    return fakeInput;
  }

  private WPI_TalonSRX getFakeTalon() {
    WPI_TalonSRX fakeTalon = mock(WPI_TalonSRX.class);
    return fakeTalon;
  }

  @Test
  public void testNoSensorReading() {
    Drivetrain fakeDrive = getFakeDrivetrain(false, false, false);
    fakeDrive.followLine(1.0);
    verify(fakeDrive, times(1)).driveLeft(1.0);
    verify(fakeDrive, times(1)).driveRight(1.0);
  }

  @Test
  public void testLeftSensorReading() {
    Drivetrain fakeDrive = getFakeDrivetrain(true, false, false);
    fakeDrive.followLine(1.0);
    verify(fakeDrive, times(1)).driveLeft(0.75);
    verify(fakeDrive, times(1)).driveRight(1.0);
  }

  @Test
  public void testRightSensorReading() {
    Drivetrain fakeDrive = getFakeDrivetrain(false, false, true);
    fakeDrive.followLine(1.0);
    verify(fakeDrive, times(1)).driveLeft(1.0);
    verify(fakeDrive, times(1)).driveRight(0.75);
  }

  @Test
  public void testBothSensorReading() {
    Drivetrain fakeDrive = getFakeDrivetrain(true, false, true);
    fakeDrive.followLine(1.0);
    verify(fakeDrive, times(1)).driveLeft(0.0);
    verify(fakeDrive, times(1)).driveRight(0.0);
  }

  @Test
  public void testMaxMotorInput() {
    Drivetrain fakeDrive = getFakeDrivetrain(false, false, false);
    fakeDrive.followLine(0.5);
    verify(fakeDrive, times(1)).driveLeft(0.5);
    verify(fakeDrive, times(1)).driveRight(0.5);
  }
}
