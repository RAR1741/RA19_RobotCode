package frc.robot;

import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.junit.Before;
import org.junit.Test;

import frc.robot.loggable.LoggableDoubleSolenoid;
import frc.robot.loggable.LoggableTalonSRX;

public class ScoringTest {
  LoggableTalonSRX fakeRoller;
  LoggableTalonSRX fakeWrist;
  LoggableDoubleSolenoid fakePushers;
  LoggableDoubleSolenoid fakePivots;

  @Before
  public void beforeEach() {
    fakeRoller = mock(LoggableTalonSRX.class);
    fakeWrist = mock(LoggableTalonSRX.class);
    fakePushers = mock(LoggableDoubleSolenoid.class);
    fakePivots = mock(LoggableDoubleSolenoid.class);
  }

  private Scoring getTestScoreInstance() {
    return new Scoring(fakeRoller, fakeWrist, fakePushers, fakePivots);
  }

  @Test
  public void testRollers() {
    Scoring scoring = getTestScoreInstance();
    scoring.roll(0.5);
    // verify(fakeRoller, times(1)).set(eq(ControlMode.PercentOutput), eq(0.5));
  }

  @Test
  public void testTilt() {
    Scoring scoring = getTestScoreInstance();
    scoring.tilt(0.7);
    // verify(fakeWrist, times(1)).set(eq(ControlMode.PercentOutput), eq(0.7));
  }

  @Test
  public void testExtend() {
    Scoring scoring = getTestScoreInstance();
    scoring.extend();
    verify(fakePushers, times(1)).set(eq(LoggableDoubleSolenoid.Value.kReverse));
  }
}