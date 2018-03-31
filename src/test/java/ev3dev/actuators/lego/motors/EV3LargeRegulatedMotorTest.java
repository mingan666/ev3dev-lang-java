package ev3dev.actuators.lego.motors;

import ev3dev.actuators.Sound;
import ev3dev.hardware.EV3DevFileSystem;
import ev3dev.hardware.EV3DevPlatform;
import fake_ev3dev.ev3dev.actuators.lego.motors.FakeEV3LargeRegulatedMotor;
import fake_ev3dev.ev3dev.sensors.FakeBattery;
import lejos.hardware.port.MotorPort;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;

public class EV3LargeRegulatedMotorTest {

    @Before
    public void resetTest() throws IOException, NoSuchFieldException, IllegalAccessException {

        FakeBattery.deleteEV3DevFakeSystemPath();
        FakeBattery.createEV3DevFakeSystemPath();

        System.setProperty(EV3DevFileSystem.EV3DEV_TESTING_KEY, FakeBattery.EV3DEV_FAKE_SYSTEM_PATH);

    }

    @Test
    public void configureMotorTest() throws Exception {

        final FakeBattery fakeBattery = new FakeBattery(EV3DevPlatform.EV3BRICK);
        final FakeEV3LargeRegulatedMotor fakeMotor = new FakeEV3LargeRegulatedMotor(EV3DevPlatform.EV3BRICK);

        EV3LargeRegulatedMotor motor = new EV3LargeRegulatedMotor(MotorPort.A);
    }

}