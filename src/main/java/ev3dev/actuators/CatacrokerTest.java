package ev3dev.actuators;

import ev3dev.utils.Brickman;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.robotics.Color;
import lejos.utility.Delay;

public class CatacrokerTest {

    public static GraphicsLCD lcd = LCD.getInstance();

    public static void main(String[] args){

        Brickman.disable();

        System.out.println("EV3 LCD Example");

        lcd.setColor(Color.WHITE);
        lcd.drawLine(0, 0, 50, 50);
        lcd.drawLine(0, 0, 30, 60);
        lcd.refresh();

        for(int x = 0; x< 10; x++){
            System.out.println("Delay " + x);
            Delay.msDelay(1000);
        }

        //throw new RuntimeException("Catacroker");
        System.exit(0);

    }
}