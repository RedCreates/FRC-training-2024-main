package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LedSubsystem extends SubsystemBase {
    
    private static AddressableLED ledBar; // instance of the led bar class

    // Buffer or the holder of the color's data and its variable
    private static AddressableLEDBuffer led_red_alliance;
    private static AddressableLEDBuffer led_blue_alliance;
    private static AddressableLEDBuffer led_red_blue;
    private static AddressableLEDBuffer led_blank;
    private static AddressableLEDBuffer led_green;
    private static AddressableLEDBuffer led_dynamic_msg;

    private static int dynamicCount = 0; // counter for dynamic messages
    private static int rainbowFirstPixelHue = 0; // tracks the hue for the first pixel in the rainbow effect
    public static boolean dynamicEffect = false; // a tracking varible for checking dynamic effect

    // a block that runs once when class is initially loaded
    static {
        ledBar = new AddressableLED(LEDConstants.kLEDBarPWM); // initializes the led bar object with the given power port
        ledBar.setLength(LEDConstants.ledLength);

        // set buffers to their specified length

        led_red_alliance = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_blue_alliance = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_red_blue = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_blank = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_green = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_dynamic_msg = new AddressableLEDBuffer(LEDConstants.ledBufferLength); // specific blinking message

        // initial message for led buffers
        for (int i = 0 ; i < LEDConstants.ledLength; i++){
            // pre-set all message buffers during initilization
            led_red_alliance.setLED(i, Color.kRed);
            led_blue_alliance.setLED(i, Color.kBlue);
            led_green.setLED(i, Color.kGreen);

            // using bitwise-AND operators .... (alternating red-blue led)
            if (((i & 3) == 0) // if the last two binary value of i & 3 is the same as 0's
             || ((i & 3) == 2) // if if the last two binary value of i & 3 is the same as 2's
             ){
                led_red_blue.setLED(i, Color.kRed);
            }  else {
                led_red_blue.setLED(i, Color.kBlue);
            }

            led_blank.setLED(i, Color.kBlack);
            led_dynamic_msg.setLED(i, Color.kBlack); // no pre-set for dynamic msg
        }

        ledBar.setData(led_blank); // displays the black-led at first
        ledBar.start(); // activates the led strip
    }

    public static void stopLedBar(){
        ledBar.stop(); // turns off
    }

    public static void startLedBar(){
        ledBar.start(); // turns on
    }

    public static void setBlankMsg(){
        dynamicEffect = false;
        ledBar.setData(led_blank); // doesn't light up
    }

    public static void setRedBlueMsg(){
        dynamicEffect = false; 
        ledBar.setData(led_red_blue); // lights up red_blue
    }

    public static void setGreenMsg(){
        dynamicEffect = false;
        ledBar.setData(led_green); // lights up green
    }

    public static void setAllianceSolid(){ // sets allaince color in led
        DriverStation.Alliance friendlyAlliance = DriverStation.getAlliance().get();
        if(friendlyAlliance == DriverStation.Alliance.Red){
            ledBar.setData(led_red_alliance);
        }
        else if(friendlyAlliance == DriverStation.Alliance.Blue){
            ledBar.setData(led_blue_alliance);
        }
        else {
            ledBar.setData(led_red_blue);
        }
    }

    public static void setRainbow(){ // sets rainbow led
        // For every pixel
        for (var i = 0; i < led_dynamic_msg.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (rainbowFirstPixelHue + (i * 180 / led_dynamic_msg.getLength())) % 180;
          // Set the value
          led_dynamic_msg.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    public static void setDynamicMsg(){
        if ( dynamicCount < 1000 ){
            dynamicCount++;
        }
        setRainbow();
        ledBar.setData(led_dynamic_msg);
    }
}
