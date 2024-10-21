package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.w8wjb.ftc.AdafruitNeoDriver;

@Autonomous
public class ColorTest extends LinearOpMode {

    private final int NUM_PIXELS = 100;

    AdafruitNeoDriver neopixels;

    Timer timer = new Timer();

    private int redStart = 0;
    private int hueGap = 0;
    private int counter = 0;
    private int counterB = 0;
    int color = Color.rgb(74, 230, 255);

    /****
     *
     *
     @ColorInt public static final int BLACK       = 0xFF000000;
     @ColorInt public static final int DKGRAY      = 0xFF444444;
     @ColorInt public static final int GRAY        = 0xFF888888;
     @ColorInt public static final int LTGRAY      = 0xFFCCCCCC;
     @ColorInt public static final int WHITE       = 0xFFFFFFFF;
     @ColorInt public static final int RED         = 0xFFFF0000;
     @ColorInt public static final int GREEN       = 0xFF00FF00;
     @ColorInt public static final int BLUE        = 0xFF0000FF;
     @ColorInt public static final int YELLOW      = 0xFFFFFF00;
     @ColorInt public static final int CYAN        = 0xFF00FFFF;
     @ColorInt public static final int MAGENTA     = 0xFFFF00FF;
     @ColorInt public static final int TRANSPARENT = 0;
     *
     *
     */


    @Override
    public void runOpMode() {

        neopixels = hardwareMap.get(AdafruitNeoDriver.class, "neopixels");

        neopixels.setNumberOfPixels(NUM_PIXELS);

        timer.reset();

        hueGap = 360 / NUM_PIXELS;

        waitForStart();

        while(opModeIsActive()) {

            neopixels.setPixelColor(0,color);
            neopixels.setPixelColor(2,color);
            neopixels.setPixelColor(3,color);
            neopixels.setPixelColor(4,color);
            neopixels.setPixelColor(5,color);
            neopixels.setPixelColor(6,color);
            neopixels.setPixelColor(7,color);
            neopixels.setPixelColor(8,color);
            neopixels.setPixelColor(9,color);

            neopixels.show();
            telemetry.addData("Color: ",color);
            telemetry.update();


        }

    }

}
