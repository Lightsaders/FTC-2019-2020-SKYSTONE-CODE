package org.firstinspires.ftc.teamcode;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/*
 * This is an example LinearOpMode that shows how to use a color sensor in a generic
 * way, insensitive which particular make or model of color sensor is used. The opmode
 * assumes that the color sensor is configured with a name of "color sensor".
 *
 * If the color sensor has a light which is controllable, you can use the X button on
 * the gamepad to toggle the light on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "Sensor: Color", group = "Sensor")

public class colors extends LinearOpMode {

    /** The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;
    NormalizedColorSensor colorSensor0;
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need something analogous when you
     * use a color sensor on your robot */
    View relativeLayout;

    /**
     * The runOpMode() method is the root of this LinearOpMode, as it is in all linear opModes.
     * Our implementation here, though is a bit unusual: we've decided to put all the actual work
     * in the main() method rather than directly in runOpMode() itself. The reason we do that is that
     * in this sample we're changing the background color of the robot controller screen as the
     * opmode runs, and we want to be able to *guarantee* that we restore it to something reasonable
     * and palatable when the opMode ends. The simplest way to do that is to use a try...finally
     * block around the main, core logic, and an easy way to make that all clear was to separate
     * the former from the latter in separate methods.
     */
    @Override public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor0 = hardwareMap.get(NormalizedColorSensor.class, "colorb");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        if (colorSensor0 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor0).enableLight(true);
        }

        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        while (opModeIsActive() && !isStopRequested() ){
            // Check the status of the x button on the gamepad

            // Read the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            NormalizedRGBA colors0 = colorSensor0.getNormalizedColors();
            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/


            int color = colors.toColor();
            int color0 = colors0.toColor();

            // Balance the colors. The values returned by getColors() are normalized relative to the
            // maximum possible values that the sensor can measure. For example, a sensor might in a
            // particular configuration be able to internally measure color intensity in a range of
            // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
            // so as to return a value it the range [0,1]. However, and this is the point, even so, the
            // values we see here may not get close to 1.0 in, e.g., low light conditions where the
            // sensor measurements don't approach their maximum limit. In such situations, the *relative*
            // intensities of the colors are likely what is most interesting. Here, for example, we boost
            // the signal on the colors while maintaining their relative balance so as to give more
            // vibrant visual feedback on the robot controller visual display.
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
            color = colors.toColor();
            float max0 = Math.max(Math.max(Math.max(colors0.red, colors0.green), colors0.blue), colors0.alpha);
            colors0.red /= max0;
            colors0.green /= max0;
            colors0.blue /= max0;
            color0 = colors0.toColor();


            telemetry.addLine("normalized color:  ")
                    .addData("a",  Color.alpha(color))
                    .addData("r",  Color.red(color))
                    .addData("g",  Color.green(color))
                    .addData("b",  Color.blue(color));
  
            telemetry.addLine("normalized color0:  ")
                    .addData("a",  Color.alpha(color0))
                    .addData("r",  Color.red(color0))
                    .addData("g",  Color.green(color0))
                    .addData("b",  Color.blue(color0));
            telemetry.update();
            if (Color.alpha(color) < 30 && opModeIsActive() && !isStopRequested()) {
               telemetry.addLine("SKYSTONEMIDDLE");

                colors = colorSensor.getNormalizedColors();
                color  = colors.toColor();
                max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
                colors.red /= max;
                colors.green /= max;
                colors.blue /= max;
                color = colors.toColor();

                telemetry.addLine("IN LOOP")
                        .addData("a",  Color.alpha(color))
                        .addData("r",  Color.red(color))
                        .addData("g",  Color.green(color))
                        .addData("b",  Color.blue(color));
                telemetry.update();
            } else if (Color.red(color0) < 180 && opModeIsActive() && !isStopRequested()){
                telemetry.addLine(" SKYSTONE WALL");

                colors0 = colorSensor0.getNormalizedColors();
                color0  = colors0.toColor();
                max0 = Math.max(Math.max(Math.max(colors0.red, colors0.green), colors0.blue), colors0.alpha);
                colors0.red /= max0;
                colors0.green /= max0;
                colors0.blue /= max0;
                color0 = colors.toColor();

                telemetry.addLine("IN LOOPb")
                        .addData("a",  Color.alpha(color0))
                        .addData("r",  Color.red(color0))
                        .addData("g",  Color.green(color0))
                        .addData("b",  Color.blue(color0));
                telemetry.update();
            }else{
                telemetry.addLine(" SKYSTONE FAR");
            }




        }

    }
}