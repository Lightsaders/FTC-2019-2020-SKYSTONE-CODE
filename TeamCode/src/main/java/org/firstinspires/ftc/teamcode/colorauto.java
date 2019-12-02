package org.firstinspires.ftc.teamcode;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Autonomous(name = "Sensor: Colora", group = "Sensor")

public class colorauto extends LinearOpMode {

    /** The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need something analogous when you
     * use a color sensor on your robot */
    View relativeLayout;
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;

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

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        // bPrevState and bCurrState keep track of the previous and current state of the button
        boolean bPrevState = false;
        boolean bCurrState = false;

        // Get a reference to our sensor object.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        while (opModeIsActive() && !isStopRequested() ){
            // Check the status of the x button on the gamepad

            // Read the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/


            int color = colors.toColor();

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


            telemetry.addLine("BEFORE LOOP")
                    .addData("a",  Color.alpha(color))
                    .addData("r",  Color.red(color))
                    .addData("g",  Color.green(color))
                    .addData("b",  Color.blue(color));
            telemetry.update();
            while (Color.red(color) < 200 && opModeIsActive() && !isStopRequested()) {
                driveBackLeft.setPower(0.4);
                driveBackRight.setPower(0.4);
                driveFrontLeft.setPower(0.4);
                driveFrontRight.setPower(0.4);

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
            }
                driveBackLeft.setPower(0);
                driveBackRight.setPower(0);
                driveFrontLeft.setPower(0);
                driveFrontRight.setPower(0);

            // convert the RGB values to HSV values.
            Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            telemetry.addLine("AFTER LOOP")
                    .addData("a",  Color.alpha(color))
                    .addData("r",  Color.red(color))
                    .addData("g",  Color.green(color))
                    .addData("b",  Color.blue(color));
            telemetry.update();
            sleep(40000000);
        }

    }
}