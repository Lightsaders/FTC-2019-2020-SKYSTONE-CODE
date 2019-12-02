package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.util.Log;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
@Autonomous(name = "csauto")
public class testfullauto extends LinearOpMode {
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;
    NormalizedColorSensor colorSensor;
    NormalizedColorSensor colorSensor0;
    private String positionSkystone = "";
    View relativeLayout;
    // REV HD 40:1 Motor Specs
    double COUNTS_PER_MOTOR_REV = 2240;    // using REV HD 40:1
    double DRIVE_GEAR_REDUCTION = 1;    // 20 tooth to 15 tooth
    double WHEEL_DIAMETER_CM = 10.16;     // mecanum wheels
    double TUNING_DRIVE = 1.1;
    double ROBOT_RADIUS_CM = 29;
    double COUNTS_PER_CM_REV = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * TUNING_DRIVE) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;

    public void runOpMode() throws InterruptedException {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());

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
        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

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

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Check the status of the x button on the gamepad
            strafeDriveEncoder(1,99999999,"LEFT");
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
            straightDriveEncoder(.6,-55);
            strafeDriveEncoder(.6,115,"LEFT");

            telemetry.addLine("normalized color:  ")
                    .addData("a", Color.alpha(color))
                    .addData("r", Color.red(color))
                    .addData("g", Color.green(color))
                    .addData("b", Color.blue(color));

            telemetry.addLine("normalized color0:  ")
                    .addData("a", Color.alpha(color0))
                    .addData("r", Color.red(color0))
                    .addData("g", Color.green(color0))
                    .addData("b", Color.blue(color0));
            telemetry.update();
            if (Color.alpha(color) < 30 && opModeIsActive() && !isStopRequested()) {
                telemetry.addLine("SKYSTONEMIDDLE");
                positionSkystone ="CENTER";
                colors = colorSensor.getNormalizedColors();
                color = colors.toColor();
                max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
                colors.red /= max;
                colors.green /= max;
                colors.blue /= max;
                color = colors.toColor();

                telemetry.addLine("IN LOOP")
                        .addData("a", Color.alpha(color))
                        .addData("r", Color.red(color))
                        .addData("g", Color.green(color))
                        .addData("b", Color.blue(color));
                telemetry.update();
            } else if (Color.red(color0) < 180 && opModeIsActive() && !isStopRequested()) {
                telemetry.addLine(" SKYSTONE WALL");
                positionSkystone = "LEFT";
                colors0 = colorSensor0.getNormalizedColors();
                color0 = colors0.toColor();
                max0 = Math.max(Math.max(Math.max(colors0.red, colors0.green), colors0.blue), colors0.alpha);
                colors0.red /= max0;
                colors0.green /= max0;
                colors0.blue /= max0;
                color0 = colors.toColor();

                telemetry.addLine("IN LOOPb")
                        .addData("a", Color.alpha(color0))
                        .addData("r", Color.red(color0))
                        .addData("g", Color.green(color0))
                        .addData("b", Color.blue(color0));
                telemetry.update();
            } else {
                telemetry.addLine(" SKYSTONE FAR");
                positionSkystone = "RIGHT";
            }
            if (!isStopRequested() && opModeIsActive())
                switch (positionSkystone) {
                    case "RIGHT":
                        if (!isStopRequested() && opModeIsActive()) {
                            strafeDriveEncoder(.6, 85, "LEFT");

                            sleep(1500);
                            straightDriveEncoder(.6, 150);

                            sleep(1400);
                            straightDriveEncoder(1, -147);
                            strafeDriveEncoder(1, 249, "LEFT");
                            straightDriveEncoder(1, 40);

                            sleep(1500);
                            straightDriveEncoder(1, -40);
                            strafeDriveEncoder(1, 59, "RIGHT");
                            straightDriveEncoder(1, 25);


                            break;

                        }


                    case "CENTER":
                        if (!isStopRequested() && opModeIsActive()) {
                            strafeDriveEncoder(1, 14, "LEFT");

                            sleep(1000);
                            straightDriveEncoder(.7, 153);
                            sleep(1000);

                            sleep(2000);
                            straightDriveEncoder(.6, -78);
                            strafeDriveEncoder(.6, 343, "LEFT");

                            sleep(1500);
                            straightDriveEncoder(1, 40);
                            straightDriveEncoder(1, -40);
                            strafeDriveEncoder(1, 69, "RIGHT");
                            straightDriveEncoder(1, 25);





                            break;
                        }
                        // straightDriveEncoder(.6,80);
                        //  straightDriveEncoder(.6,-78);
                        //    strafeDriveEncoder(.6,152,"LEFT");
                        break;
                    case "LEFT":
                        if (!isStopRequested() && opModeIsActive()) {
                            strafeDriveEncoder(1, 53, "LEFT");

                            sleep(1500);
                            straightDriveEncoder(.6, 153);
                            sleep(1000);

                            sleep(2000);
                            straightDriveEncoder(.6, -78);
                            strafeDriveEncoder(.6, 303, "LEFT");

                            sleep(1500);
                            straightDriveEncoder(1, 40);
                            straightDriveEncoder(1, -40);
                            strafeDriveEncoder(1, 79, "RIGHT");
                            straightDriveEncoder(1, 35);




                            break;
                        }
                        break;
                    default:

                        break;

                }


        }
    }
        public void straightDriveEncoder ( double speed, double distanceCM){
            int frontLeftTarget;
            int backLeftTarget;
            int frontRightTarget;
            int backRightTarget;
            double end = 0;
            double t = 0;

            if (opModeIsActive()) {

                driveFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
                driveFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
                driveBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
                driveBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

                // Determine new target position, and pass to motor controller
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_REV);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_REV);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_REV);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_REV);

                // set target position to each motor
                driveFrontLeft.setTargetPosition(frontLeftTarget);
                driveFrontRight.setTargetPosition(frontRightTarget);
                driveBackLeft.setTargetPosition(backLeftTarget);
                driveBackRight.setTargetPosition(backRightTarget);

                // Turn on run to position
                driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                driveFrontLeft.setPower(Math.abs(speed));
                driveFrontRight.setPower(Math.abs(speed));
                driveBackLeft.setPower(Math.abs(speed));
                driveBackRight.setPower(Math.abs(speed));

                t = getRuntime();
                end = (Math.abs(distanceCM) / 10.16) / (speed / 0.1) + getRuntime();


                while (opModeIsActive() && !isStopRequested() &&
                        (getRuntime() <= end) &&
                        (driveFrontLeft.isBusy() || driveFrontRight.isBusy() || driveBackLeft.isBusy() || driveBackRight.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("RUN TIME CURRENT: ", "" + getRuntime());
                    telemetry.addData("RUN TIME END: ", "" + end);
                    telemetry.addData("FRONT LEFT MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", frontLeftTarget, driveFrontLeft.getCurrentPosition());
                    telemetry.addData("FRONT RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", frontRightTarget, driveFrontRight.getCurrentPosition());
                    telemetry.addData("BACK LEFT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backLeftTarget, driveBackLeft.getCurrentPosition());
                    telemetry.addData("BACK RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backRightTarget, driveBackRight.getCurrentPosition());
                    telemetry.update();
                }

                telemetry.clearAll();
                telemetry.addData("FINISHED RUN: ", "" + (end - t));
                telemetry.update();

                // Stop all motion;
                driveFrontLeft.setPower(0);
                driveFrontRight.setPower(0);
                driveBackLeft.setPower(0);
                driveBackRight.setPower(0);

                //Turn off run to position
                driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        }

        public void strafeDriveEncoder ( double speed, double distance, String direction){
            int frontLeftTarget = 0;
            int backLeftTarget = 0;
            int frontRightTarget = 0;
            int backRightTarget = 0;
            double end = 0;
            double t = 0;

            switch (direction) {
                case "LEFT":
                    // Determine new target position, and pass to motor controller
                    frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -1.6);
                    frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -1.6);
                    backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -1.6);
                    backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -1.6);
                    break;
                case "RIGHT":
                    // Determine new target position, and pass to motor controller
                    frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * 1.6);
                    frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * 1.6);
                    backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * 1.6);
                    backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * 1.6);
                    break;
            }
            if (opModeIsActive()) {

                // set target position to each motor
                driveFrontLeft.setTargetPosition(frontLeftTarget);
                driveFrontRight.setTargetPosition(frontRightTarget);
                driveBackLeft.setTargetPosition(backLeftTarget);
                driveBackRight.setTargetPosition(backRightTarget);

                // Turn on run to position
                driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                driveFrontLeft.setPower(Math.abs(speed));
                driveFrontRight.setPower(Math.abs(speed));
                driveBackLeft.setPower(Math.abs(speed));
                driveBackRight.setPower(Math.abs(speed));

                t = getRuntime();
                end = (Math.abs(distance) / 15.54) / (speed / 0.7) + getRuntime();

                while (opModeIsActive() && !isStopRequested() &&
                        (getRuntime() <= end) &&
                        (driveFrontLeft.isBusy() || driveFrontRight.isBusy() || driveBackLeft.isBusy() || driveBackRight.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("RUN TIME CURRENT: ", "" + getRuntime());
                    telemetry.addData("RUN TIME END: ", "" + end);
                    telemetry.addData("FRONT LEFT MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", frontLeftTarget, driveFrontLeft.getCurrentPosition());
                    telemetry.addData("FRONT RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", frontRightTarget, driveFrontRight.getCurrentPosition());
                    telemetry.addData("BACK LEFT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backLeftTarget, driveBackLeft.getCurrentPosition());
                    telemetry.addData("BACK RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backRightTarget, driveBackRight.getCurrentPosition());
                    telemetry.update();
                }

                telemetry.clearAll();
                telemetry.addData("FINISHED RUN: ", "" + (end - t));
                telemetry.update();

                // Stop all motion;
                driveFrontLeft.setPower(0);
                driveFrontRight.setPower(0);
                driveBackLeft.setPower(0);
                driveBackRight.setPower(0);

                //Turn off run to position
                driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }

        }

        public void turnEncoder ( double speed, double turnDegrees, String direction){
            double tuning = 1.1;
            double distance = ROBOT_RADIUS_CM * tuning * (((turnDegrees) * (Math.PI)) / (180)); // Using arc length formula
            int frontLeftTarget = 0;
            int backLeftTarget = 0;
            int frontRightTarget = 0;
            int backRightTarget = 0;
            double end = 0;
            double t = 0;

            //RESET ENCODERS
            driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            switch (direction) {
                case "C":
                    // Determine new target position, and pass to motor controller
                    frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                    frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                    backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                    backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                    break;
                case "CC":
                    // Determine new target position, and pass to motor controller
                    frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                    frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                    backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                    backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                    break;
            }

            if (opModeIsActive()) {

                // set target position to each motor
                driveFrontLeft.setTargetPosition(frontLeftTarget);
                driveFrontRight.setTargetPosition(frontRightTarget);
                driveBackLeft.setTargetPosition(backLeftTarget);
                driveBackRight.setTargetPosition(backRightTarget);

                // Turn on run to position
                driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                driveFrontLeft.setPower(speed);
                driveFrontRight.setPower(speed);
                driveBackLeft.setPower(speed);
                driveBackRight.setPower(speed);

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.

                t = getRuntime();
                end = (Math.abs(turnDegrees) / 10.16) / (speed / 0.05) + getRuntime();

                while (opModeIsActive() &&
                        (getRuntime() <= end) &&
                        (driveFrontLeft.isBusy() || driveFrontRight.isBusy() || driveBackLeft.isBusy() || driveBackRight.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("RUN TIME CURRENT: ", "" + getRuntime());
                    telemetry.addData("RUN TIME END: ", "" + end);
                    telemetry.addData("FRONT LEFT MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", frontLeftTarget, driveFrontLeft.getCurrentPosition());
                    telemetry.addData("FRONT RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", frontRightTarget, driveFrontRight.getCurrentPosition());
                    telemetry.addData("BACK LEFT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backLeftTarget, driveBackLeft.getCurrentPosition());
                    telemetry.addData("BACK RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backRightTarget, driveBackRight.getCurrentPosition());
                    telemetry.update();
                }
                telemetry.clearAll();
                telemetry.addData("FINISHED RUN: ", "" + (end - t));
                telemetry.update();

                // Stop all motion;
                driveFrontLeft.setPower(0);
                driveFrontRight.setPower(0);
                driveBackLeft.setPower(0);
                driveBackRight.setPower(0);

                //Turn off run to position
                driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            //telemetrySender("DEGREES CURRENT: ", "" + getCurrentHeading(), "");
            //telemetrySender("DEGREES FINAL: ", "" + (getCurrentHeading() + headingStart), "");
        }

}