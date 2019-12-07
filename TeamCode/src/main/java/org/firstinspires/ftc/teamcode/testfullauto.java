package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.util.Log;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    private Servo Rotation;
    private Servo clamp;
    private  DcMotor intake;
    private DcMotor liftleft;
    private DcMotor liftright;
    private DcMotor actuator;
    // REV HD 40:1 Motor Specs
    double COUNTS_PER_MOTOR_REV = 537.5;    // using REV HD 40:1
    double DRIVE_GEAR_REDUCTION = 1;    // 20 tooth to 15 tooth
    double WHEEL_DIAMETER_CM = 11;     // mecanum wheels
    double TUNING_DRIVE = 1.1;
    double ROBOT_RADIUS_CM = 29;
    double COUNTS_PER_CM_REV = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * TUNING_DRIVE) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;
    private DistanceSensor sensorRange;
    private Servo servo;
    public void runOpMode() throws InterruptedException {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor0 = hardwareMap.get(NormalizedColorSensor.class, "colorb");
        servo = hardwareMap.servo.get("servo");
        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight) colorSensor).enableLight(true);
//        }
//        if (colorSensor0 instanceof SwitchableLight) {
//            ((SwitchableLight) colorSensor0).enableLight(true);
//        }


            driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
            driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
            driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
            driveBackRight = hardwareMap.dcMotor.get("driveBackRight");
            Rotation = hardwareMap.servo.get("Rotation");
            actuator = hardwareMap.dcMotor.get("actuator");
            clamp = hardwareMap.servo.get("clamp");
            intake = hardwareMap.dcMotor.get("intake");
            liftright = hardwareMap.dcMotor.get("liftright");
            liftleft = hardwareMap.dcMotor.get("liftleft");
            driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
            driveBackRight.setDirection(DcMotor.Direction.REVERSE);
            liftleft.setDirection(DcMotor.Direction.REVERSE);
            sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        // bPrevState and bCurrState keep track of the previous and current state of the button
        boolean bPrevState = false;
        boolean bCurrState = false;

        // Get a reference to our sensor object.


        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // Check the status of the x button on the gamepad

            // Read the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            NormalizedRGBA colors0 = colorSensor0.getNormalizedColors();
            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

            straightDriveEncoder(.6, -23);
            sleep(1000);

//            telemetry.addData("deviceName", sensorRange.getDeviceName());
//            telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
//            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
//            telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
//            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
//
//            // Rev2mDistanceSensor specific methods.
//            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
//            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
//
//            telemetry.update();
//
//
//            int color = colors.toColor();
//            int color0 = colors0.toColor();
//
//            // Balance the colors. The values returned by getColors() are normalized relative to the
//            // maximum possible values that the sensor can measure. For example, a sensor might in a
//            // particular configuration be able to internally measure color intensity in a range of
//            // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
//            // so as to return a value it the range [0,1]. However, and this is the point, even so, the
//            // values we see here may not get close to 1.0 in, e.g., low light conditions where the
//            // sensor measurements don't approach their maximum limit. In such situations, the *relative*
//            // intensities of the colors are likely what is most interesting. Here, for example, we boost
//            // the signal on the colors while maintaining their relative balance so as to give more
//            // vibrant visual feedback on the robot controller visual display.
//            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
//            colors.red /= max;
//            colors.green /= max;
//            colors.blue /= max;
//            color = colors.toColor();
//            float max0 = Math.max(Math.max(Math.max(colors0.red, colors0.green), colors0.blue), colors0.alpha);
//            colors0.red /= max0;
//            colors0.green /= max0;
//            colors0.blue /= max0;
//            color0 = colors0.toColor();
//
//
//            telemetry.addLine("normalized color:  ")
//                    .addData("a", Color.alpha(color))
//                    .addData("r", Color.red(color))
//                    .addData("g", Color.green(color))
//                    .addData("b", Color.blue(color));
//
//            telemetry.addLine("normalized color0:  ")
//                    .addData("a", Color.alpha(color0))
//                    .addData("r", Color.red(color0))
//                    .addData("g", Color.green(color0))
//                    .addData("b", Color.blue(color0));
//            telemetry.update();
//            sleep(2000);
//            while (!isStopRequested() && opModeIsActive()  ) {
//                if (Color.red(color) < 100 && opModeIsActive() && !isStopRequested()) {
//                    telemetry.addLine("SKYSTONEWall");
//                    positionSkystone = "LEFT";
//                    colors = colorSensor.getNormalizedColors();
//                    color = colors.toColor();
//                    max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
//                    colors.red /= max;
//                    colors.green /= max;
//                    colors.blue /= max;
//                    color = colors.toColor();
//
//                    telemetry.addLine("IN LOOP")
//                            .addData("a", Color.alpha(color))
//                            .addData("r", Color.red(color))
//                            .addData("g", Color.green(color))
//                            .addData("b", Color.blue(color));
//                    telemetry.update();
//                    sleep(1000);
//                    if (!isStopRequested() && opModeIsActive()) {
////                            strafeDriveEncoder(1, 53, "LEFT");
//                        telemetry.addLine(" SKYSTONE WALL");
//                        sleep(1000);
//
//                       turnEncoder(.6,90,"CC");
//                       sleep(1000);
//                        clamp.setPosition(.68);
//                        liftleft.setPower(1);
//                        liftright.setPower(1);
//                        sleep(1000);
//                        liftleft.setPower(0);
//                        liftright.setPower(0);
//                        straightDriveEncoder(.6,34);
//                        strafeDriveEncoder(.6,250,"LEFT");
//                        clamp.setPosition(1);
//                        strafeDriveEncoder(.6,67,"RIGHT");
////                            sleep(1500);
////                            straightDriveEncoder(.6, 153);
////                            sleep(1000);
////
////                            sleep(2000);
////                            straightDriveEncoder(.6, -78);
////                            strafeDriveEncoder(.6, 303, "LEFT");
////
////                            sleep(1500);
////                            straightDriveEncoder(1, 40);
////                            straightDriveEncoder(1, -40);
////                            strafeDriveEncoder(1, 79, "RIGHT");
////                            straightDriveEncoder(1, 35);
////
////
////
//
//                        break;
//                    }
//
//
//                } else if (Color.red(color0) < 140 && opModeIsActive() && !isStopRequested()) {
//                    telemetry.addLine(" SKYSTONE MIDDLE");
//                    positionSkystone = "CENTER";
//                    colors0 = colorSensor0.getNormalizedColors();
//                    color0 = colors0.toColor();
//                    max0 = Math.max(Math.max(Math.max(colors0.red, colors0.green), colors0.blue), colors0.alpha);
//                    colors0.red /= max0;
//                    colors0.green /= max0;
//                    colors0.blue /= max0;
//                    color0 = colors.toColor();
//
//                    telemetry.addLine("MIDDLE")
//                            .addData("a", Color.alpha(color0))
//                            .addData("r", Color.red(color0))
//                            .addData("g", Color.green(color0))
//                            .addData("b", Color.blue(color0));
//                    telemetry.update();
//                    sleep(1000);
//                    if (!isStopRequested() && opModeIsActive()) {
////                            strafeDriveEncoder(1, 14, "LEFT");
//                        telemetry.addLine(" SKYSTONE MIDDLE");
//                        sleep(1000);
//                        straightDriveEncoder(.5, 10);
//                        servo.setPosition(1);
////                            sleep(1000);
////                            straightDriveEncoder(.7, 153);
////                            sleep(1000);
////
////                            sleep(2000);
////                            straightDriveEncoder(.6, -78);
////                            strafeDriveEncoder(.6, 343, "LEFT");
////
////                            sleep(1500);
////                            straightDriveEncoder(1, 40);
////                            straightDriveEncoder(1, -40);
////                            strafeDriveEncoder(1, 69, "RIGHT");
////                            straightDriveEncoder(1, 25);
////
////
////
////
//
//                        break;
//                    }
//
//                } else {
//                    telemetry.addLine(" SKYSTONE FAR");
//                    positionSkystone = "RIGHT";
//                    if (!isStopRequested() && opModeIsActive()) {
//                        telemetry.addLine(" SKYSTONE FAR");
//                        sleep(1000);
//                        straightDriveEncoder(.5, 18);
//                        servo.setPosition(.9);
//                        strafeDriveEncoder(.6, 15, "RIGHT");
//
////
//                        sleep(1500);
//                        straightDriveEncoder(.6, 150);
////
////                            sleep(1400);
////                            straightDriveEncoder(1, -147);
////                            strafeDriveEncoder(1, 249, "LEFT");
////                            straightDriveEncoder(1, 40);
////
////                            sleep(1500);
////                            straightDriveEncoder(1, -40);
////                            strafeDriveEncoder(1, 59, "RIGHT");
////                            straightDriveEncoder(1, 25);
////
//
//                        break;
//
//                    }
//
//                }
//
//                idle();
//                }
            idle();
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
                    frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -.32);
                    frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * .32);
                    backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * .32);
                    backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -.32);
                    break;
                case "RIGHT":
                    // Determine new target position, and pass to motor controller
                    frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * .32);
                    frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -.32);
                    backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * -.32);
                    backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV * .32);
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
                end = (Math.abs(distance) / 26.54) / (speed / 0.7) + getRuntime();

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