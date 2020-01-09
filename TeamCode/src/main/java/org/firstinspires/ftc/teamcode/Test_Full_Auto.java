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

@Autonomous(name = "Test_Full_Auto")
public class Test_Full_Auto extends LinearOpMode {
    public DcMotor driveFrontLeft;
    public DcMotor driveFrontRight;
    public DcMotor driveBackLeft;
    public DcMotor driveBackRight;

    // Color Sensors
    public NormalizedColorSensor colorSensorLeft;
    public NormalizedColorSensor colorSensorRight;

    // Skystone Position
    public String positionSkystone = "";

    // Servos
    public Servo rotation;
    public Servo clamp;

    // Motors
    public DcMotor liftleft;
    public DcMotor liftright;
    public DcMotor actuator;

    // Gobilda Motor Specs
    double COUNTS_PER_MOTOR_REV = 386.3;    // gobilda
    double DRIVE_GEAR_REDUCTION = 1;    // 1:1
    double WHEEL_DIAMETER_CM = 10.16;     // mecanum wheels
    double TUNING_DRIVE = 1.75;
    double ROBOT_RADIUS_CM = 29;
    double COUNTS_PER_CM_REV = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * TUNING_DRIVE) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;



    public void runOpMode() throws InterruptedException {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        // Reverse Drive Motors
        driveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Init Color Sensors
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "colorSensorRight");

        // Init Servos
        rotation = hardwareMap.servo.get("rotation");
        clamp = hardwareMap.servo.get("clamp");

        // Init Motors
        actuator = hardwareMap.dcMotor.get("actuator");
        liftright = hardwareMap.dcMotor.get("liftright");
        liftleft = hardwareMap.dcMotor.get("liftleft");
        // values is a reference to the hsvValues array.

        // Get a reference to our sensor object.


        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
           straightDriveEncoder(.4,30);
           sleep(1000);
           telemetry.addLine("i did not f up");
            // Check the status of the x button on the gamepad
            NormalizedRGBA colorsLeft = colorSensorLeft.getNormalizedColors();
            NormalizedRGBA colorsRight = colorSensorRight.getNormalizedColors();

            // Settings Colors
            float max = Math.max(Math.max(Math.max(colorsLeft.red, colorsLeft.green), colorsLeft.blue), colorsLeft.alpha);
            colorsLeft.red /= max;
            colorsLeft.green /= max;
            colorsLeft.blue /= max;
            float max0 = Math.max(Math.max(Math.max(colorsRight.red, colorsRight.green), colorsRight.blue), colorsRight.alpha);
            colorsRight.red /= max0;
            colorsRight.green /= max0;
            colorsRight.blue /= max0;

            // Scanning Loop
            while (!isStopRequested() && opModeIsActive()) {
                if (colorsLeft.red < 100 && opModeIsActive() && !isStopRequested()) {// If left color sensor is black then skystone is left
                    telemetry.addLine("SKYSTONE LEFT");
                    positionSkystone = "LEFT";
                } else if (colorsRight.red < 140 && opModeIsActive() && !isStopRequested()) {// If right color sensor is black then skystone is middle
                    telemetry.addLine(" SKYSTONE MIDDLE");
                    positionSkystone = "CENTER";
                } else {// If neither color sensor is black then by process of elimination it has to be right
                    telemetry.addLine(" SKYSTONE RIGHT");
                    positionSkystone = "RIGHT";
                }
            }
                switch (positionSkystone) {
                    case "LEFT":
                        if (!isStopRequested() && opModeIsActive()) {
//                            strafeDriveEncoder(1, 53, "LEFT");
                            telemetry.addLine(" SKYSTONE WALL");
                            sleep(1000);

                            turnEncoder(.6, 90, "CC");
                            sleep(1000);
                            clamp.setPosition(.68);
                            liftleft.setPower(1);
                            liftright.setPower(1);
                            sleep(1000);
                            liftleft.setPower(0);
                            liftright.setPower(0);
                            straightDriveEncoder(.6, 34);
                            strafeDriveEncoder(.6, 250, "LEFT");
                            clamp.setPosition(1);
                            strafeDriveEncoder(.6, 67, "RIGHT");
//                            sleep(1500);
//                            straightDriveEncoder(.6, 153);
//                            sleep(1000);
//
//                            sleep(2000);
//                            straightDriveEncoder(.6, -78);
//                            strafeDriveEncoder(.6, 303, "LEFT");
//
//                            sleep(1500);
//                            straightDriveEncoder(1, 40);
//                            straightDriveEncoder(1, -40);
//                            strafeDriveEncoder(1, 79, "RIGHT");
//                            straightDriveEncoder(1, 35);
//
//
//

                            break;
                        }
                    case "CENTER":
                        if (!isStopRequested() && opModeIsActive()) {
//                            strafeDriveEncoder(1, 14, "LEFT");
                            telemetry.addLine(" SKYSTONE MIDDLE");
                            sleep(1000);
                            straightDriveEncoder(.5, 10);

//                            sleep(1000);
//                            straightDriveEncoder(.7, 153);
//                            sleep(1000);
//
//                            sleep(2000);
//                            straightDriveEncoder(.6, -78);
//                            strafeDriveEncoder(.6, 343, "LEFT");
//
//                            sleep(1500);
//                            straightDriveEncoder(1, 40);
//                            straightDriveEncoder(1, -40);
//                            strafeDriveEncoder(1, 69, "RIGHT");
//                            straightDriveEncoder(1, 25);
//
//
//
//

                            break;
                        }
                    case "RIGHT":
                        if (!isStopRequested() && opModeIsActive()) {
                            telemetry.addLine(" SKYSTONE FAR");
                            sleep(1000);
                            straightDriveEncoder(.5, 18);

                            strafeDriveEncoder(.6, 15, "RIGHT");

//
                            sleep(1500);
                            straightDriveEncoder(.6, 150);
//
                            sleep(1400);
                            straightDriveEncoder(1, -147);
                            strafeDriveEncoder(1, 249, "LEFT");
                            straightDriveEncoder(1, 40);

                            sleep(1500);
                            straightDriveEncoder(1, -40);
                            strafeDriveEncoder(1, 59, "RIGHT");
                            straightDriveEncoder(1, 25);


                        }
                        break;
                    default:
                        telemetry.addLine("I screwed up badly");


                }

                idle();
            }
            idle();
        }



    public void straightDriveEncoder(double speed, double distanceCM) {
        int frontLeftTarget;
        int backLeftTarget;
        int frontRightTarget;
        int backRightTarget;
        double end = 0;
        double t = 0;

        if (opModeIsActive()) {

            driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

            // Set Speed
            driveFrontLeft.setPower((Math.abs(speed)));
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
            // Stop all motion;
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

//            telemetry.clearAll();
//            telemetry.addData("FINISHED RUN: ", "" + (end - t));
//            telemetry.update();

            //Turn off run to position
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void strafeDriveEncoder(double speed, double distance, String direction) {
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

    public void turnEncoder(double speed, double turnDegrees, String direction) {
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