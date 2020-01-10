package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.sax.TextElementListener;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class Auto_Methods extends LinearOpMode {
    // Drive Motors
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
    public Servo rightFoundation;
    public Servo leftFoundation;

    // Motors
    public DcMotor liftleft;
    public DcMotor liftright;
    public DcMotor actuator;

    // Limit Switch
    public DigitalChannel limitSwitch;

    // Gobilda Motor Specs
    double COUNTS_PER_MOTOR_GOBUILDA = 537.5;    // gobilda
    double DRIVE_GEAR_REDUCTION = 1;    // 1:1
    double WHEEL_DIAMETER_CM = 10;     // mecanum wheels
    double TUNING_DRIVE = 1.1;
    double ROBOT_RADIUS_CM = 29;
    double COUNTS_PER_CM_GOBUILDA = ((COUNTS_PER_MOTOR_GOBUILDA * DRIVE_GEAR_REDUCTION * TUNING_DRIVE) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;
    int rpm = 435;
    double resistance = 0.9;// TODO need to change

    public void initialize() {

        // Init Drive Motors
        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        // Reverse Drive Motors
        driveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Setting Zero Behavior for Drive Train Motors
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init Color Sensors
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "colorSensorRight");

        // Init Servos
        rotation = hardwareMap.servo.get("rotation");
        clamp = hardwareMap.servo.get("clamp");
        rightFoundation = hardwareMap.servo.get("rightFoundation");
        leftFoundation = hardwareMap.servo.get("leftFoundation");

        // Init Motors
        actuator = hardwareMap.dcMotor.get("actuator");
        liftright = hardwareMap.dcMotor.get("liftright");
        liftleft = hardwareMap.dcMotor.get("liftleft");

        // Init Limit Switch
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    // This sets positionSkystone according to the colorSensors
    public void skystoneColorScan(String color) {
        // Check the status of the x button on the gamepad
        NormalizedRGBA colorsLeft = colorSensorLeft.getNormalizedColors();
        NormalizedRGBA colorsRight = colorSensorRight.getNormalizedColors();

        float[] hsvValuesLeft = new float[3];
        final float valuesLeft[] = hsvValuesLeft;
        float[] hsvValuesRight = new float[3];
        final float valuesRight[] = hsvValuesRight;

        // Settings Colors
        float max = Math.max(Math.max(Math.max(colorsLeft.red, colorsLeft.green), colorsLeft.blue), colorsLeft.alpha);
        colorsLeft.red /= max;
        colorsLeft.green /= max;
        colorsLeft.blue /= max;
        float max0 = Math.max(Math.max(Math.max(colorsRight.red, colorsRight.green), colorsRight.blue), colorsRight.alpha);
        colorsRight.red /= max0;
        colorsRight.green /= max0;
        colorsRight.blue /= max0;

//        while(opModeIsActive() && !isStopRequested()) {
            colorsLeft = colorSensorLeft.getNormalizedColors();
            Color.colorToHSV(colorsLeft.toColor(), hsvValuesLeft);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValuesLeft[0])
                    .addData("S", "%.3f", hsvValuesLeft[1])
                    .addData("V", "%.3f", hsvValuesLeft[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colorsLeft.alpha)
                    .addData("r", "%.3f", colorsLeft.red)
                    .addData("g", "%.3f", colorsLeft.green)
                    .addData("b", "%.3f", colorsLeft.blue);

            colorsRight = colorSensorRight.getNormalizedColors();
            Color.colorToHSV(colorsRight.toColor(), hsvValuesRight);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValuesRight[0])
                    .addData("S", "%.3f", hsvValuesRight[1])
                    .addData("V", "%.3f", hsvValuesRight[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colorsRight.alpha)
                    .addData("r", "%.3f", colorsRight.red)
                    .addData("g", "%.3f", colorsRight.green)
                    .addData("b", "%.3f", colorsRight.blue);

            telemetry.update();
//        }

        // Scanning Loop
        if (!isStopRequested() && opModeIsActive()) {
            colorsLeft = colorSensorLeft.getNormalizedColors();
            Color.colorToHSV(colorsLeft.toColor(), hsvValuesLeft);
            colorsRight = colorSensorRight.getNormalizedColors();
            Color.colorToHSV(colorsRight.toColor(), hsvValuesRight);
            if (hsvValuesLeft[0] >= 60 && hsvValuesRight[0] < 60 && opModeIsActive() && !isStopRequested()) {// If left color sensor is black then skystone is wall
                switch (color) {
                    case "RED":
                        positionSkystone = "WALL";
//                        telemetry.addLine("SKYSTONE WALL");
//                        telemetry.update();
                        break;
                    case "BLUE":
                        positionSkystone = "MIDDLE";
//                        telemetry.addLine("SKYSTONE BRIDGE");
//                        telemetry.update();
                        break;
                }
            } else if (hsvValuesRight[0] >= 60 && hsvValuesLeft[0] < 60 && opModeIsActive() && !isStopRequested()) {// If right color sensor is black then skystone is middle
                switch (color) {
                    case "BLUE":
                        positionSkystone = "WALL";
//                        telemetry.addLine("SKYSTONE WALL");
//                        telemetry.update();
                        break;
                    case "RED":
                        positionSkystone = "MIDDLE";
//                        telemetry.addLine("SKYSTONE BRIDGE");
//                        telemetry.update();
                        break;
                }
//                telemetry.addLine("SKYSTONE MIDDLE");
//                telemetry.update();
            } else {// If neither color sensor is black then by process of elimination it has to be bridge
                switch (color) {
                    case "BLUE":
                        positionSkystone = "BRIDGE";
//                        telemetry.addLine("SKYSTONE WALL");
//                        telemetry.update();
                        break;
                    case "RED":
                        positionSkystone = "BRIDGE";
//                        telemetry.addLine("SKYSTONE MIDDLE");
//                        telemetry.update();
                        break;
                }
            }
        }
    }

    // Drives the robot forward or backward for a given speed and distance according to the encoder
    public void straightDriveEncoder(double speed, double distanceCM, double custom) {
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
            frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA);
            frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA);
            backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA);
            backRightTarget = driveBackRight.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA);

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
//            end = ((Math.abs(distanceCM)/WHEEL_DIAMETER_CM)/(rpm*resistance))*60*(2-speed) + getRuntime();// TODO this needs to be tested
            end = getRuntime() + custom;

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
    }// TODO test this

    // Drives the robot left or right for a given speed and distance according to the encoder
    public void strafeDriveEncoder(double speed, double distanceCM, String direction, int custom) {
        int frontLeftTarget = 0;
        int backLeftTarget = 0;
        int frontRightTarget = 0;
        int backRightTarget = 0;
        double end = 0;
        double t = 0;
        double distancePerRotation = 1;// TODO this needs to be found

        switch (direction) {
            case "LEFT":
                // Determine new target position, and pass to motor controller
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * -1.8);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * 1.8);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * 1.8);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * -1.8);


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

                //TODO NOT A PERMENANT SOLUTION - THIS IS MENT TO ACCOUNT FOR DRIFT IN THE MECCANUM DRIVE
                driveFrontLeft.setPower(Math.abs(speed) * 0.8);
                driveFrontRight.setPower(Math.abs(speed) * 1);
                driveBackLeft.setPower(Math.abs(speed) * 1);
                driveBackRight.setPower(Math.abs(speed) * 0.8);

                break;
            case "RIGHT":
                // Determine new target position, and pass to motor controller
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * 1.8);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * -1.8);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * -1.8);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * 1.8);

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

                //TODO NOT A PERMENANT SOLUTION - THIS IS MENT TO ACCOUNT FOR DRIFT IN THE MECCANUM DRIVE
                driveFrontLeft.setPower(Math.abs(speed) * 1);// Change this if you want the robot to strafe more backwards
                driveFrontRight.setPower(Math.abs(speed) * 1);// Change this if you want the robot to strafe more forwards
                driveBackLeft.setPower(Math.abs(speed) * 1);// Change this if you want the robot to strafe more forwards
                driveBackRight.setPower(Math.abs(speed) * 1);// Change this if you want the robot to strafe more backwards
                break;
        }
        if (opModeIsActive()) {

            t = getRuntime();
//            end = (Math.abs(distanceCM)/distancePerRotation)*60*(2-speed) + getRuntime();// TODO this needs to be tested
            end = custom + getRuntime();

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

    }// TODO test this

    // Turns the robot clockwise(c) or counter-clockwise(cc) for a given speed and degree according to the encoder
    public void turnEncoder(double speed, double turnDegrees, String direction, int custom) {
        double tuning = 1.46;
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
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_GOBUILDA);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_GOBUILDA);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_GOBUILDA);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_GOBUILDA);
                break;
            case "CC":
                // Determine new target position, and pass to motor controller
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_GOBUILDA);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_GOBUILDA);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_GOBUILDA);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_GOBUILDA);
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

            t = getRuntime();
//            end = ((Math.abs(distance)/WHEEL_DIAMETER_CM)/(rpm*resistance))*60*(2-speed) + getRuntime();// TODO this needs to be tested
            end = getRuntime() + custom;

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
    }// TODO test this

    public void clamp(String position, int sleep) {
        switch (position) {
            case "OPEN":
                clamp.setPosition(1);
                break;
            case "CLOSE":
                clamp.setPosition(.5);
                break;
            case "SEMI OPEN":
                clamp.setPosition(.8);
                break;
        }
        sleep(sleep);// This is to allow time for the servo to move
    }

    public void foundationClamps(String state, int sleep) {
        switch (state) {
            case "DOWN":
                leftFoundation.setPosition(1);
                rightFoundation.setPosition(0.15);
                break;
            case "UP":
                leftFoundation.setPosition(0.2);
                rightFoundation.setPosition(1);
                break;
        }
        sleep(sleep);// This is to allow time for the servo to move
    }// TODO test this

    public void turnClamp(String state, int sleep) {
        switch (state) {
            case "PERP":
                rotation.setPosition(0.8);
                break;
            case "PAR":
                rotation.setPosition(0.4);
                break;
        }
        sleep(sleep);// This is to allow time for the servo to move
    }

    public void actuatorDistance(int distanceCM, double speed){
        int actuatorTarget;
        double tuner = 1;// TODO change this if distanceCM doesnt correspond to actuator distance
        double end = 0;
        double t = 0;

        if (opModeIsActive()) {

            actuator.setMode(DcMotor.RunMode.RESET_ENCODERS);

            // Determine new target position, and pass to motor controller
            actuatorTarget = actuator.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * tuner);

            // set target position to each motor
            actuator.setTargetPosition(actuatorTarget);

            // Turn on run to position
            actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            actuator.setPower(Math.abs(speed));

            t = getRuntime();
            end = (Math.abs(distanceCM) / 10.16) / (speed / 0.1) + getRuntime();// TODO this needs to be corrected

            while (opModeIsActive() && !isStopRequested() &&
                    (getRuntime() <= end) &&
                    (actuator.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("RUN TIME CURRENT: ", "" + getRuntime());
                telemetry.addData("RUN TIME END: ", "" + end);
                telemetry.addData("ACTUATOR MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", actuatorTarget, actuator.getCurrentPosition());
                telemetry.update();
            }

            telemetry.clearAll();
            telemetry.addData("FINISHED RUN: ", "" + (end - t));
            telemetry.update();

            // Stop all motion;
            actuator.setPower(0);

            //Turn off run to position
            actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }// TODO test this

    public void liftDistance(int distanceCM, double speed){
        int leftLiftTarget;
        int rightLiftTarget;
        double tune = 1;// TODO change this
        double end = 0;
        double t = 0;

        if (opModeIsActive()) {

            liftleft.setMode(DcMotor.RunMode.RESET_ENCODERS);
            liftright.setMode(DcMotor.RunMode.RESET_ENCODERS);

            // Determine new target position, and pass to motor controller
            leftLiftTarget = liftleft.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * tune);
            rightLiftTarget = liftright.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA* tune);

            // set target position to each motor
            liftleft.setTargetPosition(leftLiftTarget);
            liftright.setTargetPosition(rightLiftTarget);

            // Turn on run to position
            liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftleft.setPower(Math.abs(speed));
            liftright.setPower(Math.abs(speed));

            t = getRuntime();
            end = (Math.abs(distanceCM) / 10.16) / (speed / 0.1) + getRuntime();// TODO this needs to be corrected

            while (opModeIsActive() && !isStopRequested() &&
                    (getRuntime() <= end) &&
                    (liftleft.isBusy() || liftright.isBusy())) {

                // Display it for the driver.
                telemetry.addData("RUN TIME CURRENT: ", "" + getRuntime());
                telemetry.addData("RUN TIME END: ", "" + end);
                telemetry.addData("LEFT MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", leftLiftTarget, liftleft.getCurrentPosition());
                telemetry.addData("RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", rightLiftTarget, liftright.getCurrentPosition());
                telemetry.update();
            }

            telemetry.clearAll();
            telemetry.addData("FINISHED RUN: ", "" + (end - t));
            telemetry.update();

            // Stop all motion;
            liftleft.setPower(0);
            liftright.setPower(0);

            //Turn off run to position
            liftleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }// TODO test this

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
