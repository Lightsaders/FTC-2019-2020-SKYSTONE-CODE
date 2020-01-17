package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.sax.TextElementListener;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

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
    double TUNING_DRIVE = 1.22;
    double ROBOT_RADIUS_CM = 29;
    double COUNTS_PER_CM_GOBUILDA = ((COUNTS_PER_MOTOR_GOBUILDA * DRIVE_GEAR_REDUCTION * TUNING_DRIVE) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;

    static final double P_DRIVE_COEFF = 0.04;     // Larger is more responsive, but also less stable
    static final double P_STRAFE_COEFF = 0.02;     // Larger is more responsive, but also less stable
    private double previousHeading = 0; //Outside of method
    private double integratedHeading = 0;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //TODO make timing equation as backup for drive methods

    public void initialize() {

        // Init Drive Motors
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
        rightFoundation = hardwareMap.servo.get("rightFoundation");
        leftFoundation = hardwareMap.servo.get("leftFoundation");

        // Init Motors
        actuator = hardwareMap.dcMotor.get("actuator");
        liftright = hardwareMap.dcMotor.get("liftright");
        liftleft = hardwareMap.dcMotor.get("liftleft");

        // Init Limit Switch
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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
        // Setting Zero Behavior for Drive Train Motors
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    }// TODO add imu correction and odometry wheels

    // Drives the robot left or right for a given speed and distance according to the encoder
    public void strafeDriveEncoder(double speed, double distanceCM, String direction, double custom) {
        int frontLeftTarget = 0;
        int backLeftTarget = 0;
        int frontRightTarget = 0;
        int backRightTarget = 0;
        double end = 0;
        double t = 0;
        double distancePerRotation = 1;// TODO this needs to be found
        // Setting Zero Behavior for Drive Train Motors
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    }// TODO add imu correction and odometry wheels

    // Turns the robot clockwise(c) or counter-clockwise(cc) for a given speed and degree according to the encoder
    public void turnEncoder(double speed, double turnDegrees, String direction, double custom) {
        double tuning = 1.46;
        double distance = ROBOT_RADIUS_CM * tuning * (((turnDegrees) * (Math.PI)) / (180)); // Using arc length formula
        int frontLeftTarget = 0;
        int backLeftTarget = 0;
        int frontRightTarget = 0;
        int backRightTarget = 0;
        double end = 0;
        double t = 0;

        // Setting Zero Behavior for Drive Train Motors
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    }// TODO use imu instead of encoders

    public void clamp(String position, int sleep) {
        switch (position) {
            case "OPEN":
                clamp.setPosition(.8);
                break;
            case "CLOSE":
                clamp.setPosition(.25);
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
                rightFoundation.setPosition(.9);
                break;
        }
        sleep(sleep);// This is to allow time for the servo to move
    }

    public void turnClamp(String state, int sleep) {
        switch (state) {
            case "PERP":
                rotation.setPosition(.98);
                break;
            case "PAR":
                rotation.setPosition(.64);
                break;
        }
        sleep(sleep);// This is to allow time for the servo to move
    }

    public void actuatorDistance(int distanceCM, double speed) {
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
                    (actuator.isBusy())) {

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
    }// TODO get measurement

    public void liftDistance(int distanceCM, double speed) {
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
            rightLiftTarget = liftright.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM_GOBUILDA * tune);

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
    }// TODO odometry pulley

    @Override
    public void runOpMode() throws InterruptedException {

    }

    // TODO IMU calibration method

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int frontLeftTarget;
        int frontRightTarget;
        int backLeftTarget;
        int backRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Setting Zero Behavior for Drive Train Motors
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_CM_GOBUILDA);
            frontLeftTarget = driveFrontLeft.getCurrentPosition() + moveCounts;
            frontRightTarget = driveFrontRight.getCurrentPosition() + moveCounts;
            backLeftTarget = driveBackLeft.getCurrentPosition() + moveCounts;
            backRightTarget = driveBackRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            driveFrontLeft.setTargetPosition(frontLeftTarget);
            driveFrontRight.setTargetPosition(frontRightTarget);
            driveBackLeft.setTargetPosition(backLeftTarget);
            driveBackRight.setTargetPosition(backRightTarget);

            driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            driveFrontLeft.setPower(speed);
            driveFrontRight.setPower(speed);
            driveBackLeft.setPower(speed);
            driveBackRight.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (driveFrontLeft.isBusy() && driveFrontRight.isBusy() && driveBackLeft.isBusy() && driveBackRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                driveFrontLeft.setPower(leftSpeed);
                driveBackLeft.setPower(leftSpeed);
                driveFrontRight.setPower(rightSpeed);
                driveBackRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d:%7d:%7d", frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
                telemetry.addData("Actual", "%7d:%7d:%7d:%7d", driveFrontLeft.getCurrentPosition(),
                        driveFrontRight.getCurrentPosition(), driveBackLeft.getCurrentPosition(), driveBackRight.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    Double getHeading() {
        double heading = 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = formatAngle(angles.angleUnit, angles.firstAngle);
        return heading;
    }

    Double formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    Double formatDegrees(double degrees) {
        return AngleUnit.DEGREES.normalize(degrees);
    }

    public void gyroStrafe(double speed,
                           double distance,
                           double angle,
                           String direction) {

        int frontLeftTarget;
        int frontRightTarget;
        int backLeftTarget;
        int backRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double tune;
        tune = 1.22;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_CM_GOBUILDA * tune);
            frontLeftTarget = driveFrontLeft.getCurrentPosition() + moveCounts;
            frontRightTarget = driveFrontRight.getCurrentPosition() + moveCounts;
            backLeftTarget = driveBackLeft.getCurrentPosition() + moveCounts;
            backRightTarget = driveBackRight.getCurrentPosition() + moveCounts;


            switch (direction) {
                case "LEFT":
                    // Set Target and Turn On RUN_TO_POSITION
                    driveFrontLeft.setTargetPosition(frontLeftTarget * -1);
                    driveFrontRight.setTargetPosition(frontRightTarget);
                    driveBackLeft.setTargetPosition(backLeftTarget);
                    driveBackRight.setTargetPosition(backRightTarget * -1);

                    driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // start motion.
                    speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                    driveFrontLeft.setPower(speed * -1);
                    driveFrontRight.setPower(speed);
                    driveBackLeft.setPower(speed);
                    driveBackRight.setPower(speed * -1);

                    // keep looping while we are still active, and BOTH motors are running.
                    while (opModeIsActive() &&
                            (driveFrontLeft.isBusy() && driveFrontRight.isBusy() && driveBackLeft.isBusy() && driveBackRight.isBusy())) {

                        // adjust relative speed based on heading error.
                        error = getError(angle);
                        steer = getSteer(error, P_DRIVE_COEFF);

                        leftSpeed = speed + steer;
                        rightSpeed = -1 * speed - steer;

                        // Normalize speeds if either one exceeds +/- 1.0;
                        max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                        if (max > 1.0) {
                            leftSpeed /= max;
                            rightSpeed /= max;
                        }

                        driveFrontLeft.setPower(rightSpeed);
                        driveBackLeft.setPower(leftSpeed);
                        driveFrontRight.setPower(leftSpeed);
                        driveBackRight.setPower(rightSpeed);

                        // Display drive status for the driver.
                        telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                        telemetry.addData("Target", "%7d:%7d:%7d:%7d", frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
                        telemetry.addData("Actual", "%7d:%7d:%7d:%7d", driveFrontLeft.getCurrentPosition(),
                                driveFrontRight.getCurrentPosition(), driveBackLeft.getCurrentPosition(), driveBackRight.getCurrentPosition());
                        telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                        telemetry.update();
                    }
                    break;
                case "RIGHT":
                    // Set Target and Turn On RUN_TO_POSITION
                    driveFrontLeft.setTargetPosition(frontLeftTarget);
                    driveFrontRight.setTargetPosition(frontRightTarget * -1);
                    driveBackLeft.setTargetPosition(backLeftTarget * -1);
                    driveBackRight.setTargetPosition(backRightTarget);

                    driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // start motion.
                    speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                    driveFrontLeft.setPower(speed);
                    driveFrontRight.setPower(speed * -1);
                    driveBackLeft.setPower(speed * -1);
                    driveBackRight.setPower(speed);

                    // keep looping while we are still active, and BOTH motors are running.
                    while (opModeIsActive() &&
                            (driveFrontLeft.isBusy() && driveFrontRight.isBusy() && driveBackLeft.isBusy() && driveBackRight.isBusy())) {

                        // adjust relative speed based on heading error.
                        error = getError(angle);
                        steer = getSteer(error, P_DRIVE_COEFF);

                        rightSpeed = speed - steer;
                        leftSpeed = -1 * speed + steer;

                        // Normalize speeds if either one exceeds +/- 1.0;
                        max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                        if (max > 1.0) {
                            leftSpeed /= max;
                            rightSpeed /= max;
                        }

                        driveFrontLeft.setPower(leftSpeed);
                        driveBackLeft.setPower(rightSpeed);
                        driveFrontRight.setPower(rightSpeed);
                        driveBackRight.setPower(leftSpeed);

                        // Display drive status for the driver.
                        telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                        telemetry.addData("Target", "%7d:%7d:%7d:%7d", frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
                        telemetry.addData("Actual", "%7d:%7d:%7d:%7d", driveFrontLeft.getCurrentPosition(),
                                driveFrontRight.getCurrentPosition(), driveBackLeft.getCurrentPosition(), driveBackRight.getCurrentPosition());
                        telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                        telemetry.update();
                    }
                    break;
            }

            // Stop all motion;
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn(double speed,
                         double angle,
                         String direction) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            switch (direction) {
                case "C":
                    driveFrontLeft.setPower(speed);
                    driveFrontRight.setPower(speed * -1);
                    driveBackLeft.setPower(speed);
                    driveBackRight.setPower(speed * -1);

                    while (opModeIsActive() && !isStopRequested() &&
                            getIntegratedHeading() <= angle) {

                    }

                    break;
                case "CC":
                    driveFrontLeft.setPower(speed*-1);
                    driveFrontRight.setPower(speed);
                    driveBackLeft.setPower(speed*-1);
                    driveBackRight.setPower(speed);

                    while (opModeIsActive() && !isStopRequested() &&
                            getIntegratedHeading() >= angle) {

                    }
                    break;
            }
        }
        // Stop all motion;
        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);
    }


    private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }
}
