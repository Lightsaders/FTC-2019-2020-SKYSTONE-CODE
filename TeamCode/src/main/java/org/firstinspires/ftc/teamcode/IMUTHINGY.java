package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@Autonomous(name = "IMU THINGY")

public class IMUTHINGY extends LinearOpMode {

    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;

    public BNO055IMU IMU = null;

    public static final double HEADING_THRESHOLD = 1 ;
    public static final double P_TURN_COEFF = 0.1;
    static final double P_DRIVE_COEFF = 0.15;

    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftBackMotor = hardwareMap.get(DcMotor.class, "lbm");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rbm");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "lfm");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rfm");

        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);

// Start Init IMU
        BNO055IMU.Parameters Bparameters = new BNO055IMU.Parameters();
        Bparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Bparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        Bparameters.loggingEnabled = true;
        Bparameters.loggingTag = "IMU";
        Bparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(Bparameters);
// End Init IMU
        telemetry.addData("IMU Init'D", true);
        telemetry.update();

        waitForStart();

// Turn Left
        gyroTurn(0.3, -90);
    }

    public void gyroTurn(double speed, double angle) {
// keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
// Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;
        double PosOrNeg = 1;
        double SpeedError;
        double error = getError(angle);

// determine turn power based on +/- error
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0;
            rightSpeed = 0;
            onTarget = true;
        } else {
            PosOrNeg = Range.clip((int) error, -0.3, 0.3);
            steer = getSteer(error, PCoeff);
            leftSpeed = Range.clip(speed + Math.abs(error / 150), speed, .5) * PosOrNeg;

            rightSpeed = -leftSpeed;
        }

// Set motor speeds.
        leftBackMotor.setPower(leftSpeed);
        rightBackMotor.setPower(rightSpeed);
        leftFrontMotor.setPower(leftSpeed);
        rightFrontMotor.setPower(rightSpeed);

// Display debug info in telemetry.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Left,Right.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;
// calculate error in -179 to +180 range (
        robotError = targetAngle - getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * (2 * PCoeff), -1, 1);
    }

    public double getHeading() {
        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;

    }
    public void driveForward(double power){
        leftBackMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
    }
    public void driveBackwards(double power){
        leftBackMotor.setPower(-power);
        leftFrontMotor.setPower(-power);
        rightBackMotor.setPower(-power);
        rightBackMotor.setPower(-power);
    }
}