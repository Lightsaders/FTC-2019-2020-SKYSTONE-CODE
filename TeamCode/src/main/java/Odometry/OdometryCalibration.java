package Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Auto_Methods;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
@Disabled
public class OdometryCalibration extends LinearOpMode {

    Auto_Methods auto_methods = new Auto_Methods() {
        @Override
        public void initCompBot() {
            super.initCompBot();
        }

        @Override
        public void initPracticeBot() {
            super.initPracticeBot();
        }

        @Override
        public void skystoneColorScan(String color) {
            super.skystoneColorScan(color);
        }

        @Override
        public void straightDriveEncoder(double speed, double distanceCM, double timeCutOff) {
            super.straightDriveEncoder(speed, distanceCM, timeCutOff);
        }

        @Override
        public void strafeDriveEncoder(double speed, double distanceCM, String direction, double timeCutOff) {
            super.strafeDriveEncoder(speed, distanceCM, direction, timeCutOff);
        }

        @Override
        public void turnEncoder(double speed, double turnDegrees, String direction, double timeCutOff) {
            super.turnEncoder(speed, turnDegrees, direction, timeCutOff);
        }

        @Override
        public void gyroDrive(double speed, double distance, double angle, int timeCutOff) {
            super.gyroDrive(speed, distance, angle, timeCutOff);
        }

        @Override
        public void gyroStrafe(double speed, double distance, double angle, String direction, int timeCutOff) {
            super.gyroStrafe(speed, distance, angle, direction, timeCutOff);
        }

        @Override
        public void gyroTurn(double speed, double angle, String direction, int timeCutOff) {
            super.gyroTurn(speed, angle, direction, timeCutOff);
        }

        @Override
        public void clamp(String position, int sleep) {
            super.clamp(position, sleep);
        }

        @Override
        public void foundationClamps(String position, int sleep) {
            super.foundationClamps(position, sleep);
        }

        @Override
        public void turnClamp(String position, int sleep) {
            super.turnClamp(position, sleep);
        }

        @Override
        public void actuatorDistance(int distanceCM, double speed) {
            super.actuatorDistance(distanceCM, speed);
        }

        @Override
        public void liftDistance(int distanceCM, double speed) {
            super.liftDistance(distanceCM, speed);
        }

        @Override
        public void runOpMode() throws InterruptedException {
            super.runOpMode();
        }

        @Override
        public double getError(double targetAngle) {
            return super.getError(targetAngle);
        }

        @Override
        public double getSteer(double error, double correctionCoeff) {
            return super.getSteer(error, correctionCoeff);
        }
        @Override
        public double getZAngle() {
            return super.getZAngle();
        }
    };

    final double PIVOT_SPEED = 0.5;

    // Gobilda Motor Specs
    double COUNTS_PER_MOTOR_GOBUILDA = 537.5;    // gobilda
    double DRIVE_GEAR_REDUCTION = 1;    // 1:1
    double WHEEL_DIAMETER_CM = 10;     // mecanum wheels
    double TUNING_DRIVE = 1.22;
    double COUNTS_PER_CM_GOBUILDA = ((COUNTS_PER_MOTOR_GOBUILDA * DRIVE_GEAR_REDUCTION * TUNING_DRIVE) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffsetFront = 0;
    double horizontalTickOffsetBack = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFrontFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffsetFront.txt");
    File horizontalTickOffsetBackFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffsetBack.txt");

    @Override
    public void runOpMode() throws InterruptedException {

        auto_methods.initPracticeBot();
        // FRONT LEFT is vertical odometry wheel on the left side
        // FRONT RIGHT is vertical odometry wheel on the right side
        // BACK LEFT is horizontal odometry wheel on the front side
        // BACK RIGHT is horizontal odometry wheel on the back side
        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(auto_methods.getZAngle() < 90 && opModeIsActive()){
            auto_methods.driveFrontLeft.setPower(-PIVOT_SPEED);
            auto_methods.driveBackLeft.setPower(-PIVOT_SPEED);
            auto_methods.driveFrontRight.setPower(PIVOT_SPEED);
            auto_methods.driveBackRight.setPower(PIVOT_SPEED);
            if(auto_methods.getZAngle() < 60) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            }else{
                setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", auto_methods.getZAngle());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", auto_methods.getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = auto_methods.getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(auto_methods.driveFrontLeft.getCurrentPosition()) + (Math.abs(auto_methods.driveFrontRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_CM_GOBUILDA);

        horizontalTickOffsetFront = auto_methods.driveBackLeft.getCurrentPosition()/Math.toRadians(auto_methods.getZAngle());
        horizontalTickOffsetBack = auto_methods.driveBackRight.getCurrentPosition()/Math.toRadians(auto_methods.getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFrontFile, String.valueOf(horizontalTickOffsetFront));
        ReadWriteFile.writeFile(horizontalTickOffsetBackFile, String.valueOf(horizontalTickOffsetBack));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset Front", horizontalTickOffsetFront);
            telemetry.addData("Horizontal Encoder Offset Back", horizontalTickOffsetBack);
            //Display raw values
            telemetry.addData("IMU Angle", auto_methods.getZAngle());
            telemetry.addData("Vertical Left Position", auto_methods.driveFrontLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", auto_methods.driveFrontRight.getCurrentPosition());
            telemetry.addData("Horizontal Position Left", auto_methods.driveBackLeft.getCurrentPosition());
            telemetry.addData("Horizontal Position Right", auto_methods.driveBackRight.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);
            //Update values
            telemetry.update();
        }
    }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        auto_methods.driveFrontLeft.setPower(lf);
        auto_methods.driveBackLeft.setPower(lb);
        auto_methods.driveFrontRight.setPower(rf);
        auto_methods.driveBackRight.setPower(rb);
    }
}
