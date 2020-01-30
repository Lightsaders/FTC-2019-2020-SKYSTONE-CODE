package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE_FOUNDATION_GRAB_TURN")
public class BLUE_FOUNDATION_GRAB_NF extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initCompBot();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            strafeDriveEncoder(0.3, 10,  "LEFT",2);
            rightFoundation.setPosition(.4);
            sleep(1000);
            actuator.setPower(1);//TODO use method created however it requires encoders on actuator
            sleep(300);
            actuator.setPower(0);
            turnClamp("PAR", 700);
            clamp("OPEN", 500);
            actuator.setPower(-1);//TODO use method created however it requires encoders on actuator
            sleep(100);
            actuator.setPower(0);
            rightFoundation.setPosition(.8);
            turnClamp("PAR", 250);
            clamp("OPEN", 250);
            gyroDrive(.2, 88, 0,2);
            leftFoundation.setPosition(1);
            rightFoundation.setPosition(0.1);
            sleep(1600);
            turnEncoder(0.6,55,"CC", 2);
            straightDriveEncoder(0.6,-30,2);
            leftFoundation.setPosition(1);
            rightFoundation.setPosition(0.1);
            turnEncoder(0.6,75,"CC",2);
            straightDriveEncoder(.5,80,3);
            leftFoundation.setPosition(0.2);
            rightFoundation.setPosition(.9);
            sleep(1500);
            strafeDriveEncoder(.5,50,"LEFT",2);
            straightDriveEncoder(.5,-140,2);
            clamp("CLOSE", 700);

        }
    }
}