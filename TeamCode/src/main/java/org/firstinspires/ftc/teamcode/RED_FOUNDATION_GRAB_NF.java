package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "RED_FOUNDATION_GRAB_NF")
public class RED_FOUNDATION_GRAB_NF extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initCompBot();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            straightDriveEncoder(0.3, 39, 1);
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
            straightDriveEncoder(.5,-100,3);
            leftFoundation.setPosition(0.2);
            rightFoundation.setPosition(.9);
            sleep(1500);
            strafeDriveEncoder(.5,50,"LEFT",2);
            clamp("CLOSE", 700);
            strafeDriveEncoder(.5,50,"LEFT",2);
            straightDriveEncoder(.3,-15,1);

        }
    }
}
