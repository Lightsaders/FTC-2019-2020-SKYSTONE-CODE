package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FOUNDATION_GRAB_BLUE")
public class Foundation_Grab_Blue extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initialize();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            strafeDriveEncoder(.7,19,"LEFT",3);
            straightDriveEncoder(.3,120,0);
            leftFoundation.setPosition(1);// TODO replace with method foundationClamps
            rightFoundation.setPosition(0.15);
            sleep(1500);
            straightDriveEncoder(.3,-130,0);
            leftFoundation.setPosition(0);// TODO replace with method foundationClamps
            rightFoundation.setPosition(1);
            sleep(1500);
            strafeDriveEncoder(.8,100,"RIGHT",3);
        }
        }
}
