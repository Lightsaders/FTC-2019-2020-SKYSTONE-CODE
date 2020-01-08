package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous (name = "SMART_AUTO_RED")
public class Smart_Auto_Red extends LinearOpMode {

    Methods method = new Methods();

    @Override
    public void runOpMode() throws InterruptedException {
        // init robot
        method.init(hardwareMap);
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // Robot actions in auto
            method.straightDriveEncoder(1,120);
            //sleep(1000);
//            method.strafeDriveEncoder(1,100,"LEFT");
//            sleep(1000);
//            method.turnEncoder(1,90,"CC");
        }
    }
}
