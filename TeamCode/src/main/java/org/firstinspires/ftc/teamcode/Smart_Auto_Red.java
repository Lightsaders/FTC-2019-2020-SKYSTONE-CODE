package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous (name = "SMART_AUTO_RED")
public class Smart_Auto_Red extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initialize();
        telemetry.addLine();
        telemetry.update();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // Robot actions in auto

            straightDriveEncoder(1,20);
//            method.driveFrontLeft.setPower(1.0);
//            sleep(1000);
        }
    }
}
