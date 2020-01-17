package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "AUTO_RESET_FOUNDATION")
public class Auto_Reset_Foundatio extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initialize();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            actuator.setPower(-1);//TODO use method created however it requires encoders on actuator
            sleep(100);
            actuator.setPower(0);
            turnClamp("PERP", 700);
            clamp("CLOSE",500);
        }
    }
}