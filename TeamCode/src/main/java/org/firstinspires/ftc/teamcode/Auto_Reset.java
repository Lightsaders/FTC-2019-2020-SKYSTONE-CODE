package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "AUTO_RESET")
public class Auto_Reset extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initialize();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            turnClamp("PERP", 250);
            clamp("CLOSE", 250);
            actuator.setPower(-1);//TODO use method created however it requires encoders on actuator
            sleep(550);
            actuator.setPower(0);
        }
    }
}