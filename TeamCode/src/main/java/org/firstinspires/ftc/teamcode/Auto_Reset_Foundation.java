package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AUTO_RESET_FOUNDATION")
public class Auto_Reset_Foundation extends Auto_Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        // init robot
        initCompBot();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            actuator.setPower(-1);//TODO use method created however it requires encoders on actuator
            sleep(300);
            actuator.setPower(0);


        }
    }
}