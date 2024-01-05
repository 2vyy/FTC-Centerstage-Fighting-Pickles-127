package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "servoAuto")
public class servoAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Servo arm;
    @Override
    public void runOpMode() {
        arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        arm.setPosition(0.5);
        arm.setPosition(1);

        runtime.reset();
        while(runtime.time()<3.0) {}
    }
}