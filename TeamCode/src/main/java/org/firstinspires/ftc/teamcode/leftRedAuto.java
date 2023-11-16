package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "leftRedAuto")
public class leftRedAuto extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    public DcMotor[] motors = new DcMotor[]{leftFront, leftRear, rightFront, rightRear};
    private DcMotor slides;
    private Servo right;
    private Servo left;

    // inches, speed is 0-1
    private void Move_Forward_Backward(int Distance, double Speed) {
        Reset();
        for (DcMotor motor : motors) {
            motor.setTargetPosition(Distance*120);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Speed);
        }
        while (motors[0].isBusy() || motors[3].isBusy()) {
            sleep(100);
        }
    }

    private void Turning(double Target_Degrees) {
        Reset();
        motors[0].setTargetPosition((int) (Target_Degrees * -26.44));
        motors[1].setTargetPosition((int) (Target_Degrees * -26.44));
        motors[2].setTargetPosition((int) (Target_Degrees * 26.44));
        motors[3].setTargetPosition((int) (Target_Degrees * 26.44));
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
        while (motors[0].isBusy() || motors[3].isBusy()) {
            sleep(100);
        }
    }

    // inches
    private void Move_Left_Right(int Distance) {
        Reset();
        motors[0].setTargetPosition(Distance * 144);
        motors[2].setTargetPosition(Distance * -144);
        motors[1].setTargetPosition(Distance * -144);
        motors[3].setTargetPosition(Distance * 144);
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.8);
        }
        while (motors[0].isBusy() || motors[3].isBusy()) {
            sleep(100);
        }
    }

    //0 to idk like 9000 ish?
    private void SlideMovement(int Target) {
        Reset();
        slides.setTargetPosition(Target);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
        while (slides.isBusy()) {
            sleep(100);
        }
    }
    private void openClaw() {
        right.setPosition(0.275);
        left.setPosition(0.275);
    }
    @Override
    public void runOpMode() {
        motors[0] = hardwareMap.get(DcMotor.class, "leftFront");
        motors[2] = hardwareMap.get(DcMotor.class, "rightFront");
        motors[1] = hardwareMap.get(DcMotor.class, "leftRear");
        motors[3] = hardwareMap.get(DcMotor.class, "rightRear");
        slides = hardwareMap.get(DcMotor.class, "slides");
        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");

        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        // Put initialization blocks here. NO NECESSARY CHANGES ABOVE HERE
        if (opModeIsActive()) {
            // PUT AUTONOMOUS INSTRUCTIONS HERE
            closeClaw();
            Move_Left_Right(-6);
            Move_Forward_Backward(84, 1);
            Move_Left_Right(-24);
            SlideMovement(6500);
            openClaw();
            sleep(1000);
            closeClaw();
            SlideMovement(-6500);
            Move_Left_Right(-24);
            Move_Forward_Backward(8, 1);
        }
        while (opModeIsActive()) {
            // Put loop blocks here.
            telemetry.update();
            telemetry.addData("rightFront", motors[2].getCurrentPosition());
            telemetry.addData("leftFront", motors[0].getCurrentPosition());
            telemetry.addData("rightRear", motors[3].getCurrentPosition());
            telemetry.addData("leftRear", motors[1].getCurrentPosition());
        }
    }
    private void closeClaw() {
        right.setPosition(0.5);
        left.setPosition(0.5);
    }
    private void Reset() {
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}