package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "rightRedAuto")
public class rightRedAuto extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    public DcMotor[] motors = new DcMotor[]{leftFront, leftRear, rightFront, rightRear};
    private DcMotor slidesAsDcMotor;
    private Servo rightAsServo;
    private Servo leftAsServo;

    // inches, speed is 0-1
    private void Move_Forward_Backward(int Distance, double Speed) {
        Reset();
        for (DcMotor motor : motors) {
            motor.setTargetPosition(Distance*120);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Speed);
        }
        while (leftFront.isBusy() || rightRear.isBusy()) {
            sleep(100);
        }
    }

    private void Turning(double Target_Degrees) {
        Reset();
        leftFront.setTargetPosition((int) (Target_Degrees * -26.44));
        leftRear.setTargetPosition((int) (Target_Degrees * -26.44));
        rightFront.setTargetPosition((int) (Target_Degrees * 26.44));
        rightRear.setTargetPosition((int) (Target_Degrees * 26.44));
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
        while (leftFront.isBusy() || rightRear.isBusy()) {
            sleep(100);
        }
    }

    // inches
    private void Move_Left_Right(int Distance) {
        Reset();
        leftFront.setTargetPosition(Distance * 144);
        rightFront.setTargetPosition(Distance * -144);
        leftRear.setTargetPosition(Distance * -144);
        rightRear.setTargetPosition(Distance * 144);
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setPower(0.8);
        }
        while (leftFront.isBusy() || rightRear.isBusy()) {
            sleep(100);
        }
    }

    //0 to idk like 9000 ish?
    private void SlideMovement(int Target) {
        Reset();
        slidesAsDcMotor.setTargetPosition(Target);
        slidesAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesAsDcMotor.setPower(1);
    }
    private void openClaw() {
        rightAsServo.setPosition(0.275);
        leftAsServo.setPosition(0.275);
    }
    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        slidesAsDcMotor = hardwareMap.get(DcMotor.class, "slidesAsDcMotor");
        rightAsServo = hardwareMap.get(Servo.class, "rightAsServo");
        leftAsServo = hardwareMap.get(Servo.class, "leftAsServo");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftAsServo.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        // Put initialization blocks here. NO NECESSARY CHANGES ABOVE HERE
        if (opModeIsActive()) {
            // PUT AUTONOMOUS INSTRUCTIONS HERE
            Move_Left_Right(-2);
            Move_Forward_Backward(36, 1);
            Move_Left_Right(-24);
            SlideMovement(8250);
            openClaw();
            sleep(1000);
            closeClaw();
            SlideMovement(0);
            Move_Left_Right(-24);
            Move_Forward_Backward(12, 1);
        }
        while (opModeIsActive()) {
            // Put loop blocks here.
            telemetry.update();
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
        }
    }
    private void closeClaw() {
        rightAsServo.setPosition(0.5);
        leftAsServo.setPosition(0.5);
    }
    private void Reset() {
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        slidesAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}