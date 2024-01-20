package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp_Test", group="Linear Opmode")

public class TeleOp_Test extends LinearOpMode {

    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor motor;
    private Servo arm1;
    private Servo arm2;
    private Servo claw;
    private double leftPower;
    private double rightPower;
    private final int TICKS_PER_REV = 288;
    private final double CIRCUMFERENCE = 3.14;
    private final double TICKS_PER_INCH = TICKS_PER_REV/CIRCUMFERENCE;

    @Override
    public void runOpMode() {

        //initializing motors
        motor = hardwareMap.get(DcMotor.class, "hexMotor");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        claw = hardwareMap.get(Servo.class, "claw");

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //arm.setPosition(0);
        arm1.setDirection(Servo.Direction.REVERSE);
        arm2.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Init");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //motorAction();
            //armAction();
            clawAction();

            //DRIVE
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            //telemetry.addLine(arm1.getPosition()+"");
            telemetry.addLine(arm2.getPosition()+"");
            telemetry.addLine(motor.getCurrentPosition()+"");
            telemetry.addLine(claw.getPosition()+"");

            telemetry.update();
        }
    }

    public void motorAction() {
        if(gamepad1.dpad_down) {
            motor.setTargetPosition(0);
        } else if (gamepad1.dpad_up) {
            motor.setTargetPosition((int) (-5 * TICKS_PER_INCH));
        } else if (gamepad1.left_bumper) {
            motor.setTargetPosition((int) (5 * TICKS_PER_INCH));
        }

        if(!motor.isBusy()) {
            motor.setPower(0);
        } else {
            motor.setPower(.2);
        }
    }

    public void armAction() {
        if(gamepad1.x) {
            telemetry.addLine("trying to raise");
            arm1.setPosition(.3);
            arm2.setPosition(.3);
        } else if (gamepad1.y) {
            telemetry.addLine("trying to lower");
            arm1.setPosition(.7);
            arm2.setPosition(.7);
        }
    }

    public void clawAction() {
        if(gamepad1.a) {
            claw.setPosition(.5);
        } else if (gamepad1.b) {
            claw.setPosition(0);
        }
    }
}
