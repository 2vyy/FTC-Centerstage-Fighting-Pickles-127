package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp_Test", group="Linear Opmode")

public class TeleOp_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime arm_timer = new ElapsedTime();

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor motor;
    private CRServo leftArm;
    private CRServo rightArm;
    private Servo claw;
    private double leftPower;
    private double rightPower;
    private double speedLimit;
    private final int TICKS_PER_REV = 288;
    private final double CIRCUMFERENCE = 3.14;
    private final double TICKS_PER_INCH = TICKS_PER_REV/CIRCUMFERENCE;

    private enum ARM_POS {
        UP,
        DOWN,
        HOLD
    }

    private ARM_POS current_pos = ARM_POS.HOLD;

    @Override
    public void runOpMode() {

        //initializing motors
        motor = hardwareMap.get(DcMotor.class, "hexMotor");
        leftArm = hardwareMap.get(CRServo.class, "left");
        rightArm = hardwareMap.get(CRServo.class, "right");
        claw = hardwareMap.get(Servo.class, "claw");

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(.4);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftArm.setDirection(CRServo.Direction.FORWARD);
        rightArm.setDirection(CRServo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        leftArm.setPower(0);
        rightArm.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Init");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            motorAction();
            armAction();
            clawAction();
            driveAction(-gamepad1.left_stick_y, gamepad1.right_stick_x);

            //telemetry.addLine(arm1.getPosition()+"");
            telemetry.addLine(leftArm.getPower()+"");
            telemetry.addLine(rightArm.getPower()+"");
            telemetry.addLine(motor.getCurrentPosition()+"");
            telemetry.addLine(claw.getPosition()+"");
            telemetry.addLine(current_pos+"");
            telemetry.addLine(arm_timer+"");

            telemetry.update();
        }
    }

    public void driveAction(double drive, double turn) {
        if(gamepad1.right_trigger>0.2) {
            speedLimit = 0.25;
        } else {
            speedLimit = 1.0;
        }

        double denominator = Math.max(Math.abs(drive) + Math.abs(turn), 1);
        leftPower    = Range.clip(drive + turn, -speedLimit, speedLimit) / denominator ;
        rightPower   = Range.clip(drive - turn, -speedLimit, speedLimit) / denominator ;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void motorAction() {
        if(gamepad1.x) {
            motor.setTargetPosition(0);
        } else if (gamepad1.left_bumper) {
            motor.setTargetPosition((int) (-4.1 * TICKS_PER_INCH));
        } else if (gamepad1.y) {
            motor.setTargetPosition((int) (4.1 * TICKS_PER_INCH));
        }

//        if(!motor.isBusy()) {
//            motor.setPower(0);
//        } else {
//            motor.setPower(.2);
//        }
    }

    public void armAction() {
        if(gamepad1.dpad_down) {
            current_pos = ARM_POS.DOWN;
        } else if (gamepad1.dpad_up) {
            current_pos = ARM_POS.UP;
        }

        switch(current_pos) {
            case HOLD: {
                double leftArmPower = leftArm.getPower();
                if(leftArmPower!=0 && leftArmPower!=-.1) {
                    leftArm.setPower(-.1);
                    rightArm.setPower(-.1);
                }
            }
            break;

            case UP: {
                if(arm_timer.time()>1) {
                    arm_timer.reset();
                }
                leftArm.setPower(-.45);
                rightArm.setPower(-.45);
            }
            break;

            case DOWN: {
                if(arm_timer.time()>1) {
                    arm_timer.reset();
                }
                leftArm.setPower(.2);
                rightArm.setPower(.2);
            }
            break;

        }

        if(arm_timer.time()>.75) {
            if(current_pos == ARM_POS.DOWN) {
                leftArm.setPower(0);
                rightArm.setPower(0);
            }
            current_pos = ARM_POS.HOLD;
        }
    }

    public void clawAction() {
        if(gamepad1.a) {
            claw.setPosition(.1);
        } else if (gamepad1.b) {
            claw.setPosition(0);
        }
    }
}
