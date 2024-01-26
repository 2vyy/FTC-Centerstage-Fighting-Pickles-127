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

        //motor.setTargetPosition(50);
        motor.setPower(.4);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //arm.setPosition(0);
        leftArm.setDirection(CRServo.Direction.FORWARD);
        rightArm.setDirection(CRServo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Init");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //motorAction();
            armAction();
            //clawAction();

            //DRIVE
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

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

    public void motorAction() {
        if(gamepad1.x) {
            motor.setTargetPosition(0);
        } else if (gamepad1.left_bumper) {
            motor.setTargetPosition((int) (-3.5 * TICKS_PER_INCH));
        } else if (gamepad1.y) {
            motor.setTargetPosition((int) (3.5 * TICKS_PER_INCH));
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
                if(leftArm.getPower()!=1) {
                    leftArm.setPower(-.1);
                    rightArm.setPower(-.1);
                }
            }
            break;

            case UP: {
                if(arm_timer.time()>1) {
                    arm_timer.reset();
                }
                leftArm.setPower(-.4);
                rightArm.setPower(-.4);
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
            current_pos = ARM_POS.HOLD;
        }
    }

    public void clawAction() {
        if(gamepad1.a) {
            claw.setPosition(.75);
        } else if (gamepad1.b) {
            claw.setPosition(0);
        }
    }
}
