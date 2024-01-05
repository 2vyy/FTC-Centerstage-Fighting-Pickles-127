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
    private Servo arm;
    private double leftPower;
    private double rightPower;
    private final int TICKS_PER_REV = 288;
    private final double CIRCUMFERENCE = 3.14;
    private final double TICKS_PER_INCH = TICKS_PER_REV/CIRCUMFERENCE;

    @Override
    public void runOpMode() {

        //initializing motors
        motor = hardwareMap.get(DcMotor.class, "hexMotor");
        arm = hardwareMap.get(Servo.class, "arm");

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
        arm.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Init");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            motorAction();
            armAction();

            //DRIVE
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addLine(arm.getPosition()+"");
            telemetry.addLine(motor.getCurrentPosition()+"");

            telemetry.update();
        }
    }

    public void motorAction() {
        if(gamepad1.x) {
            motor.setTargetPosition(0);
        } else if (gamepad1.y) {
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
        if(gamepad1.a) {
            telemetry.addLine("trying to raise");
            arm.setPosition(0.55);
        } else if (gamepad1.b) {
            telemetry.addLine("trying to lower");
            arm.setPosition(0.475);
        }
    }
}
