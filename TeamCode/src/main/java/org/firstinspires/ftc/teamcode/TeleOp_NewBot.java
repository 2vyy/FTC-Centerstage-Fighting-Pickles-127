package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp_NewBot", group="Linear Opmode")

public class TeleOp_NewBot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private Servo leftSwiper;
    private Servo rightSwiper;

    double leftPower;
    double rightPower;

    double upperPowerBound = 1.0;
    double lowerPowerBound = -1.0;
    
    @Override
    public void runOpMode() {

        //initializing motors
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftSwiper = hardwareMap.get(Servo.class, "leftSwiper");
        rightSwiper = hardwareMap.get(Servo.class, "rightSwiper");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); //probably reverse
        leftSwiper.setDirection(Servo.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightSwiper.setDirection(Servo.Direction.REVERSE);
//        leftSwiper.setPosition(.3);
//        runtime.reset();
//        while(runtime.time()<1.0) {}
//        rightSwiper.setPosition(.5);
//
//        runtime.reset();
//        while(runtime.time()<1.0) {}
//
//        rightSwiper.setPosition(.9);
//        runtime.reset();
//        while(runtime.time()<.5) {}
//        leftSwiper.setPosition(.7);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //maybe set mode?

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Init");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            swiperAction();


            //DRIVE
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.update();
        }
    }

    public void swiperAction() {
        if(gamepad1.a) {
            rightSwiper.setPosition(.9);
            leftSwiper.setPosition(.7);
        } else if (gamepad1.b) {
            leftSwiper.setPosition(.3);
            rightSwiper.setPosition(.5);
        }
    }
}
