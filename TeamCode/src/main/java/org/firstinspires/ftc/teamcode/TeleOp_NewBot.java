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
    
    @Override
    public void runOpMode() {

        //initializing motors
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftSwiper = hardwareMap.get(Servo.class, "leftSwiper");
        //rightSwiper = hardwareMap.get(Servo.class, "rightSwiper");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); //probably reverse
        leftSwiper.setDirection(Servo.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightSwiper.setDirection(Servo.Direction.REVERSE);
        //rightSwiper.setPosition(0.25);
        leftSwiper.setPosition(.4);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Init");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            swiperAction();
            //powerAction();



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
            telemetry.addLine("Trying to open");
            leftSwiper.setPosition(.7);
            //rightSwiper.setPosition(.45);
        } else if (gamepad1.b) {
            leftSwiper.setPosition(.3);
            //rightSwiper.setPosition(.25);
        }
    }

    public void powerAction() {
        if(gamepad1.y) {
            if(leftPower==1.0) {
                leftPower=0.5;
                rightPower=0.5;
            } else {
                leftPower=1;
                rightPower=1;
            }
        }
    }
}
