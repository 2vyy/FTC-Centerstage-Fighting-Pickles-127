package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp_NewBot", group="Linear Opmode")

public class TeleOp_NewBot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor topTankTread;
    private DcMotor bottomTankTread;
    private Servo leftSwiper;
    private Servo rightSwiper;
    private Servo clawRotate;
    private Servo claw;

    private void tankTreadAction() {
        if(gamepad1.x) {
            topTankTread.setPower(1);
            bottomTankTread.setPower(1);
        } else {
            topTankTread.setPower(0);
            bottomTankTread.setPower(0);
        }
    }

    private void swiperAction() {
        double curTime = runtime.time();
        if(gamepad1.a) {
            leftSwiper.setPosition(.5);
            rightSwiper.setPosition(.5);
            if (runtime.time()-curTime>1.0) {
                leftSwiper.setPosition(0);
                rightSwiper.setPosition(0);
                curTime = runtime.time();
            }
        }
    }

    private void clawAction() {
        if(gamepad1.y) {
            clawRotate.setPosition(.35);
            claw.setPosition(.25);
        } else {
            clawRotate.setPosition(0);
            claw.setPosition(.5);
        }
    }
    
    @Override
    public void runOpMode() {

        //initializing motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        topTankTread = hardwareMap.get(DcMotor.class, "topTankTread");
        bottomTankTread = hardwareMap.get(DcMotor.class, "bottomTankTread");
        leftSwiper = hardwareMap.get(Servo.class, "leftSwiper");
        rightSwiper = hardwareMap.get(Servo.class, "rightSwiper");
        claw = hardwareMap.get(Servo.class, "claw");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");

        leftSwiper.setDirection(Servo.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        topTankTread.setDirection(DcMotor.Direction.FORWARD);
        bottomTankTread.setDirection(DcMotor.Direction.FORWARD);
        rightSwiper.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Init");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            swiperAction();
            tankTreadAction();
            clawAction();
            double max;
            
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
//            double leftBackPower = axial - lateral + yaw;
//            double rightBackPower = axial + lateral - yaw;
            

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//            max = Math.max(max, Math.abs(leftBackPower));
//            max = Math.max(max, Math.abs(rightBackPower));

            if(max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
//                leftBackPower /= max;
//                rightBackPower /= max;
            }
            
            //leftFrontPower  = gamepad1.x ? 1.0 : -1.0;  // X gamepad
            //leftBackPower   = gamepad1.a ? 1.0 : -1.0;  // A gamepad
            //rightFrontPower = gamepad1.y ? 1.0 : -1.0;  // Y gamepad
            //rightBackPower  = gamepad1.b ? 1.0 : -1.0;  // B gamepad
            

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
//            leftBackDrive.setPower(leftBackPower);
//            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("Slide", slides.getCurrentPosition());
            telemetry.update();
        }
    }
}
