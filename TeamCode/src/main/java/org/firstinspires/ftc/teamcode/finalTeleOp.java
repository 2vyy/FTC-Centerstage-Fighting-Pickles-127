package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="finalTeleOp", group="Linear Opmode")

public class finalTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor slides;
    private Servo right;
    private Servo left;
    
    private void slideAction() {
        //if claw open, close before raising/lowering
        if(left.getPosition()==0.25 && (gamepad1.dpad_up || gamepad1.dpad_down)) {
            left.setPosition(0.5);
            right.setPosition(0.5);
        }
        if (gamepad1.dpad_up) {
            slides.setPower(1);
        } else if (gamepad1.dpad_down) {
            slides.setPower(-1);
        } else {
            slides.setPower(0);
        }
    }
    
    private void clawAction() {
        if (gamepad1.b) {
              left.setPosition(0.5);
              right.setPosition(0.5);
            } else if (gamepad1.a) {
              left.setPosition(0.25);
              right.setPosition(0.25);
            }
    }
    
    @Override
    public void runOpMode() {

        //initializing motors
        slides = hardwareMap.get(DcMotor.class, "slides");
        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");
        left.setDirection(Servo.Direction.REVERSE);
        
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Test", "" + runtime.time());
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            slideAction();
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
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if(max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            
            //leftFrontPower  = gamepad1.x ? 1.0 : -1.0;  // X gamepad
            //leftBackPower   = gamepad1.a ? 1.0 : -1.0;  // A gamepad
            //rightFrontPower = gamepad1.y ? 1.0 : -1.0;  // Y gamepad
            //rightBackPower  = gamepad1.b ? 1.0 : -1.0;  // B gamepad
            

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Slide", slides.getCurrentPosition());
            telemetry.update();
        }
    }
}
