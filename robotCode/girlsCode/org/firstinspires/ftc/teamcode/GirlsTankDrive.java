package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TankDriveGirl", group="Iterative Opmode")
@Disabled
public class GirlsTankDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftMotor = null;
    private DcMotor arm = null;
    final double liftPower=1.0;
    private Servo leftServo = null;
    private Servo rightServo = null;
    double leftServoPosition;
    double rightServoPosition;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "ld");
        rightDrive = hardwareMap.get(DcMotor.class, "rd");
        arm = hardwareMap.get(DcMotor.class,"arm");
        leftServo = hardwareMap.get(Servo.class, "ls");
        rightServo = hardwareMap.get(Servo.class, "rs");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        leftPower  = -gamepad1.left_stick_y / 2;
        rightPower = -gamepad1.right_stick_y / 2;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        if (gamepad2.right_bumper) {
            liftMotor.setPower(liftPower);

        }
        else if (gamepad2.left_bumper) {
            liftMotor.setPower(-liftPower);
        }
        else
            liftMotor.setPower(0.0);


        if (gamepad2.y) {
            arm.setPower(0.3); // extend arm
        } else {
            arm.setPower(0);
        }

        if (gamepad2.a) {
            arm.setPower(-0.3); // extend arm
        } else {
            arm.setPower(0);
        }



        if (gamepad1.right_bumper) {
            // open the servo
            leftServoPosition = 1;
            rightServoPosition = 0;
        } else {
            // closed position
            leftServoPosition = 0.775;
            rightServoPosition = 0.125;
        }

        leftServo.setPosition(leftServoPosition);
        rightServo.setPosition(rightServoPosition);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
