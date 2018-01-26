

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="varsity", group="TeleOp")  // @Autonomous(...) is the other common choice
@Disabled
public class VarsityTeleOp extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor elevatorMotor = null;
    private DcMotor bepisMotor = null;
    private DcMotor tiltMotor = null;
    private DcMotor pusher = null;
    private Servo claw = null;
    private Servo chopper = null;
    private ColorSensor colorSensor = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void teleWait(double time) {
        double startTime = System.currentTimeMillis();
        while (((System.currentTimeMillis() - startTime) * Math.pow(10, -3)) < time) { /*Wait for turn*/ }
    }
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        //Initialize the hardware variables from phone
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        elevatorMotor = hardwareMap.dcMotor.get("elevator motor");
        bepisMotor = hardwareMap.dcMotor.get("bepis motor");
        tiltMotor = hardwareMap.dcMotor.get("tilt motor");
        pusher = hardwareMap.dcMotor.get("pusher");
        claw = hardwareMap.get(Servo.class, "claw");
        chopper = hardwareMap.get(Servo.class, "chopper");
        colorSensor = hardwareMap.get(ColorSensor.class,"color sensor");


        //Set the drive motor directions
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        bepisMotor.setDirection(DcMotor.Direction.FORWARD);
        tiltMotor.setDirection(DcMotor.Direction.FORWARD);
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
        telemetry.addData("Status", "Running: " + runtime.toString());

        //joystick goes negative when pushed forwards
        leftMotor.setPower(-(gamepad1.left_stick_y/2.0));
        rightMotor.setPower(-(gamepad1.right_stick_y/2.0));

        //elevator control with left joystick on gamepad 2
        elevatorMotor.setPower(gamepad2.left_stick_y);

        //control tilt of bepis with right joystick on gamepad 2
        tiltMotor.setPower(-gamepad2.right_stick_y/2.0);


        if(gamepad2.b) {
            tiltMotor.setPower(1);
            long startTime = System.currentTimeMillis();
            while (((System.currentTimeMillis() - startTime) * Math.pow(10, -3)) < 2) { /*Wait for pusher*/ }
            bepisMotor.setPower(1);
            startTime = System.currentTimeMillis();
            while (((System.currentTimeMillis() - startTime) * Math.pow(10, -3)) < 2) { /*Wait for pusher*/ }
            tiltMotor.setPower(-1);
            startTime = System.currentTimeMillis();
            while (((System.currentTimeMillis() - startTime) * Math.pow(10, -3)) < 2) { /*Wait for pusher*/ }
            bepisMotor.setPower(-1);
            startTime = System.currentTimeMillis();
            while (((System.currentTimeMillis() - startTime) * Math.pow(10, -3)) < 2) { /*Wait for pusher*/ }
        }

        //move claw down with gamepad 1 right bumber and up with gamepad 1 left bumper
        while(gamepad2.right_trigger > 0 && claw.getPosition() != 0)
            claw.setPosition(claw.getPosition()-.05);
        while (gamepad2.left_trigger > 0)
            claw.setPosition(claw.getPosition()+.05);

        //move chopper down with A and up with B
        if(gamepad2.a)
            chopper.setPosition(.5);
        if (gamepad2.b)
            chopper.setPosition(0);

        //bool->int for bepisMotor
        if(gamepad2.dpad_up)
            bepisMotor.setPower(-1);
        else if (gamepad2.dpad_down)
            bepisMotor.setPower(1);
        else
            bepisMotor.setPower(0);

        if (gamepad2.y/*.right_bumper*/) {
            pusher.setPower(0.5);
        }
        else if (gamepad2.x/*left_bumper*/) {
            pusher.setPower(-0.5);
        }
        else {
            pusher.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }

}