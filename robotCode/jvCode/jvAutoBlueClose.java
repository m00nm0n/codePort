import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Your_Username on 11/13/17.
 */
@Autonomous(name="JV AUTO BLUE CLOSE", group="Linear Opmode")
//@Disabled
public class jvAutoBlueClose extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo rightServo = null;
    private Servo leftServo = null;
    private DcMotor liftMotor = null;
    private Servo rightBase = null;
    private Servo leftBase = null;
    private Servo rightArm = null;
    private Servo leftArm = null;
    private ColorSensor rightSensor = null;
    private ColorSensor leftSensor = null;
    private double time = 1.0;

    public void wait(double time) {
        double startTime = System.currentTimeMillis();
        while (((System.currentTimeMillis() - startTime) * Math.pow(10, -3)) < time) { /*Waiting for bot to finish*/ }
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "ld");
        rightDrive = hardwareMap.get(DcMotor.class, "rd");
        rightServo = hardwareMap.servo.get("rs");
        leftServo = hardwareMap.servo.get("ls");
        liftMotor = hardwareMap.get(DcMotor.class, "lifty");
//        rightArm = hardwareMap.servo.get("ra");
//        leftArm = hardwareMap.servo.get("la");
//        rightBase = hardwareMap.servo.get("rb");
//        leftBase = hardwareMap.servo.get("lb");
//        rightSensor = hardwareMap.get(ColorSensor.class, "rc");
//        leftSensor = hardwareMap.get(ColorSensor.class, "lc");
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
//        leftBase.setDirection(Servo.Direction.FORWARD);
//        rightBase.setDirection(Servo.Direction.FORWARD);

        //grip box
        rightServo.setPosition(0.0);
        leftServo.setPosition(0.0);

        //initializing servo positions
//        leftBase.setPosition(0.2);
//        leftArm.setPosition(0.4);
//        rightBase.setPosition(0.8);

        //pick up block so that it doesn't drag on the ground
        liftMotor.setPower(0.9);
        sleep(2000);
        liftMotor.setPower(0.0);

        /*setting arm position so that it can do the color sensing stuff*/
//        leftBase.setPosition(0.8);
//        sleep(1000);
        //sensing
//        int red = 0, blue = 0;
//        for (int times = 0; times < 1000; times++) {
//            red += leftSensor.red();
//            blue += leftSensor.blue();
//            sleep(1);
//        }
        //knock off jewel
//        if (blue < red)
//            leftArm.setPosition(1.0);
//        else if (red < blue)
//            leftArm.setPosition(0.0);
//        else;
//        sleep(1000);

        //resets servo arm position
//        leftBase.setPosition(0.2);

        //veers into scoring position
        rightDrive.setPower(0.2);
        leftDrive.setPower(0.4);
        sleep(300);
        rightDrive.setPower(0.2);
        sleep(1000);
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);

        //putting the block down
        liftMotor.setPower(-0.9);
        sleep(2000);
        liftMotor.setPower(0.0);

        //drops block and backs up at the same time
        leftDrive.setPower(-0.1);
        rightDrive.setPower(-0.1);
        rightServo.setPosition(0.5);
        leftServo.setPosition(0.5);
        sleep(500);
        rightDrive.setPower(0.0);
        leftDrive.setPower(0.0);
    }
    }

/*
1=lb
2=rb
4=ra
3=la

 */