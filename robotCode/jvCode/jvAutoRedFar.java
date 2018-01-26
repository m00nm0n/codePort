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
@Autonomous(name="JV AUTO RED FAR", group="Linear Opmode")
//@Disabled
public class jvAutoRedFar extends LinearOpMode {

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
//        rightArm = hardwareMap.servo.get("ra");
//        leftArm = hardwareMap.servo.get("la");
//        rightBase = hardwareMap.servo.get("rb");
//        leftBase = hardwareMap.servo.get("lb");
        leftDrive = hardwareMap.get(DcMotor.class, "ld");
        rightDrive = hardwareMap.get(DcMotor.class, "rd");
        rightServo = hardwareMap.servo.get("rs");
        leftServo = hardwareMap.servo.get("ls");
//        rightSensor = hardwareMap.colorSensor.get("rc");
//        leftSensor = hardwareMap.colorSensor.get("lc");
        liftMotor = hardwareMap.get(DcMotor.class, "lifty");
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBase.setDirection(Servo.Direction.FORWARD);
//        rightBase.setDirection(Servo.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightServo.setPosition(0.0);
        leftServo.setPosition(0.0);
//        leftBase.setPosition(0.2);
//        rightBase.setPosition(0.9);
//        leftArm.setPosition(0.5);
//        rightArm.setPosition(0.55);
        liftMotor.setPower(0.9);
        sleep(1000);
        liftMotor.setPower(0.0);
//        rightBase.setPosition(0.3);
//        int red = 0, blue = 0;
//        for (int times = 0; times < 1000; times++) {
//            red += leftSensor.red();
//            blue += leftSensor.blue();
//            sleep(1);
//        }
//        sleep(1000);
//        if (blue < red)
//            rightArm.setPosition(1.0);
//        else if (red < blue)
//            rightArm.setPosition(0.0);
//        else;
//        sleep(1000);
//        rightBase.setPosition(0.9);
//        sleep(1000);
        rightServo.setPosition(0.0);
        leftServo.setPosition(0.0);
        liftMotor.setPower(0.9);
        sleep(2000);
        liftMotor.setPower(0.0);
        rightDrive.setPower(0.2);
        leftDrive.setPower(0.2);
        sleep(1200);
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        sleep(250);
        rightDrive.setPower(-0.5);
        leftDrive.setPower(0.5);
        sleep(500);
        rightDrive.setPower(0.0);
        leftDrive.setPower(0.0);
        sleep(250);
        rightDrive.setPower(0.2);
        leftDrive.setPower(0.2);
        sleep(2000);
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        liftMotor.setPower(-0.9);
        sleep(2000);
        liftMotor.setPower(0.0);
        leftDrive.setPower(-0.1);
        rightDrive.setPower(-0.1);
        rightServo.setPosition(0.5);
        leftServo.setPosition(0.5);
        sleep(1000);
        rightDrive.setPower(0.0);
        leftDrive.setPower(0.0);
    }
}
