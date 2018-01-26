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
@Autonomous(name="JV AUTO SERVO TEST", group="Linear Opmode")
@Disabled
public class jvAutoServoTest extends LinearOpMode {

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
    //private ColorSensor rightSensor = null;
    //private ColorSensor leftSensor = null;
    private double time = 1.0;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "ld");
        rightDrive = hardwareMap.get(DcMotor.class, "rd");
        rightServo = hardwareMap.servo.get("rs");
        leftServo = hardwareMap.servo.get("ls");
        liftMotor = hardwareMap.get(DcMotor.class, "lifty");
        rightArm = hardwareMap.servo.get("ra");
        leftArm = hardwareMap.servo.get("la");
        rightBase = hardwareMap.servo.get("rb");
        leftBase = hardwareMap.servo.get("lb");
        //rightSensor = hardwareMap.get(ColorSensor.class, "rc");
        //leftSensor = hardwareMap.get(ColorSensor.class, "lc");
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBase.setDirection(Servo.Direction.FORWARD);
        rightBase.setDirection(Servo.Direction.FORWARD);
        leftBase.setPosition(0.2);
        rightBase.setPosition(0.9);
        leftArm.setPosition(0.4);
        rightArm.setPosition(0.5);
        sleep(1000);
        for (int count = 3; count > 0; count--) {
leftArm.setPosition(0.0);
rightArm.setPosition(0.9);
rightServo.setPosition(0.5);
leftServo.setPosition(0.5);
rightBase.setPosition(0.8);
leftBase.setPosition(0.3);
sleep(1000);
leftArm.setPosition(0.8);
rightArm.setPosition(0.1);
            rightServo.setPosition(0.0);
            leftServo.setPosition(0.0);
            leftBase.setPosition(0.2);
            rightBase.setPosition(0.9);
sleep(1000);
        }
    }
}
/*
1=lb
2=rb
4=ra
3=la

 */