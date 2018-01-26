

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Your_Username on 11/13/17.
 */
@Autonomous(name="JV AUTO BLUE FAR", group="Linear Opmode")
//@Disabled
public class jvAutoBlueFar extends LinearOpMode {

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

    public boolean isBlue(ColorSensor color_sensor) {

        int green = color_sensor.green();
        int blue = color_sensor.blue();
        int red = color_sensor.red();
        telemetry.addData(""+red," RED RGB VALUE");
        telemetry.addData(""+color_sensor.blue()," BLUE RGB VALUE");
        telemetry.addData(""+color_sensor.green()," GREEN RGB VALUE");
        telemetry.update();
        double percent_blue = ((double) (red + green + blue)) / ((double) (blue));
        if (blue != 0 && blue != 255 && percent_blue > 0.75)
            return true;
        return false;
    }

    public boolean isRed(ColorSensor color_sensor) {
        int green = color_sensor.green();
        int blue = color_sensor.blue();
        int red = color_sensor.red();
        telemetry.addData(""+red," RED RGB VALUE");
        telemetry.addData(""+color_sensor.blue()," BLUE RGB VALUE");
        telemetry.addData(""+color_sensor.green()," GREEN RGB VALUE");
        telemetry.update();
        double percent_red = ((double) (red + green + blue)) / ((double) (red));
        if(red != 0 && blue != 255 && percent_red > 0.75)
            return true;
        return false;
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
        //rightArm = hardwareMap.servo.get("ra");
        //leftArm = hardwareMap.servo.get("la");
        //rightBase = hardwareMap.servo.get("rb");
        //leftBase = hardwareMap.servo.get("lb");
        //rightSensor = hardwareMap.get(ColorSensor.class, "rc");
        //leftSensor = hardwareMap.get(ColorSensor.class, "lc");
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        //leftBase.setDirection(Servo.Direction.FORWARD);
        //rightBase.setDirection(Servo.Direction.FORWARD);
        rightServo.setPosition(0.0);
        leftServo.setPosition(0.0);
        //leftBase.setPosition(0.2);
        //rightBase.setPosition(0.8);
        liftMotor.setPower(0.9);
        //leftArm.setPosition(0.4);
        //sleep(1000);
        liftMotor.setPower(0.0);
//        leftBase.setPosition(0.8);
//        sleep(5000);
//        int red = 0, blue = 0;
//        for (int times = 0; times < 1000; times++) {
//            red += leftSensor.red();
//            red+= rightSensor.red();
//            blue += rightSensor.blue();
//            blue += leftSensor.blue();
//            sleep(1);
//        }
//        red = leftSensor.red() + rightSensor.red();
//        blue = leftSensor.blue() +rightSensor.blue();
        //if (isRed(leftSensor))
        //    leftArm.setPosition(0.0);
        //if (isBlue(leftSensor))
        //    leftArm.setPosition(1.0);

        //telemetry.update();
        //sleep(1000);

//        leftBase.setPosition(0.2);
        sleep(1000);
        rightDrive.setPower(0.3);
        leftDrive.setPower(0.2);
        sleep(1200);
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        sleep(250);
        rightDrive.setPower(0.5);
        leftDrive.setPower(-0.5);
        sleep(500);
        rightDrive.setPower(0.0);
        leftDrive.setPower(0.0);
        sleep(250);
        rightDrive.setPower(0.3);
        leftDrive.setPower(0.2);
        sleep(2000);
        liftMotor.setPower(-0.9);
        rightDrive.setPower(0.0);
        leftDrive.setPower(0.0);
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
/*
1=lb
2=rb
4=ra
3=la

 */