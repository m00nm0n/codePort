/**
 * Created by Your_Username on 11/2/17.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Robot {


    // Attribute Declarations
    private double velocity;
    private double rotate_time;
    private DcMotor right_motor;
    private DcMotor left_motor;
    private DcMotor arm;

    //Constructors
    public Robot(double v, double rt, DcMotor rm, DcMotor lm) {
        velocity = v;
        rotate_time = rt;
        right_motor = rm;
        left_motor = lm;
    }

    public Robot() {
        right_motor = null;
        left_motor = null;
    }

    // Getter/Setter functions
    public void setVelocity(double v) {
        velocity = v;
    }

    public void setRotate_time(double rt) {
        rotate_time = rt;
    }

    public void setRight_motor(DcMotor rm) {
        right_motor = rm;
    }

    public void setLeft_motor(DcMotor lm) {
        left_motor = lm;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getRotate_time() {
        return rotate_time;
    }

    public DcMotor getRight_motor() {
        return right_motor;
    }

    public DcMotor getLeft_motor() {
        return left_motor;
    }

    public DcMotor getArm() { return arm; }

    //Private functions
    private double calculate_turn_time(int degree) {
        double div_val = 90 / degree;
        return rotate_time / div_val;
    }

    //Public functions
    public void rotateRight(int degree) {
        long startTime = System.currentTimeMillis();
        left_motor.setPower(1.0);
        double rtime = calculate_turn_time(degree);
        this.wait(rtime);
        left_motor.setPower(0.0);
    }

    public void wait(double time) {
        long startTime = System.currentTimeMillis();
        while (((System.currentTimeMillis() - startTime) * Math.pow(10, -3)) < time) { /*Wait for turn*/ }
    }

    public void rotateLeft(int degree) {
        right_motor.setPower(1.0);
        double rtime = calculate_turn_time(degree);
        this.wait(rtime);
        right_motor.setPower(0.0);
    }

    public void goXFeetForward(double dist) {
        double time = dist / velocity;
        left_motor.setPower(1.0);
        right_motor.setPower(1.0);
        this.wait(time);
        left_motor.setPower(0.0);
        right_motor.setPower(0.0);
    }

    public void goXFeetBackwards(double dist) {
        double time = dist / velocity;
        left_motor.setPower(-1.0);
        right_motor.setPower(-1.0);
        this.wait(time);
        left_motor.setPower(0.0);
        right_motor.setPower(0.0);
    }

    public void extendArm() {
        arm.setPower(1.0);
        this.wait(.0005);
        arm.setPower(0.0);
    }

    public String findColor(ColorSensor colorSensor) {
        int red = colorSensor.red(), green = colorSensor.green(), blue = colorSensor.blue();
        double percentBlue = ((double) (red + green + blue) / (double) (blue));
        double percentRed = ((double) (red + green + blue) / (double) (red));
        if(blue!=0 && percentBlue > 0.60)
            return "blue";
        if(red!=0 && percentRed > 0.60)
            return "red";
        return "no color can be detected";
    }

}
