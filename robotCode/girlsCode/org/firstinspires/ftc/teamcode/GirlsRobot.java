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
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import AutonomousModules.ColorSensorBot;


/*
This code has been made specifically for the Girls' Robot
Modify VELOCITY and ROTATE_TIME in order to optimize for
your bot
 */

public class GirlsRobot extends ColorSensorBot {

    private DcMotor arm;
    private Servo arm_servo;

    public GirlsRobot(double v, double rt, DcMotor rm, DcMotor lm,DcMotor a,Servo as,ColorSensor cs /* add extra motors */) {
        super(v,rt,rm,lm,cs);
        arm=a;
        arm_servo=as;
    }

    public void setArm(DcMotor a) { arm=a; }
    public DcMotor getArm() { return arm; }

    /*
    public boolean isBlue() {
        if ((double) color_sensor.blue() / (double)(color_sensor.blue()+color_sensor.red()+color_sensor.green()) > .75)
            return true;
        return false;
    }

    public boolean isRed() {
        if ((double) color_sensor.red() / (double)(color_sensor.blue()+color_sensor.red()+color_sensor.green()) > .75)
            return true;
        return false;
    }
*/
    public void extendArm() {
        arm.setPower(0.3);
        this.wait(1.0);
        arm.setPower(0.0);
    }

    public void hitRed() {
        if (isRed()) {
            arm_servo.setPosition(1.0);
        }
        else {
            arm_servo.setPosition(0.0);
        }
    }

    public void hitBlue() {
        if (isBlue()) {
            arm_servo.setPosition(1.0);
        }
        else {
            arm_servo.setPosition(0.0);
        }
    }

    public void retractArm() {
        getArm().setPower(-0.3);
        this.wait(1.0);
        getArm().setPower(0.0);
    }

    /*public void distanceSensor() {
        if(distanceSensor();
    }*/


}
