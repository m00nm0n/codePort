/**
 * Created by Your_Username on 11/2/17.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
This code has been made specifically for the Varsity Robot 2017/18
Modify VELOCITY and ROTATE_TIME in order to optimize for
your bot
 */

public class BasicMovements {

    final static double VELOCITY=1.72;
    final static double ROTATE_TIME=2.0;

    private static double calculate_turn_time(int degree) {
        double div_val = 90/degree;
        return ROTATE_TIME/div_val;
    }

    public static void rotateRight(DcMotor lm,int degree) {
        long startTime = System.currentTimeMillis();
        lm.setPower(1.0);
        double rtime = calculate_turn_time(degree);
        while (((System.currentTimeMillis() - startTime) * Math.pow(10,-3)) < rtime) { /*Wait for turn*/ }
        lm.setPower(0.0);
    }

    public static void rotateLeft(DcMotor rm,int degree) {
        long startTime = System.currentTimeMillis();
        rm.setPower(1.0);
        double rtime = calculate_turn_time(degree);
        while (((System.currentTimeMillis() - startTime) * Math.pow(10,-3)) < rtime) { }
        rm.setPower(0.0);
    }

    public static void goXFeetForward(DcMotor lm, DcMotor rm, double dist) {
        long startTime = System.currentTimeMillis();
        double time = dist/VELOCITY;
        lm.setPower(1.0);
        rm.setPower(1.0);
        while (((System.currentTimeMillis() - startTime) * Math.pow(10,-3)) < time) { }
        lm.setPower(0.0);
        rm.setPower(0.0);
    }

    public static void goXFeetBackwards(DcMotor lm, DcMotor rm, double dist) {
        long startTime = System.currentTimeMillis();
        double time = dist/VELOCITY;
        lm.setPower(-1.0);
        rm.setPower(-1.0);
        while (((System.currentTimeMillis() - startTime) * Math.pow(10,-3)) < time) { }
        lm.setPower(0.0);
        rm.setPower(0.0);
    }
}
