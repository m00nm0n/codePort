/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.teamcode.BasicMovements;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Varsity Auto Blue Left", group="Linear Opmode")
@Disabled
public class VarsityAutoBlueLeft extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor bepisMotor = null;
    private DcMotor elevatorMotor = null;
    private DcMotor tiltMotor = null;
    private DcMotor pusher = null;
    private VarsityRobot robot = new VarsityRobot();//(1.72,2.0,rightDrive,leftDrive,bepisMotor,tiltMotor);
    /* velocity=1.72, rotate_time=2.0 */


    private double time = 1.0;

    public void wait(double time) {
        double startTime = System.currentTimeMillis();
        while (((System.currentTimeMillis() - startTime) * Math.pow(10, -3)) < time) { /*Waiting for bot to finish*/ }
    }

    public String findColor(ColorSensor colorSensor) {
        int red = colorSensor.red(), green = colorSensor.green(), blue = colorSensor.blue();
        double percentBlue = ((double) (red+green+blue) / (double)(blue));
        double percentRed = ((double) (red+green+blue) / (double)(red));
        if(percentBlue > 0.75)
            return "blue";
        if(percentRed > 0.75)
            return "red";
        return "color cannot be sensed";
    }

    @Override
    public void runOpMode() {

        // figure out the pictogram
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";
        RelicRecoveryVuMark vuMarkPosition = PictogramRecognition.getPictogramLocation(parameters);
        if (vuMarkPosition != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMarkPosition);
            telemetry.update();
        }

        // move that pictogram stuff

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftDrive = hardwareMap.get(DcMotor.class, "left motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right motor");
        bepisMotor = hardwareMap.get(DcMotor.class, "bepis motor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator motor");
        tiltMotor = hardwareMap.get(DcMotor.class, "tilt motor");
        pusher = hardwareMap.get(DcMotor.class, "pusher");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        bepisMotor.setDirection(DcMotor.Direction.FORWARD);
        tiltMotor.setDirection(DcMotor.Direction.FORWARD);

        robot.rotateRight(90);
        robot.goXFeetForward(2);
        robot.rotateLeft(90);
        robot.goXFeetForward(0.2);
    }
}


