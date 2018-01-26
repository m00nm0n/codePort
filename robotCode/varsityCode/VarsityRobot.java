import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Your_Username on 11/16/17.
 */

public class VarsityRobot extends Robot {

    private DcMotor arm;
    private DcMotor tilt;
    private ColorSensor colorSensor;


    public VarsityRobot(double v, double rt, DcMotor rm, DcMotor lm, DcMotor a, DcMotor t, ColorSensor colorSensor /* extra motors */) {
        super(v,rt,rm,lm);
        arm=a;
        tilt=t;
        this.colorSensor = colorSensor;
    }

    public VarsityRobot() {}

    public String findColor() {
        int red = colorSensor.red(), green = colorSensor.green(), blue = colorSensor.blue();
        double percentBlue = ((double) (red+green+blue) / (double)(blue));
        double percentRed = ((double) (red+green+blue) / (double)(red));
        if(percentBlue > 0.75)
            return "blue";
        if(percentRed > 0.75)
            return "red";
        return "color cannot be sensed";
    }

}
