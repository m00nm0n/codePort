import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Your_Username on 1/5/18.
 */

public class ColorSensorBot extends Robot {

    private ColorSensor color_sensor = null;


    public ColorSensorBot(double v, double rt, DcMotor rm, DcMotor lm, ColorSensor cs) {
        super(v,rt,rm,lm);
        color_sensor=cs;
    }

    public void wait(double time) {
        double startTime = System.currentTimeMillis();
        while (((System.currentTimeMillis() - startTime) * Math.pow(10, -3)) < time) { /*Waiting for bot to finish*/ }
    }

    public boolean isBlue() {
        int red = color_sensor.red();
        int green = color_sensor.green();
        int blue = color_sensor.blue();
        double percent_blue = ((double) (red + green + blue)) / ((double) (blue));
        if (blue != 0 && blue != 255 && percent_blue > 0.75)
            return true;
        return false;
    }

    public boolean isRed() {
        int red = color_sensor.red();
        int green = color_sensor.green();
        int blue = color_sensor.blue();
        double percent_red = ((double) (red + green + blue)) / ((double) (red));
        if(red != 0 && blue != 255 && percent_red > 0.75)
            return true;
        return false;
    }

    public String findcolor(ColorSensor color_sensor) {
        int red = color_sensor.red(), green = color_sensor.green(), blue = color_sensor.blue();
        double percentBlue = ((double) (red + green + blue) / (double)(blue));
        double percentRed = ((double) (red + green + blue) / (double)(red));
        if (percentBlue > 0.75)
            return "blue";
        if (percentRed > 0.75)
            return "red";
        return "color cannot be sensed";
    }
}
