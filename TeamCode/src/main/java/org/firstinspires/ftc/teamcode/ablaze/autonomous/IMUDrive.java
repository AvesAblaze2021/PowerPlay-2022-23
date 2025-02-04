
package org.firstinspires.ftc.teamcode.ablaze.autonomous;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;


public class IMUDrive {
    Orientation lastAngles = new Orientation();
    double globalAngle;
    private AblazeRobot ablazeRobot;


    public void initialize(AblazeRobot ablazeRobot){
        this.ablazeRobot = ablazeRobot;
        initializeImu();
    }

    public void initializeImu() {
        //Initialize IMU parameters
        //ablazeRobot = new AblazeRobot();
        //ablazeRobot.initialize(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        ablazeRobot.getImu().initialize(parameters);
    }

    public void resetAngle() {
        lastAngles = ablazeRobot.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = ablazeRobot.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //Use this method
    public double checkDirection() {
        /*The gain value determines how sensitive the correction is to direction changes.
         *You will have to experiment with your robot to get small smooth direction changes
         *to stay on a straight line.

         */

        double correction, angle, gain = 0.01;

        angle = getAngle();
        if (angle == 0) {
            correction = 0;             //No adjustment.
        }
        else {
            correction = -angle;        //Reverse sign of angle for correction.
        }

        correction = correction * gain;

        return correction;
    }

    public void rotate(double degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        ablazeRobot.getLeftFrontDrive().setPower(leftPower);
        //aRobot.getLeftBackDrive().setPower(leftPower);
        ablazeRobot.getRightFrontDrive().setPower(rightPower);
        // aRobot.getRightBackDrive().setPower(rightPower);

        //Rotate until turn is completed.
        if (degrees < 0) {
            //On right turn we have to get off zero first
            while (getAngle() == 0) {
            }
            while (getAngle() > degrees) {
            }
        } else {   //Left turn
            while (getAngle() < degrees) {
            }
        }

        //Turn the motors off
        ablazeRobot.getLeftFrontDrive().setPower(0);
        ablazeRobot.getLeftBackDrive().setPower(0);
        ablazeRobot.getRightFrontDrive().setPower(0);
        ablazeRobot.getRightBackDrive().setPower(0);

        //Reset angle tracking on new heading.
        resetAngle();
    }

}
