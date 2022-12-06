package org.firstinspires.ftc.teamcode.ablaze.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class AblazeRobot {
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private static DcMotor horizontalSlideMotor = null;
    private static DcMotor verticalSlideMotor = null;
    private WebcamName webCam;
    private BNO055IMU  imu;

    private double defaultPower = 0;

    public void initialize(HardwareMap hwMap) {
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontMotor"); // Port: 0
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontMotor"); // Port: 1
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackMotor"); // Port: 2
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackMotor"); // Port: 3
        horizontalSlideMotor =  hwMap.get(DcMotor.class, "horizontalSlideMotor");//Port X0
        verticalSlideMotor =  hwMap.get(DcMotor.class, "verticalSlideMotor");//Port X1

        imu = hwMap.get(BNO055IMU.class, "imu1"); //Port I2 Bus 0
        webCam = hwMap.get(WebcamName.class, "VuforiaCam"); // Port: 3

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        horizontalSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        verticalSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public WebcamName getWebCam() {
        return webCam;
    }

    public DcMotor getLeftBackDrive() {
        return leftBackDrive;
    }

    public DcMotor getLeftFrontDrive() {
        return leftFrontDrive;
    }

    public DcMotor getRightBackDrive() {
        return rightBackDrive;
    }

    public DcMotor getRightFrontDrive() { return rightFrontDrive; }

    public static DcMotor getHorizontalSlideMotor() { return horizontalSlideMotor; }

    public static DcMotor getVerticalSlideMotor() { return verticalSlideMotor; }

    public double getDefaultPower() {
        return defaultPower;
    }
    
    public BNO055IMU getImu()  {
         return imu;   
    }

    public void driveTime(double power) {
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
    }

    public void stop() {
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

}
