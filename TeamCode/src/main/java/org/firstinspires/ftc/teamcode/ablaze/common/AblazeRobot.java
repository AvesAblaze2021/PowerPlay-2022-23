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
    private DcMotor verticalSlideMotor = null;
    private Servo rotationServo = null;
    private Servo clawServo = null;
    private WebcamName webCam;
    private BNO055IMU  imu;

    private double defaultPower = 0;

    public void initialize(HardwareMap hwMap) {
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontMotor"); // Port: 0
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontMotor"); // Port: 1
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackMotor"); // Port: 2
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackMotor"); // Port: 3
        verticalSlideMotor =  hwMap.get(DcMotor.class, "verticalSlideMotor");//Port X1
        rotationServo = hwMap.get(Servo.class, "rotationServo");
        clawServo = hwMap.get(Servo.class, "scissorServo");

        imu = hwMap.get(BNO055IMU.class, "imu1"); //Port I2 Bus 0
        webCam = hwMap.get(WebcamName.class, "Webcam 1"); // Port: 3

        rotationServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        verticalSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    public DcMotor getVerticalSlideMotor() { return verticalSlideMotor; }

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

    public Servo getRotationServo() {
        return rotationServo;
    }

    public Servo getClawServo(){return clawServo;}
}
