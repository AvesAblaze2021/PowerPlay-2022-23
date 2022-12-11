package org.firstinspires.ftc.teamcode.ablaze.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class PracticeRobot
{
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private WebcamName webCam;
    //private DcMotor horizontalSlideMotor;
    //private DcMotor verticalSlideMotor;

    private double defaultPower = 0.5;
    private BNO055IMU imu;
    
    public void initialize(HardwareMap hwMap){
      leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontMotor"); // Port: 0
      rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontMotor"); // Port: 1
      rightBackDrive = hwMap.get(DcMotor.class, "rightBackMotor"); // Port: 2
      leftBackDrive = hwMap.get(DcMotor.class, "leftBackMotor"); // Port: 3
      //horizontalSlideMotor =  hwMap.get(DcMotor.class, "horizontalSlideMotor");//Port X0
      //verticalSlideMotor =  hwMap.get(DcMotor.class, "verticalSlideMotor");//Port X1
      
      imu = hwMap.get(BNO055IMU.class, "imu1"); //Port I2 Bus 0
      
      leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
      rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
      rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
      leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

      //horizontalSlideMotor.setDirection(DcMotor.Direction.FORWARD);
      //verticalSlideMotor.setDirection(DcMotor.Direction.FORWARD);

      //horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public DcMotor getRightFrontDrive() {
        return rightFrontDrive;
    }

    //public DcMotor getHorizontalSlideMotor() { return horizontalSlideMotor; }

    //public DcMotor getVerticalSlideMotor() { return verticalSlideMotor; }
    
    public double getDefaultPower(){
        return defaultPower;
    }
}
