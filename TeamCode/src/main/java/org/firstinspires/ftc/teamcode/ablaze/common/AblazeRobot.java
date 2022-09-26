package org.firstinspires.ftc.teamcode.FreightFrenzy202122.ablaze.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class AblazeRobot
{
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private WebcamName webCam;

    private double defaultPower = 0;
    private BNO055IMU imu;

    private double defaultMinPosition = 0.3;
    private double defaultMaxPosition = 0.5;
    
    private double signalPosition = 0.5;


    public void initialize(HardwareMap hwMap){
      leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontMotor"); // Port: 0
      rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontMotor"); // Port: 1
      rightBackDrive = hwMap.get(DcMotor.class, "rightBackMotor"); // Port: 2
      leftBackDrive = hwMap.get(DcMotor.class, "leftBackMotor"); // Port: 3
      
      imu = hwMap.get(BNO055IMU.class, "imu1"); //Port I2 Bus 0
      webCam = hwMap.get(WebcamName.class, "VuforiaCam"); // Port: 3
      
      leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
      rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
      rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
      leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
}
