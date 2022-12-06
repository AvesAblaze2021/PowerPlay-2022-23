package org.firstinspires.ftc.teamcode.ablaze.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.ablaze.common.AblazeRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
//import java.util.Math;

@Autonomous
public class LinearSlideTest extends LinearOpMode{
  AblazeRobot robot = new AblazeRobot();
  int lastPos = 0; //Used for encoder algorithm
  double motorPower = 0.2;
  ElapsedTime runtime = new ElapsedTime();
  
  @Override
  public void runOpMode(){
    robot.initialize(hardwareMap);
    DcMotor horizontal = robot.getHorizontalSlideMotor();
    DcMotor vertical = robot.getVerticalSlideMotor();
    //moveSlideEncoder(vertical, 200, 500); //timeout in MS
    moveSlideTime(vertical, motorPower, 1000);
  }
  
  //Regular timed algorithm
  public void moveSlideTime(DcMotor slideMotor, double power, int time){
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor.setPower(power);
    sleep(time);
    slideMotor.setPower(0);
  }
  
  //Option 2 algorithm described in pseudocode - see drive
  public void moveSlideEncoder(DcMotor slideMotor, int ticks, int timeout){
    //Encoder setup - adjust tick value w/ last pos
    int newTicks = Math.abs(ticks - lastPos);
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //Starting motor
    slideMotor.setTargetPosition(newTicks);
    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideMotor.setPower(0.2);
    
    //Encoder loop
    runtime.reset();
    while(opModeIsActive() && runtime.milliseconds() < timeout && slideMotor.isBusy()){
      continue;
    }
    
    //End motion + store current tick value before reset
    slideMotor.setPower(0);
    lastPos = slideMotor.getCurrentPosition();
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }
    
}
