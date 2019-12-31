package org.firstinspires.ftc.teamcode.lib.hardware.base;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.movement.Position;
import org.firstinspires.ftc.teamcode.lib.util.GlobalVars;
import org.firstinspires.ftc.teamcode.lib.util.Math7571;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;
import static org.firstinspires.ftc.teamcode.lib.util.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.*;

public class DriveTrain {

  public RevMotor fl, fr, bl, br;


  private OpMode opMode;
  private HardwareMap hardwareMap;

  //the actual speed the robot is moving
  public static double xSpeed = 0;
  public static double ySpeed = 0;
  public static double turnSpeed = 0;

  //last update time
  private long lastUpdateTime = 0;

  public DriveTrain(){

    //initMotors();

    worldXPosition = 0;
    worldYPosition = 0;
    worldAngle_rad = Math.toRadians(0);

  }

  public void initMotors(RevMotor[] motors) {

    fl = motors[0];
    fr = motors[1];
    bl = motors[2];
    br = motors[3];

    fl.setDirection(DcMotorSimple.Direction.REVERSE);
    bl.setDirection(DcMotorSimple.Direction.REVERSE);

    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

  }

  public void driveInches(double distance){
    System.out.println("distance: " + distance);
  }

  public void drivePID(double distance){

  }

  public void turnGyro(double targetHeading){

  }

  /**converts movement_y, movement_x, movement_turn into motor powers */
  public void applyMovement() {
    long currTime = SystemClock.uptimeMillis();
    if(currTime - lastUpdateTime < 16){
      return;
    }
    lastUpdateTime = currTime;


    double fl_power_raw = movement_y-movement_turn+movement_x*1.5;
    double bl_power_raw = movement_y-movement_turn- movement_x*1.5;
    double br_power_raw = -movement_y-movement_turn-movement_x*1.5;
    double fr_power_raw = -movement_y-movement_turn+movement_x*1.5;

    //find the maximum of the powers
    double maxRawPower = Math.abs(fl_power_raw);
    if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
    if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
    if(Math.abs(fr_power_raw) > maxRawPower){ maxRawPower = Math.abs(fr_power_raw);}

    //if the maximum is greater than 1, scale all the powers down to preserve the shape
    double scaleDownAmount = 1.0;
    if(maxRawPower > 1.0){
      //when max power is multiplied by this ratio, it will be 1.0, and others less
      scaleDownAmount = 1.0/maxRawPower;
    }
    fl_power_raw *= scaleDownAmount;
    bl_power_raw *= scaleDownAmount;
    br_power_raw *= scaleDownAmount;
    fr_power_raw *= scaleDownAmount;


    //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
    fl.setPower(fl_power_raw);
    bl.setPower(bl_power_raw);
    br.setPower(br_power_raw);
    fr.setPower(fr_power_raw);
  }


  public double getXPos(){
    return worldXPosition;
  }

  public double getYPos(){
    return worldYPosition;
  }

  public double getWorldAngle_rad() {
    return worldAngle_rad;
  }

}
