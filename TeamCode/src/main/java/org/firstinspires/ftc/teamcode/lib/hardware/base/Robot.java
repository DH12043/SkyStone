package org.firstinspires.ftc.teamcode.lib.hardware.base;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;
import org.firstinspires.ftc.teamcode.lib.movement.Position;
import org.firstinspires.ftc.teamcode.lib.util.OpMode7571;
import org.firstinspires.ftc.teamcode.lib.util.PIDController;
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;
//import org.openftc.revextensions2.RevBulkData;
//import org.openftc.revextensions2.RevExtensions2;

import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.worldYPosition;

@TeleOp
public class Robot extends OpMode{

  private PIDController pidRotate, pidDrive;

  private boolean isAuto = false;

//  private RevBulkData revExpansionMasterBulkData;
//
//  private ExpansionHubEx revMaster;
//  private ExpansionHubEx revSlave;

  private Gamepad mainGp, auxGp;

  public Position position;

  private RevMotor[] motors;


  DriveTrain dt = new DriveTrain();

  private void initPID(){

    pidRotate = new PIDController(0.001, 0, 0);
    pidDrive = new PIDController(0.001, 0, 0);

    pidRotate.setOutputLimits(1);
    pidRotate.setSetpoint(0);

  }


  @Override
  public void init() {

//    RevExtensions2.init();
//
//    revMaster = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");
//    revSlave = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 5");
//
//    motors = new RevMotor[]{new RevMotor((ExpansionHubMotor) hardwareMap.get("fl"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("fr"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("bl"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("br"),true)};


    dt.initMotors(motors);
    initPID();

  }

  @Override
  public void loop() {

    getRevBulkData();

    telemetry.addLine("got bulk data!");

    if(!isAuto){
      getGamepads(gamepad1, gamepad2);
      telemetry.addLine("got gamepads!");
    }


    dt.applyMovement();
    telemetry.addLine("movements applied!");

    MyPosition.giveMePositions(
        dt.fr.getCurrentPosition(),
        dt.fl.getCurrentPosition(),
        dt.bl.getCurrentPosition());

    telemetry.addLine("positions set!");

    telemetry.addLine("wx: " + worldXPosition);
    telemetry.addLine("wy: " + worldYPosition);
    telemetry.addLine("wa: " + worldAngle_rad);

    telemetry.update();


  }

  public void getGamepads(Gamepad main, Gamepad aux){

    this.mainGp = main;
    this.auxGp = aux;

  }

  /**
   * Gets all the data from the expansion hub in one command to increase loop times
   */
  public void getRevBulkData() {
//        boolean needToPollMaster = !AutoFeeder.canPollMasterAtLowerRate ||
//            currTimeMillis-lastUpdateMasterTime > 300;
//        if(needToPollMaster){
//    RevBulkData newDataMaster;
//    try{
//      newDataMaster = revMaster.getBulkInputData();
//      if(newDataMaster != null){
//        revExpansionMasterBulkData = newDataMaster;
//      }
//    }catch(Exception e){
//      //don't set anything if we get an exception
//    }
//
//
//    for(RevMotor revMotor : motors) {
//      if (revMotor == null) {
//        continue;
//      }
//      if (revExpansionMasterBulkData != null) {
//        revMotor.setEncoderReading(revExpansionMasterBulkData.getMotorCurrentPosition(revMotor.myMotor));
//      }
//    }

  }
}