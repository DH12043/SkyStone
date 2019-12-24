package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class StraightenerBasedOnCompletionOrOverride extends OpMode {
    private ModernRoboticsI2cRangeSensor OrientationSensor;
    private Servo OrientationServoLeft;
    private Servo OrientationServoRight;
    private String stoneOrientation = "empty";
    private String lCurrentPosition = "lDisengage";
    private String rCurrentPosition = "rDisengage";
    private double orientDistance = 0;
    private double lDisengage = .5;
    private double rDisengage = 1;
    private double lEngage = 1;
    private double rEngage = .5;
    private boolean readyToGrab = false;
    private boolean manualReset = false;

    //State variables
    private boolean stoneFullyInStraightener = false;
    private boolean straightenerBusy = false;

    //delay vars
    private boolean firstRightRun = true;
    private double startRightTime = 0;
    private double currentRightTime = 0;
    private double actualRightTime = 0;
    private double lastActualRightTime = 0;
    private double rightOrientCheck = 1;

    private boolean firstLeftRun = true;
    private double startLeftTime = 0;
    private double currentLeftTime = 0;
    private double actualLeftTime = 0;
    private double lastActualLeftTime = 0;

    private double targetTime = .5;

    public void init() {
        OrientationSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "OrientationSensor");
        OrientationServoLeft = hardwareMap.get(Servo.class, "OrientationServoLeft");
        OrientationServoRight = hardwareMap.get(Servo.class, "OrientationServoRight");
    }

    public void loop () {
        stoneFullyInStraightener = gamepad1.a;

        orientStone();
        manualOverride();

    }

    private void senseOrientation() {
        orientDistance = OrientationSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("sensor distance", orientDistance);

        // StoneOrientation is assigned relative to the side of the robot that the studs of the stone
        // are on when looking at the robot from the back.
        // this is true for the servos as well; left and right are assigned relative to the back
        if(!straightenerBusy) {
            if (orientDistance > .25 && orientDistance <= .75) {
                stoneOrientation = "left";
            } else if (orientDistance > .75 && orientDistance <= 2.3) {
                stoneOrientation = "center";
            } else if (orientDistance > 4) {
                stoneOrientation = "empty";
            } else {
                stoneOrientation = "right";
            }
            telemetry.addData("orientPosition:  ", stoneOrientation);
        }
    }

    private void orientStone() {
        senseOrientation();

        if (stoneFullyInStraightener) {
            if (stoneOrientation.equals("right")) {
                runLeftServo();
            }

            if (stoneOrientation.equals("left")) {
                runRightServo();
            }

            if (stoneOrientation.equals("center")) {
                readyToGrab = true;
            }
            else {
                readyToGrab = false;
            }
        }
    }

 /*   private void engageRightServo () {
        OrientationServoRight.setPosition(rEngage);
        rCurrentPosition = "rEngage";
    }

    private void disengageRightServo() {
        OrientationServoRight.setPosition(rDisengage);
        rCurrentPosition = "rDisengage";
    }

    private void engageLeftServo () {
        OrientationServoLeft.setPosition(lEngage);
        lCurrentPosition = "lEngage";
    }

    private void disengageLeftServo() {
        OrientationServoLeft.setPosition(lDisengage);
        lCurrentPosition = "lDisengage";
    }*/

    private void manualOverride() {
        if (gamepad2.right_bumper) {
            manualReset = true;
        }
    }

    private void runRightServo() {
        if(firstRightRun && !straightenerBusy) {
            OrientationServoRight.setPosition(rEngage);
            startRightTime = getRuntime();
            firstRightRun = false;
            straightenerBusy = true;
        }
        currentRightTime = getRuntime();
        actualRightTime = (currentRightTime-startRightTime);
        telemetry.addData("actualRightTime", actualRightTime);
        telemetry.addData("target time", targetTime);

        if(actualRightTime > targetTime) {
            OrientationServoRight.setPosition(rDisengage);
            telemetry.addData("is ready for return", "yes");
            firstRightRun = true;
            straightenerBusy = false;
        }
//        lastActualRightTime = actualRightTime;
    }

    private void runLeftServo() {
        if(firstLeftRun && !straightenerBusy) {
            OrientationServoLeft.setPosition(lEngage);
            startLeftTime = getRuntime();
            firstLeftRun = false;
            straightenerBusy = true;
        }
        currentLeftTime = getRuntime();
        actualLeftTime = (currentLeftTime-startLeftTime);
        telemetry.addData("actualLeftTime", actualLeftTime);
        telemetry.addData("target time", targetTime);

        if(actualLeftTime > targetTime) {
            OrientationServoLeft.setPosition(lDisengage);
            telemetry.addData("is ready for return", "yes");
            firstLeftRun = true;
            straightenerBusy = false;
        }
//        lastActualRightTime = actualRightTime;
    }

}
