package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SenseStonePresence extends OpMode {
    private ModernRoboticsI2cRangeSensor stonePresenceSensor;

    private boolean stonePresence = false;
    private boolean stoneFullyInStraightener = false;
    private double stoneDistanceFromBack = 0;
    //measured in inches
    private double cutoffForPossession = 2;
    //measured in inches

    public void init (){
        stonePresenceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "stonePresenceSensor");
    }

    public void loop () {
        getStonePresence();
    }

    private void getStonePresence (){
        stoneDistanceFromBack = stonePresenceSensor.getDistance(DistanceUnit.INCH);

        if (stoneDistanceFromBack < stoneDistanceFromBack) {
            stoneFullyInStraightener = true;
        }
        else {
            stoneFullyInStraightener = false;
        }
    }
}
