package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class GiantWaffle extends OpMode {

    private Servo _lFoundationator;
    private Servo _rFoundationator;
    private boolean _lastWaffleState = false;

    @Override
    public void init() {

        _lFoundationator = hardwareMap.servo.get("Left Foundationator");
        _rFoundationator = hardwareMap.servo.get("Right Foundationator");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void loop() {
        boolean currentWaffleState = gamepad1.a;
        if (_lastWaffleState == currentWaffleState){
            return;
        }
        else if(_lastWaffleState == false && currentWaffleState == true){
            raiseFoundationator ();
        }
        else{
            lowerFoundationator();
        }

    }

    private void raiseFoundationator() {
        _lFoundationator.setPosition(.25);
        _rFoundationator.setPosition(.25);
    }

    private void lowerFoundationator() {
        _lFoundationator.setPosition(0);
        _rFoundationator.setPosition(0);

    }
}
