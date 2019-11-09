package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "PowerSurgeTeleOp")
public class PowerSurgeTeleOp extends OpMode {

    public static final double DEADZONE = 0.15;

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor IntakeMotor;
    private DcMotor LiftMotor;
    private Servo LeftServo;
    private Servo RightServo;

    private int foundationatorState = 0;
    private int intakeState = 1;
    private boolean outputButton;
    private boolean intakeButton;
    private boolean firstPressDpadUp = true;
    private boolean foundationatorFirstPress = true;

    @Override
    public void init() {
        initializeVerticalLift();
        initializeFoundationator();
        initializeDriveTrain();
        initializeIntakeMechanism();
    }

    @Override
    public void loop() {
        checkVerticalLift();
        checkFoundationator();
        checkDriveTrain();
        checkIntakeMechanism();
    }

    //
    // VERTICAL LIFT
    //

    public void initializeVerticalLift() {
        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
    }

    public void checkVerticalLift() {
    }

    //
    // FOUNDATIONATOR
    //

    public void initializeFoundationator() {
        LeftServo = hardwareMap.servo.get("LeftServo");
        RightServo = hardwareMap.servo.get("RightServo");
    }

    public void checkFoundationator() {
        boolean foundationatorToggle = gamepad1.right_bumper;

        if (foundationatorToggle == true) {
            if (foundationatorFirstPress == true) {
                if (foundationatorState == 1) {
                    servosDown();
                    foundationatorState = 0;
                }
                else {
                    servosUp();
                    foundationatorState = 1;
                }
                foundationatorFirstPress = false;
            }
        }
        else {
            foundationatorFirstPress = true;
        }
    }

    public void servosDown() {
        LeftServo.setPosition(.6);
        RightServo.setPosition(.4);
    }

    public void servosUp() {
        LeftServo.setPosition(1);
        RightServo.setPosition(0);
        LeftServo.setPosition(1.0);
        RightServo.setPosition(0.0);
    }

    //
    // DRIVE TRAIN
    //

    public void initializeDriveTrain() {
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
    }

    public void checkDriveTrain() {
        double forwardButton = gamepad1.left_stick_y;
        double sidewaysButton = gamepad1.left_stick_x;
        double spinningButton = gamepad1.right_stick_x;

        forwardButton = DeadModifier(forwardButton);
        sidewaysButton = DeadModifier(sidewaysButton);
        spinningButton = DeadModifier(spinningButton);

        Drive(forwardButton, sidewaysButton, spinningButton);
    }

    public double  DeadModifier(double joystickValue) {
        if(joystickValue < DEADZONE && joystickValue > -DEADZONE)
            return 0;
        else {
            return joystickValue;
        }
    }

    public void Drive(double DZForwardButton, double DZSidewaysButton, double DZSpinningButton) {
        BackRight.setPower(-DZSidewaysButton - DZForwardButton - DZSpinningButton);
        BackLeft.setPower(-DZSidewaysButton + DZForwardButton - DZSpinningButton);
        FrontRight.setPower(DZSidewaysButton - DZForwardButton - DZSpinningButton);
        FrontLeft.setPower(DZSidewaysButton + DZForwardButton - DZSpinningButton);
    }

    public void initializeIntakeMechanism() {
        IntakeMotor = hardwareMap.dcMotor.get("Intake");
    }

    public void checkIntakeMechanism() {
        intakeButton = gamepad2.dpad_up;
        outputButton = gamepad2.dpad_down;

        intake(intakeButton);
    }

    public void intake(boolean intakeButton) {
        if (intakeButton) {
            if (firstPressDpadUp) {
                if (intakeState == 1) {
                    intakeState = 0;
                }
                else {
                    intakeState = 1;
                }
                firstPressDpadUp = false;
            }
        }
        else {
            firstPressDpadUp = true;
        }
        if (outputButton) {
            IntakeMotor.setPower(-1);
        }
        else {
            if (intakeState == 1) {
                IntakeMotor.setPower(1);
            }
            else if (intakeState == 0) {
                IntakeMotor.setPower(0);
            }
        }
    }
}