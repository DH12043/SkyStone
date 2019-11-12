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
    static final double countsPerMotor          = 1120 ;
    static final double gearReduction           = 1.0 ;
    static final double wheelDiameter           = 4.0 ;
    static final double countsPerInch           = (countsPerMotor * gearReduction) /
            (wheelDiameter * Math.PI);
    static final double spinInchesPerDegrees    = (15.375 * Math.PI) / 360;
    static final double rotateDegrees           = (30.75 * Math.PI) / 360;
    static final double spinCountsPerDegree     = (countsPerInch * spinInchesPerDegrees);

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
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void checkVerticalLift() {
        boolean LiftUpButton = gamepad1.right_bumper;
        boolean LiftDownButton = gamepad1.left_bumper;

        if (LiftMotor.getCurrentPosition() > (LiftMotor.getTargetPosition() + 100) && LiftMotor.getCurrentPosition() < (LiftMotor.getTargetPosition() + 500)) {
            LiftMotor.setPower(.25);
        }
        else if (LiftMotor.getCurrentPosition() < (LiftMotor.getTargetPosition() - 100) && LiftMotor.getCurrentPosition() > (LiftMotor.getTargetPosition() - 500)) {
            LiftMotor.setPower(-.25);
        }
        else if (LiftUpButton == true) {
            LiftMotor.setTargetPosition((int)(LiftMotor.getCurrentPosition() + (4 * countsPerInch)));
            LiftMotor.setPower(.5);
        }
        else if (LiftDownButton == true) {
            LiftMotor.setTargetPosition((int)(LiftMotor.getCurrentPosition() + (-4 * countsPerInch)));
            LiftMotor.setPower(-.5);
        }
        else if (LiftMotor.getCurrentPosition() == LiftMotor.getTargetPosition()) {
            LiftMotor.setPower(0);
        }
    }

    //
    // FOUNDATIONATOR
    //

    public void initializeFoundationator() {
        LeftServo = hardwareMap.servo.get("LeftServo");
        RightServo = hardwareMap.servo.get("RightServo");
    }

    public void checkFoundationator() {
        boolean foundationatorToggle = gamepad1.x;

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