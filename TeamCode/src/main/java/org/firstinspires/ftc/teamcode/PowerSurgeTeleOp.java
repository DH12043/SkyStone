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
    private Servo lFoundationator;
    private Servo rFoundationator;

    private int intakeState = 0;
    private boolean outputButton;
    private boolean intakeButton;
    private boolean firstPressDpadUp = true;
    private boolean lastWaffleState = false;
    private boolean isWaffleStateRaised = false;
    static final double countsPerMotor          = 10000 ;                           //TODO Change
    static final double gearReduction           = 1.0 ;                             //TODO Change
    static final double wheelDiameter           = 4.0 ;                             //TODO Change
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
        LiftMotor.setPower(0);
        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void checkVerticalLift() {
        boolean LiftUpButton = gamepad1.right_bumper;
        boolean LiftDownButton = gamepad1.left_bumper;

        telemetry.addData("LiftMotor","LiftMotor.getCurrentPosition() - LiftMotor.getTargetPosition()");

        //setting the Target position if we press the right bumper
        if (LiftUpButton == true) {
            LiftMotor.setTargetPosition((int)(LiftMotor.getTargetPosition() + (4 * countsPerInch)));
        }
        //setting the Target position if we press the left Bumper
        else if (LiftDownButton == true) {
            LiftMotor.setTargetPosition((int)(LiftMotor.getTargetPosition() + (-4 * countsPerInch)));
        }
        //setting the motor to .5 if we are lower than our Target Position
        if (LiftMotor.getTargetPosition() > (LiftMotor.getCurrentPosition())) {
            LiftMotor.setPower(.5);
        }
        //setting our power to -.5 if we are higher than our Target Position
        else if (LiftMotor.getTargetPosition() < (LiftMotor.getCurrentPosition())) {
            LiftMotor.setPower(-.5);
        }
        //setting the motor to brake if we are at our Target Position
        else {
            LiftMotor.setPower(0);
        }
    }

    //
    // FOUNDATIONATOR
    //

    public void initializeFoundationator() {
        lFoundationator = hardwareMap.servo.get("lFoundationator");
        rFoundationator = hardwareMap.servo.get("rFoundationator");
    }

    public void checkFoundationator() {
        boolean currentWaffleState = gamepad1.a;

        if (lastWaffleState == currentWaffleState){
            return;
        }
        else if(lastWaffleState == false && currentWaffleState == true){
            if(isWaffleStateRaised == true) {
                lowerFoundationator();
            } else {
                raiseFoundationator();
            }
        }
        lastWaffleState = currentWaffleState;
    }

    private void raiseFoundationator() {
        lFoundationator.setPosition(0);
        rFoundationator.setPosition(.27);
        isWaffleStateRaised = true;
        telemetry.addData("Raising Foundationator", "");
    }

    private void lowerFoundationator() {
        lFoundationator.setPosition(.27);
        rFoundationator.setPosition(0);
        isWaffleStateRaised = false;
        telemetry.addData("Lowering Foundationator", "");
    }

    //
    // DRIVE TRAIN
    //

    public void initializeDriveTrain() {
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void checkDriveTrain() {
        double forwardButton = gamepad1.left_stick_y;
        double sidewaysButton = gamepad1.left_stick_x;
        double spinningButton = gamepad1.right_stick_x;

        BackRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

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
        DZSpinningButton = -DZSpinningButton;
        FrontRight.setPower(-DZSidewaysButton - DZForwardButton + DZSpinningButton);
        FrontLeft.setPower(-DZSidewaysButton + DZForwardButton + DZSpinningButton);
        BackRight.setPower(DZSidewaysButton - DZForwardButton + DZSpinningButton);
        BackLeft.setPower(DZSidewaysButton + DZForwardButton + DZSpinningButton);
    }

    //
    // INTAKE
    //

    public void initializeIntakeMechanism() {
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
    }

    public void checkIntakeMechanism() {
        intakeButton = gamepad1.dpad_up;
        outputButton = gamepad1.dpad_down;

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