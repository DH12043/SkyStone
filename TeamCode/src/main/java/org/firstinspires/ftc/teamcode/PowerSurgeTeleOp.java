package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "PowerSurgeTeleOp")
public class PowerSurgeTeleOp extends OpMode {

    public static final double DEADZONE = 0.15;

    DcMotor verticalRight, verticalLeft, horizontal;

    String verticalLeftEncoderName = "FrontLeft";
    String verticalRightEncoderName = "BackRight";
    String horizontalEncoderName = "BackLeft";

    final double COUNTS_PER_INCH = 307.699557;
    private static final int PreFoundationXPosition = 36;
    private static final int PreFoundationYPosition = 95;
    private static final int FoundationXPosition = 48;
    private static final int FoundationYPosition = 107;
    private static final int BuildSiteXPosition = 9;
    private static final int BuildSiteYPosition = 111;
    private static final int ParkLineXPosition = 9;
    private static final int ParkLineYPosition = 72;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor IntakeMotor;
    private DcMotor LiftMotor;
    private Servo lFoundationator;
    private Servo rFoundationator;
    private Servo OrientationServoLeft;
    private Servo OrientationServoRight;
    private CRServo IntakeAssistServo;
    private Servo IntakeReleaseServo;

    private ModernRoboticsI2cRangeSensor OrientationSensor;

    private int intakeState = 0;
    private boolean outputButton;
    private boolean intakeButton;
    private boolean firstPressDpadUp = true;

    private boolean liftEncoderState = true;
    private boolean firstPressa = true;

    private boolean lastWaffleState = false;
    private boolean isWaffleStateRaised = false;
    private double foundationatorPosition = .335;

    static final double countsPerMotor          = 10000 ;                           //TODO Change
    static final double gearReduction           = 1.0 ;                             //TODO Change
    static final double wheelDiameter           = 4.0 ;                             //TODO Change
    static final double countsPerInch           = (countsPerMotor * gearReduction) /
            (wheelDiameter * Math.PI);
    static final double spinInchesPerDegrees    = (15.375 * Math.PI) / 360;
    static final double rotateDegrees           = (30.75 * Math.PI) / 360;
    static final double spinCountsPerDegree     = (countsPerInch * spinInchesPerDegrees);

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
    private boolean stoneFullyInStraightener = false;
    private boolean straightenerBusy = false;
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

    private boolean firstPressb = true;
    private int intakeReleaseState = 1;

    @Override
    public void init() {
        telemetry.addData("Version Number", "12-22-19 1016pm");
        initializeVerticalLift();
        initializeFoundationator();
        initializeDriveTrain();
        initializeOdometry();
        initializeIntakeMechanism();
        initializeStraightener();
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        startOdometry();
        telemetry.addData("Status", "Odometry System has started");
        telemetry.update();
    }

    @Override
    public void loop() {
        checkVerticalLift();
        checkFoundationator();
        checkOdometry();
        checkDriveTrain();
        checkIntakeMechanism();
        checkStraightener();
        telemetry.update();
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
        boolean LiftManualToggleButton = gamepad1.y;

        if (LiftManualToggleButton) {
            if (firstPressa) {
                liftEncoderState =! liftEncoderState;
                firstPressa = false;
            }
        }
        else {
            firstPressa = true;
        }

        if (liftEncoderState) {
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("LiftMotor","LiftMotor.getCurrentPosition() - LiftMotor.getTargetPosition()");
            //setting the Target position if we press the right bumper
            if (LiftUpButton) {
                LiftMotor.setTargetPosition((int)(LiftMotor.getTargetPosition() + (4 * countsPerInch)));
            }
            //setting the Target position if we press the left Bumper
            else if (LiftDownButton) {
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
        else {
            LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LiftMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }

    //
    // FOUNDATIONATOR
    //

    public void initializeFoundationator() {
        lFoundationator = hardwareMap.servo.get("lFoundationator");
        rFoundationator = hardwareMap.servo.get("rFoundationator");
        lFoundationator.setPosition(foundationatorPosition);
        rFoundationator.setPosition(0);
    }

    public void checkFoundationator() {
        boolean currentWaffleState = gamepad1.a;

        if (lastWaffleState == currentWaffleState){
            return;
        }
        else if(!lastWaffleState && currentWaffleState){
            if(isWaffleStateRaised) {
                lowerFoundationator();
            } else {
                raiseFoundationator();
            }
        }
        lastWaffleState = currentWaffleState;
    }

    private void raiseFoundationator() {
        lFoundationator.setPosition(0);
        rFoundationator.setPosition(foundationatorPosition);
        isWaffleStateRaised = true;
        telemetry.addData("Raising Foundationator", "");
    }

    private void lowerFoundationator() {
        lFoundationator.setPosition(foundationatorPosition);
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
    // Odometry
    //

    public void initializeOdometry() {
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startOdometry() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
    }

    public void checkOdometry() {
        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

        telemetry.addData("Vertical Left Encoder", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Encoder", verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Encoder", horizontal.getCurrentPosition());

        telemetry.addData("Thread Active", positionThread.isAlive());
    }

    double getDistanceFromCoordinates(double x, double y, OdometryGlobalCoordinatePosition position) {
        double deltaX = x - position.returnXCoordinate();
        double deltaY = y - position.returnYCoordinate();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    //
    // INTAKE
    //

    public void initializeIntakeMechanism() {
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        IntakeAssistServo = hardwareMap.crservo.get("IntakeAssistServo");
        IntakeReleaseServo = hardwareMap.servo.get("IntakeReleaseServo");
        IntakeMotor.setPower(0);
        IntakeAssistServo.setPower(0);
        IntakeReleaseServo.setPosition(.6);
    }

    public void checkIntakeMechanism() {
        intakeButton = gamepad1.dpad_up;
        outputButton = gamepad1.dpad_down;

        intake(intakeButton);

        if (gamepad1.x) {
            if (firstPressb) {
                if (intakeReleaseState == 1) {
                    intakeReleaseState = 0;
                }
                else {
                    intakeReleaseState = 1;
                }
                firstPressb = false;
            }
        }
        else {
            firstPressb = true;
        }

        if (intakeReleaseState == 0) {
            IntakeReleaseServo.setPosition(.15);
        }
        else {
            IntakeReleaseServo.setPosition(.6);
        }


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
            IntakeAssistServo.setPower(1);
        }
        else {
            if (intakeState == 1) {
                IntakeMotor.setPower(1);
                IntakeAssistServo.setPower(-1);
            }
            else if (intakeState == 0) {
                IntakeMotor.setPower(0);
                IntakeAssistServo.setPower(0);
            }
        }
    }

    //
    // Straightener
    //

    public void initializeStraightener() {
        OrientationSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "OrientationSensor");
        OrientationServoLeft = hardwareMap.get(Servo.class, "OrientationServoLeft");
        OrientationServoRight = hardwareMap.get(Servo.class, "OrientationServoRight");
        OrientationServoLeft.setPosition(lDisengage);
        OrientationServoRight.setPosition(rDisengage);
    }

    public void checkStraightener() {
        stoneFullyInStraightener = gamepad1.b;
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
    }
}