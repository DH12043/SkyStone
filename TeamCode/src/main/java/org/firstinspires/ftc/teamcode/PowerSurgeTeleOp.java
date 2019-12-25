package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;


@TeleOp(name = "PowerSurgeTeleOp")
public class PowerSurgeTeleOp extends OpMode {

    public static final double DEADZONE = 0.15;

    DcMotor verticalRight, verticalLeft, horizontal;

    String verticalLeftEncoderName = "FrontLeft";
    String verticalRightEncoderName = "BackRight";
    String horizontalEncoderName = "BackLeft";

    private double RobotXPosition;
    private double RobotYPosition;
    private double RobotRotation;

    private double StartingXPosition;
    private double StartingYPosition;
    private double StartingRotation;

    private File startingXpositionFile = AppUtil.getInstance().getSettingsFile("startingXposition.txt");
    private File startingYpositionFile = AppUtil.getInstance().getSettingsFile("startingYposition.txt");
    private File startingθpositionFile = AppUtil.getInstance().getSettingsFile("startingθposition.txt");

    final double COUNTS_PER_INCH = 307.699557;
    private static final int PreFoundationXPosition = 36;
    private static final int PreFoundationYPosition = 95;
    private static final int FoundationXPosition = 48;
    private static final int FoundationYPosition = 107;
    private static final int BuildSiteXPosition = 9;
    private static final int BuildSiteYPosition = 111;
    private static final int ParkLineXPosition = 9;
    private static final int ParkLineYPosition = 72;

    private static final double GRABBERSERVOCLOSEDPOSITION = 0;
    private static final double GRABBERSERVOOPENPOSITION = .5;

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
    private Servo GrabberServo;
    private Servo OrientStoneServo;
    private Servo MoveArmServo;
    private Servo OrientationServoLeft;
    private Servo OrientationServoRight;
    private CRServo IntakeAssistServo;
    private Servo IntakeReleaseServo;

    private ModernRoboticsI2cRangeSensor OrientationSensor;
    private ModernRoboticsI2cRangeSensor StonePresenceSensor;

    private int intakeState = 0;
    private boolean outputButton;
    private boolean intakeButton;
    private boolean firstPressDpadUp = true;

    private boolean liftEncoderState = true;
    private boolean firstPressa = true;
    private boolean firstLiftUpButton;
    private boolean firstLiftDownButton;
    private boolean liftUpCommand;
    private boolean liftDownCommand;
    private boolean firstPressDown = true;
    private boolean firstPressUp = true;
    private int liftHeight = 0;
    private boolean liftIsDown = true;

    private boolean lastWaffleState = false;
    private boolean isWaffleStateRaised = false;
    private double foundationatorPosition = .335;

    static final double countsPerMotor          = 383.6;
    static final double gearReduction           = 1.0 ;
    static final double wheelDiameter           = 1.771653543307087 ;
    static final double countsPerInch           = (countsPerMotor * gearReduction) / (wheelDiameter * Math.PI);

    private String stoneOrientation = "empty";
    private String lCurrentPosition = "lDisengage";
    private String rCurrentPosition = "rDisengage";
    private double orientDistance = 0;
    private double stoneDistance;
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
        telemetry.addData("Version Number", "12-23-19 700pm");
        initializeVerticalLift();
        initializeFoundationator();
        initializeDriveTrain();
        initializeOdometry();
        initializeIntakeMechanism();
        initializeStraightener();
        initializeGrabber();
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
        double LiftUpButton = gamepad1.right_trigger;
        double LiftDownButton = gamepad1.left_trigger;
        boolean LiftManualToggleButton = gamepad1.y;
        boolean LiftOverideDownButton = gamepad2.dpad_down;
        boolean LiftOverideUpButton = gamepad2.dpad_up;

        if (LiftManualToggleButton) {
            if (firstPressa) {
                liftEncoderState =! liftEncoderState;
                firstPressa = false;
            }
        }
        else {
            firstPressa = true;
        }

        if (LiftOverideDownButton) {
            if (firstPressDown) {
                liftHeight--;
                firstPressDown = false;
            }
        }
        else {
            firstPressDown = true;
        }

        if (LiftOverideUpButton) {
            if (firstPressUp) {
                liftHeight++;
                firstPressUp = false;
            }
        }
        else {
            firstPressUp = true;
        }

        if (liftEncoderState) {
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor.setPower(.1);
            telemetry.addData("LiftMotorDistance",LiftMotor.getCurrentPosition() - LiftMotor.getTargetPosition());
            //setting the Target position if we press the right bumper
            if (LiftUpButton > .5) {
                if (firstLiftUpButton) {
                    liftUpCommand = true;
                    firstLiftUpButton = false;
                }
                else {
                    liftUpCommand = false;
                }
            }
            else {
                firstLiftUpButton = true;
            }

            if (LiftDownButton > .5) {
                if (firstLiftDownButton) {
                    liftDownCommand = true;
                    firstLiftDownButton = false;
                }
                else {
                    liftDownCommand = false;
                }
            }
            else {
                firstLiftDownButton = true;
            }

            if (liftUpCommand) {
                liftHeight++;
                LiftMotor.setTargetPosition((int)(liftHeight * (4 * countsPerInch)));
                liftUpCommand = false;
            }
            else if (liftDownCommand) {
                LiftMotor.setTargetPosition(0);
                liftDownCommand = false;
            }

            telemetry.addData("Lift Height", liftHeight);

            /*if (liftUpCommand) {
                LiftMotor.setTargetPosition((int)(LiftMotor.getTargetPosition() + (4 * countsPerInch)));
            }
            //setting the Target position if we press the left Bumper
            else if (liftDownCommand) {
                LiftMotor.setTargetPosition((int)(LiftMotor.getTargetPosition() + (-4 * countsPerInch)));
            }*/
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

        if (isWaffleStateRaised) {
            telemetry.addData("Foundationator is", "Raised");
        }
        else {
            telemetry.addData("Foundationator is", "Lowered");
        }
    }

    private void raiseFoundationator() {
        lFoundationator.setPosition(0);
        rFoundationator.setPosition(foundationatorPosition);
        isWaffleStateRaised = true;
    }

    private void lowerFoundationator() {
        lFoundationator.setPosition(foundationatorPosition);
        rFoundationator.setPosition(0);
        isWaffleStateRaised = false;
    }

    //
    // Grabber Arm
    //

    private void initializeGrabber() {
        GrabberServo =  hardwareMap.servo.get("GrabberServo");
        OrientStoneServo = hardwareMap.servo.get("OrientStoneServo");
        MoveArmServo = hardwareMap.servo.get("MoveArmServo");
    }

    private void grabStoneFromStraightener() {
        GrabberServo.setPosition(.5);
        // close grabber servo
        return;
    }
    //this.moveArmToScorePosition("horizontal");
    private void moveArmToScorePosition(String orientation) {
        if(orientation.contentEquals("long")) {
        }
        else if(orientation.contentEquals("wide")) {
        }
        // move arm to scoring position
        // orient the grabber to prepare to place the stone
        return;
    }

    private void releaseStoneFromGrabber() {
        GrabberServo.setPosition(0);
        // open grabber servo to release stone
        return;
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

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
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

        //StartingXPosition = Double.parseDouble(ReadWriteFile.readFile(startingXpositionFile).trim());
        //StartingYPosition = Double.parseDouble(ReadWriteFile.readFile(startingYpositionFile).trim());
        //StartingRotation = Double.parseDouble(ReadWriteFile.readFile(startingθpositionFile).trim());
        StartingXPosition = 0;
        StartingYPosition = 0;
        StartingRotation = 0;
    }

    public void startOdometry() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
    }

    public void checkOdometry() {
        RobotXPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
        RobotYPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
        RobotRotation = (globalPositionUpdate.returnOrientation()) + StartingRotation;

        telemetry.addData("X", RobotXPosition);
        telemetry.addData("Y", RobotYPosition);
        telemetry.addData("θ", RobotRotation);

        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("θ (Degrees)", globalPositionUpdate.returnOrientation());

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
        StonePresenceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "StonePresenceSensor");
        OrientationServoLeft = hardwareMap.get(Servo.class, "OrientationServoLeft");
        OrientationServoRight = hardwareMap.get(Servo.class, "OrientationServoRight");
        OrientationServoLeft.setPosition(lDisengage);
        OrientationServoRight.setPosition(rDisengage);
    }

    public void checkStraightener() {
        //stoneFullyInStraightener = gamepad1.b;
        stoneDistance = StonePresenceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Stone Distance", stoneDistance);
        if (stoneDistance < .5) {
            stoneFullyInStraightener = true;
        }
        else {
            stoneFullyInStraightener = false;
        }

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