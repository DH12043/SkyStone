package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.util.Range;
import java.io.File;

@TeleOp(name = "PowerSurgeTeleOp")
public class PowerSurgeTeleOp extends OpMode {

    // DRIVETRAIN AND ODOMETRY

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

    private static final double DECELERATION_START_POINT = 96;
    private static final double DECELERATION_ZERO_POINT = -6;
    private static final double TURNING_DECELERATION_START_POINT = 180;
    private static final double TURNING_DECELERATION_ZERO_POINT = -5;
    private static final double X_SPEED_MULTIPLIER = 1; // Compensates for slower movement while strafing was 1.2

    private double lastDistanceToTarget = 0;

    private double movement_x;
    private double movement_y;
    private double movement_turn;

    private double rightBackupDistance;
    private double leftBackupDistance;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    // MOTORS SERVOS SENSORS

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
    private ModernRoboticsI2cRangeSensor RightBackupSensor;
    private ModernRoboticsI2cRangeSensor LeftBackupSensor;

    // BUTTONS

    private double LiftUpButton;
    private double LiftDownButton;
    private boolean LiftManualToggleButton;
    private boolean LiftEncoderResetButton;
    private boolean LiftOverideDownButton;
    private boolean LiftOverideUpButton;
    private boolean LiftHeightResetButton;
    private boolean LiftHeightResetButton2;

    private boolean currentWaffleState;

    private double grabStoneButton;
    private double releaseStoneButton;
    private boolean emergencyEjectButton;
    private boolean grabberManualButton;
    private boolean armManualButton;

    private boolean outputButton;
    private boolean intakeButton;
    private boolean intakeReleaseButton;

    private boolean manualLeftServoButton;
    private boolean manualRightServoButton;

    private boolean odometryResetButton;
    private boolean halfSpeedDriveButton;
    private boolean autoDriveButton;

    private boolean killSwitch;
    private boolean killSwitch2;

    // FIRST PRESS AND RUN

    private boolean firstLiftUpButton;
    private boolean firstLiftDownButton;
    private boolean liftUpCommand;
    private boolean liftDownCommand;
    private boolean firstPressDpadUp = true;
    private boolean firstPressBumpers = true;
    private boolean liftEncoderState = true;
    private boolean firstPressy = true;
    private boolean firstPressb = true;
    private boolean firstPressx = true;
    private boolean firstPressx2 = true;
    private boolean firstPressb2 = true;
    private boolean firstPressDown = true;
    private boolean firstPressUp = true;
    private boolean firstLeftRun = true;
    private boolean firstPressy2 = true;
    private boolean firstPressRightTrigger = true;
    private boolean firstPressLeftTrigger = true;
    private boolean firstPressArmManual = true;
    private boolean firstPressGrabberManual = true;
    private boolean firstRunZeroLiftPosition = true;
    private boolean firstPressLiftHeightReset = true;

    // WAFFLE STUFF

    private boolean lastWaffleState = false;
    private boolean isWaffleStateRaised = false;

    // INTAKE STUFF

    private int intakeState = 0;
    private int intakeReleaseState = 1;
    private int liftHeight = -1;

    // LIFT STUFF

    static final double countsPerMotor          = 383.6;
    static final double gearReduction           = 1.0 ;
    static final double wheelDiameter           = 1.771653543307087 ;
    static final double countsPerInch           = (countsPerMotor * gearReduction) / (wheelDiameter * Math.PI);
    static final double liftOffset = (2.5 * countsPerInch);     //TODO Change

    // ORIENTER STUFF

    private String stoneOrientation = "empty";
    private String lCurrentPosition = "lDisengage";
    private String rCurrentPosition = "rDisengage";
    private double foundationatorPosition = .335;
    private double orientDistance = 0;
    private double lDisengage = .2;
    private double rDisengage = .9;
    private double lEngage = .85;
    private double rEngage = .25;
    private double startRightTime = 0;
    private double currentRightTime = 0;
    private double actualRightTime = 1.5;
    private double lastActualRightTime = 0;
    private double rightOrientCheck = 1;
    private double startLeftTime = 0;
    private double currentLeftTime = 0;
    private double actualLeftTime = 1.5;
    private double waitTime = 1;
    private double lastActualLeftTime = 0;
    private double targetTime = .5;
    private double stoneDistance = 0;
    private boolean readyToGrab = false;
    private boolean stoneFullyInStraightener = false;
    private boolean straightenerBusy = false;
    private boolean firstRightRun = true;

    // GRABBER STUFF

    private double startGrabberTime;
    private double currentGrabberTime;
    private int liftGrabberState = 0;
    private int grabberReturnState = 0;
    private int emergencyStoneEjectState = 0;
    private double grabberOpenPosition = .4;
    private double armInsidePosition = .75;
    private double armOutsidePosition = .5;
    private double grabberClosedPosition = 0;
    private boolean grabberManualClosed = false;
    private boolean armManualClosed = true;
    private boolean readyToRelease = false;

    //last update time
    private long lastUpdateTime = 0;


    @Override
    public void init() {
        telemetry.addData("Version Number", "1-3-20 1000pm");
        initializeVerticalLift();
        initializeFoundationator();
        initializeGrabber();
        initializeDriveTrain();
        initializeOdometry();
        initializeIntakeMechanism();
        initializeStraightener();
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        startFoundationator();
        startIntakeMechanism();
        startVerticalLift();
        startGrabber();
        startOdometry();
        telemetry.addData("Status", "Odometry System has started");
        telemetry.update();
    }

    @Override
    public void loop() {
        LiftUpButton = gamepad1.right_trigger;
        LiftDownButton = gamepad1.left_trigger;
        LiftManualToggleButton = gamepad2.y;
        LiftEncoderResetButton = gamepad2.x;
        LiftOverideDownButton = gamepad2.dpad_down;
        LiftOverideUpButton = gamepad2.dpad_up;
        LiftHeightResetButton = gamepad2.right_bumper;
        LiftHeightResetButton2 = gamepad2.left_bumper;

        currentWaffleState = gamepad1.a;

        grabStoneButton = gamepad1.right_trigger;
        releaseStoneButton = gamepad1.left_trigger;
        emergencyEjectButton = gamepad1.y;
        grabberManualButton = gamepad2.b;
        armManualButton = gamepad2.a;

        outputButton = gamepad1.dpad_down;
        intakeButton = gamepad1.dpad_up;
        intakeReleaseButton = gamepad1.x;

        manualLeftServoButton = gamepad2.dpad_right;
        manualRightServoButton = gamepad2.dpad_left;

        odometryResetButton = gamepad1.b;
        halfSpeedDriveButton = gamepad1.left_bumper;
        autoDriveButton = gamepad1.right_bumper;

        killSwitch = gamepad1.dpad_right;
        killSwitch2 = gamepad1.dpad_left;

        if (killSwitch || killSwitch2) {
            FrontRight.setPower(0);
            BackRight.setPower(0);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            LiftMotor.setPower(0);
            IntakeMotor.setPower(0);
        }
        else {
            checkDriveTrain();
            checkOdometry();
            checkVerticalLift();
            checkFoundationator();
            checkIntakeMechanism();
            checkStraightener();
            checkGrabber();
            telemetry.update();
        }
    }

    //
    // VERTICAL LIFT
    //

    private void initializeVerticalLift() {
        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setPower(1);
        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void startVerticalLift() {
        LiftMotor.setTargetPosition((int)( 2 * countsPerInch));
    }

    private void checkVerticalLift() {
        if (LiftManualToggleButton) {
            if (firstPressy2) {
                liftEncoderState =! liftEncoderState;
                firstPressy2 = false;
            }
        }
        else {
            firstPressy2 = true;
        }

        if (LiftEncoderResetButton) {
            LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        if (LiftHeightResetButton && LiftHeightResetButton2) {
            if (firstPressLiftHeightReset) {
                liftHeight = -1;
                firstPressLiftHeightReset = false;
            }
        }
        else {
            firstPressLiftHeightReset = true;
        }

        if (liftEncoderState) {
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor.setPower(1);
            /*telemetry.addData("LiftMotorDistance",LiftMotor.getCurrentPosition() - LiftMotor.getTargetPosition());
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
            }*/

            if (liftUpCommand) {
                if (liftHeight >= 10) {
                    liftHeight = 10;
                }
                else {
                    liftHeight++;
                }
                LiftMotor.setPower(1);
                LiftMotor.setTargetPosition((int)(liftHeight * (4 * countsPerInch) + liftOffset));
                liftUpCommand = false;
            }
            else if (liftDownCommand) {
                LiftMotor.setPower(.5);
                LiftMotor.setTargetPosition((int)(2 * countsPerInch));
                liftDownCommand = false;
            }

            telemetry.addData("Lift Height", liftHeight);
        }
        else {
            LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LiftMotor.setPower(LiftUpButton - LiftDownButton);
        }
    }

    //
    // FOUNDATIONATOR
    //

    private void initializeFoundationator() {
        lFoundationator = hardwareMap.servo.get("lFoundationator");
        rFoundationator = hardwareMap.servo.get("rFoundationator");
    }

    private void startFoundationator() {
        lFoundationator.setPosition(0);
        rFoundationator.setPosition(foundationatorPosition);
    }

    private void checkFoundationator() {
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

    private void startGrabber() {
        MoveArmServo.setDirection(Servo.Direction.FORWARD);
        MoveArmServo.setPosition(0);
        GrabberServo.setPosition(grabberOpenPosition);
    }

    private void checkGrabber() {
        if (liftEncoderState) {
            telemetry.addData("readyToGrab", readyToGrab);
            if (readyToGrab && !readyToRelease && liftGrabberState == 0 && grabberReturnState == 0 && emergencyStoneEjectState == 0) {
                if (firstRunZeroLiftPosition) {
                    startGrabberTime = getRuntime();
                    firstRunZeroLiftPosition = false;
                }
                currentGrabberTime = getRuntime();
                if (currentGrabberTime - startGrabberTime > .25) {
                    LiftMotor.setPower(1);
                    LiftMotor.setTargetPosition(0);
                }
            }
            else {
                firstRunZeroLiftPosition = true;
            }

            if (readyToGrab) {
                if (grabStoneButton > .5) {
                    if (firstPressRightTrigger) {
                        GrabberServo.setPosition(grabberClosedPosition);
                        liftGrabberState = 1;
                        startGrabberTime = getRuntime();
                        firstPressRightTrigger = false;
                    }
                } else {
                    firstPressRightTrigger = true;
                }
            }
            grabRotateRaiseStone();

            if (releaseStoneButton > .5) {
                if (firstPressLeftTrigger) {
                    GrabberServo.setPosition(grabberOpenPosition);
                    liftGrabberState = 0;
                    grabberReturnState = 1;
                    startGrabberTime = getRuntime();
                    firstPressLeftTrigger = false;
                }
            } else {
                firstPressLeftTrigger = true;
            }
            returnGrabberArm();

            if (emergencyEjectButton) {
                if (firstPressy) {
                    emergencyStoneEjectState = 1;
                    GrabberServo.setPosition(grabberClosedPosition);
                    startGrabberTime = getRuntime();
                    firstPressy = false;
                }
            } else {
                firstPressy = true;
            }
            emergencyStoneEject();
        }
        else {
            if (grabberManualButton) {
                firstPressGrabberManual = false;
                if (grabberManualClosed) {
                    GrabberServo.setPosition(grabberOpenPosition);
                }
                else {
                    GrabberServo.setPosition(grabberClosedPosition);
                }
                grabberManualClosed = !grabberManualClosed;
            }
            else {
                firstPressGrabberManual = true;
            }

            if (armManualButton) {
                firstPressArmManual = false;
                if (armManualClosed) {
                    MoveArmServo.setDirection(Servo.Direction.FORWARD);
                    MoveArmServo.setPosition(1);
                }
                else {
                    MoveArmServo.setDirection(Servo.Direction.REVERSE);
                    MoveArmServo.setPosition(0);
                }
                armManualClosed = !armManualClosed;
            }
            else {
                firstPressArmManual = true;
            }
        }
    }

    private void grabRotateRaiseStone() {
        if(liftGrabberState == 1) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > .5) {
                liftGrabberState++;
                startGrabberTime = getRuntime();
            }
        }
        else if(liftGrabberState == 2) {
            currentGrabberTime = getRuntime();
            if (liftHeight <= 1) {
                LiftMotor.setPower(1);
                LiftMotor.setTargetPosition((int)((10 * countsPerInch) + liftOffset));
                if(LiftMotor.getCurrentPosition() > (int)((9 * countsPerInch) + liftOffset)) {
                    MoveArmServo.setDirection(Servo.Direction.REVERSE);
                    MoveArmServo.setPosition(0);
                    if (currentGrabberTime - startGrabberTime > 2) {
                        liftUpCommand = true;
                        liftGrabberState = 0;
                    }
                }
            }
            else {
                liftUpCommand = true;
                liftGrabberState++;
                startGrabberTime = getRuntime();
            }
        }
        else if(liftGrabberState == 3) {
            currentGrabberTime = getRuntime();
            if (LiftMotor.getCurrentPosition() > (int)(liftHeight * (4 * countsPerInch) + (liftOffset-(.5 * countsPerInch)))) {
                liftGrabberState++;
            }
        }
        else if(liftGrabberState == 4) {
            MoveArmServo.setDirection(Servo.Direction.REVERSE);
            MoveArmServo.setPosition(0);
            readyToRelease = true;
            liftGrabberState = 0;
        }
    }

    private void returnGrabberArm() {
        if (grabberReturnState == 1) {
            LiftMotor.setPower(.5);
            LiftMotor.setTargetPosition((int)((liftHeight * (4 * countsPerInch)) + liftOffset - (2 * countsPerInch)));
            if (LiftMotor.getCurrentPosition() < (int)(liftHeight * (4 * countsPerInch) + liftOffset - (1.5 * countsPerInch))) {
                grabberReturnState++;
                startGrabberTime = getRuntime();
            }
        }
        if (grabberReturnState == 2) {
            LiftMotor.setPower(1);
            LiftMotor.setTargetPosition((int)((liftHeight * (4 * countsPerInch)) + liftOffset + (1 * countsPerInch)));
            if (LiftMotor.getCurrentPosition() > (int)(liftHeight * (4 * countsPerInch) + liftOffset + (.5 * countsPerInch))) {
                StartingXPosition = -(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                StartingYPosition = -(globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                StartingRotation = -(globalPositionUpdate.returnOrientation());
                grabberReturnState++;
                startGrabberTime = getRuntime();
            }
        }
        if (grabberReturnState == 3) {
            currentGrabberTime = getRuntime();
            GrabberServo.setPosition(grabberOpenPosition);

            if(currentGrabberTime - startGrabberTime > .5) {
                if (liftHeight <= 1) {
                    LiftMotor.setPower(1);
                    LiftMotor.setTargetPosition((int)((6 * countsPerInch) + liftOffset));
                    if(LiftMotor.getCurrentPosition() > (int)((5 * countsPerInch) + liftOffset)) {
                        MoveArmServo.setDirection(Servo.Direction.FORWARD);
                        MoveArmServo.setPosition(1);
                        grabberReturnState++;
                        startGrabberTime = getRuntime();
                    }
                }
                else {
                    MoveArmServo.setDirection(Servo.Direction.FORWARD);
                    MoveArmServo.setPosition(1);
                    grabberReturnState++;
                    startGrabberTime = getRuntime();
                }
            }

        }
        else if (grabberReturnState == 4) {
            currentGrabberTime = getRuntime();
            if(currentGrabberTime - startGrabberTime > 1.5) {
                liftDownCommand = true;
                readyToRelease = false;
                grabberReturnState = 0;
            }
        }
    }

    private void emergencyStoneEject() {
        if(emergencyStoneEjectState == 1) {
            LiftMotor.setPower(.5);
            LiftMotor.setTargetPosition(0);
            if(LiftMotor.getCurrentPosition() < (int)(countsPerInch / 4)) {
                emergencyStoneEjectState++;
            }
        }
        else if(emergencyStoneEjectState == 2) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > .5) {
                LiftMotor.setPower(1);
                LiftMotor.setTargetPosition((int)((8 * countsPerInch) + liftOffset));
                if(LiftMotor.getCurrentPosition() > (int)((7 * countsPerInch) + liftOffset)) {
                    MoveArmServo.setDirection(Servo.Direction.FORWARD);
                    MoveArmServo.setPosition(.5);
                    startGrabberTime = getRuntime();
                    emergencyStoneEjectState++;
                }
            }
        }
        else if(emergencyStoneEjectState == 3) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > .5) {
                GrabberServo.setPosition(grabberOpenPosition);
                MoveArmServo.setDirection(Servo.Direction.FORWARD);
                MoveArmServo.setPosition(1);
                startGrabberTime = getRuntime();
                emergencyStoneEjectState++;
            }
        }
        else if(emergencyStoneEjectState == 4) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > 1) {
                liftDownCommand = true;
                emergencyStoneEjectState = 0;
            }
        }
    }

    //
    // DRIVE TRAIN
    //

    private void initializeDriveTrain() {
        RightBackupSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "RightBackupSensor");
        LeftBackupSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "LeftBackupSensor");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void checkDriveTrain() {
        rightBackupDistance = RightBackupSensor.getDistance(DistanceUnit.INCH);
        leftBackupDistance = LeftBackupSensor.getDistance(DistanceUnit.INCH);

        telemetry.addData("Right Backup Distance", rightBackupDistance);
        telemetry.addData("Left Backup Distance", leftBackupDistance);

        if (autoDriveButton) {
            if (firstPressBumpers) {
                firstPressBumpers = false;
            }
            goToPositionMrK(0,0,1,.5, 0);
        }
        else {
            firstPressBumpers = true;

            movement_y = DeadModifier(-gamepad1.left_stick_y);
            movement_x = DeadModifier(gamepad1.left_stick_x);
            movement_turn = DeadModifier(gamepad1.right_stick_x);

            if (halfSpeedDriveButton) {
                movement_y = movement_y / 5;
                movement_x = movement_x / 3;
                movement_turn = movement_turn / 5;
            }

            applyMovement();

            //double forwardButton = gamepad1.left_stick_y;
            //double sidewaysButton = gamepad1.left_stick_x;
            //double spinningButton = gamepad1.right_stick_x;

            //FrontRight.setDirection(DcMotor.Direction.REVERSE);
            //BackLeft.setDirection(DcMotor.Direction.REVERSE);

            //forwardButton = DeadModifier(forwardButton);
            //sidewaysButton = DeadModifier(sidewaysButton);
            //spinningButton = DeadModifier(spinningButton);

            //Drive(forwardButton, sidewaysButton, spinningButton);
        }
    }

    /**
     * Universal goToPosition method that assumes the following coordinate system for the
     * odometry output values: , RobotYPosition, and :
     * RobotRotation: 0*->360* rotation values, where 0* and 360* are straight forward
     * RobotXPosition: Positive X would be strafing to the right
     * RobotYPosition: Positive Y would be moving forwards
     * Method generates three values indicating the immediate motion needed to move the
     * robot towards the global position requested:
     * movement_turn: Positive value to turn clockwise, negative for counterclockwise
     * movement_x: Positive value indicates strafe to the right proportionally
     * movement_y: Positive value indicates drive forward proportionally
     * These values can be used as inputs to a method that typically accepts joystick control,
     * in some cases the 'y' value must be negated.
     *
     * author: Original code by FTC Team 11115 Gluten Free, modified by Travis Kuiper.
     * date: 2020/01/01
     *
     * @param x global target coordinate 'x' component
     * @param y global target coordinate 'x' component
     * @param maxMovementSpeed max speed value to be given to any one drivetrain direction of motion
     * @param maxTurnSpeed max turning speed to be given to drivetrain rotation
     * @param preferredAngle global target coordinate theta component
     */
    private void goToPositionMrK(double x, double y, double maxMovementSpeed, double maxTurnSpeed, double preferredAngle) {
        double distanceToTarget = Math.hypot(x-RobotXPosition, y-RobotYPosition);
        double absoluteAngleToTarget = Math.atan2(y-RobotYPosition, x-RobotXPosition);
        double relativeAngleToPoint = AngleWrap(-absoluteAngleToTarget
                - Math.toRadians(RobotRotation) + Math.toRadians(90));

        double relativeXToPoint = 2 * Math.sin(relativeAngleToPoint);
        double relativeYToPoint = Math.cos(relativeAngleToPoint);

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double yDecelLimiter = Range.clip(Math.abs((distanceToTarget - DECELERATION_ZERO_POINT)
                / (DECELERATION_START_POINT - DECELERATION_ZERO_POINT)), 0, 1);
        double xDecelLimiter = Range.clip(yDecelLimiter * X_SPEED_MULTIPLIER, 0, 1);

        double relativeTurnAngle = AngleWrap(Math.toRadians(preferredAngle)-Math.toRadians(RobotRotation));
        double turnDecelLimiter = Range.clip((Math.abs(Math.toDegrees(relativeTurnAngle)) - TURNING_DECELERATION_ZERO_POINT)
                / (TURNING_DECELERATION_START_POINT - TURNING_DECELERATION_ZERO_POINT), 0, 1);

        movement_x = movementXPower * Range.clip(maxMovementSpeed, -xDecelLimiter, xDecelLimiter);
        movement_y = movementYPower * Range.clip(maxMovementSpeed, -yDecelLimiter, yDecelLimiter);

        if (distanceToTarget < 1) {
            movement_turn = 0;
        } else {
            //movement_turn = Range.clip(Range.clip(relativeTurnAngle / Math.toRadians(30),
            //        -1, 1) * maxTurnSpeed, -turnDecelLimiter, turnDecelLimiter);
            movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(TURNING_DECELERATION_START_POINT), -1, 1) * maxTurnSpeed;
        }
        telemetry.addData("relativeTurnAngle", relativeTurnAngle);
        telemetry.addData("turnDecelLimiter", turnDecelLimiter);
        telemetry.addData("relativeXToPoint", relativeXToPoint);
        telemetry.addData("relativeYToPoint", relativeYToPoint);
        telemetry.addData("X Movement", movement_x);
        telemetry.addData("Y Movement", movement_y);
        telemetry.addData("Turn Movement", movement_turn);

        lastDistanceToTarget = distanceToTarget;

        applyMovement();
    }

    /**
     * Converts movement_y, movement_x, movement_turn into motor powers.
     *
     * author: Original code by FTC Team 7571 Alumineers, modified by Travis Kuiper
     * date: 2020/01/01
     *
     * movement_turn: Positive value to turn clockwise, negative for counterclockwise
     * movement_x: Positive value indicates strafe to the right proportionally
     * movement_y: Positive value indicates drive forward proportionally
     */
    // Code comes from 11115 Peter and 7571 Alumineers
    private void applyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        //double fl_power_raw = movement_y-movement_turn+movement_x;
        //double bl_power_raw = movement_y-movement_turn-movement_x;
        //double br_power_raw = -movement_y-movement_turn-movement_x;
        //double fr_power_raw = -movement_y-movement_turn+movement_x;

        double fl_power_raw = movement_y+movement_turn+movement_x;
        double bl_power_raw = movement_y+movement_turn-movement_x;
        double br_power_raw = -movement_y+movement_turn-movement_x;
        double fr_power_raw = -movement_y+movement_turn+movement_x;

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
        FrontLeft.setPower(-fl_power_raw);
        BackLeft.setPower(-bl_power_raw);
        BackRight.setPower(-br_power_raw);
        FrontRight.setPower(-fr_power_raw);
    }

    /**
     * wrap angle to -180 to 180
     * @param angle angle to be wrapped in radians
     * @return new angle in radians
     */

    private static double AngleWrap(double angle){

        while(angle < -Math.PI){
            angle += 2*Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2*Math.PI;
        }

        return angle;
    }

    // By Paul

    private double DeadModifier(double joystickValue) {
        if(joystickValue < DEADZONE && joystickValue > -DEADZONE)
            return 0;
        else {
            return joystickValue;
        }
    }

    private void Drive(double DZForwardButton, double DZSidewaysButton, double DZSpinningButton) {
        DZSpinningButton = -DZSpinningButton;
        FrontRight.setPower(-DZSidewaysButton - DZForwardButton + DZSpinningButton);
        FrontLeft.setPower(-DZSidewaysButton + DZForwardButton + DZSpinningButton);
        BackRight.setPower(DZSidewaysButton - DZForwardButton + DZSpinningButton);
        BackLeft.setPower(DZSidewaysButton + DZForwardButton + DZSpinningButton);
    }

    //
    // Odometry
    //

    private void initializeOdometry() {
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //These values also affect the drive motors so we also reversed FrontRight
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

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

    private void startOdometry() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 25);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
    }

    private void checkOdometry() {
        RobotXPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
        RobotYPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
        RobotRotation = (globalPositionUpdate.returnOrientation()) + StartingRotation;

        if (odometryResetButton) {
            if (firstPressb2) {
                firstPressb2 = false;
                StartingXPosition = -(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                StartingYPosition = -(globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                StartingRotation = -(globalPositionUpdate.returnOrientation());
            }
        }
        else {
            firstPressb2 = true;
        }

        if (RobotRotation < 0){
            RobotRotation += 360;
        }

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

    //
    // INTAKE
    //

    private void initializeIntakeMechanism() {
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        IntakeAssistServo = hardwareMap.crservo.get("IntakeAssistServo");
        IntakeReleaseServo = hardwareMap.servo.get("IntakeReleaseServo");
        IntakeMotor.setPower(0);
        IntakeAssistServo.setPower(0);
    }
    private void startIntakeMechanism() {
        IntakeReleaseServo.setPosition(.6);
        IntakeAssistServo.setPower(0);
    }

    private void checkIntakeMechanism() {
        intake(intakeButton);

        if (intakeReleaseButton) {
            if (firstPressx2) {
                if (intakeReleaseState == 1) {
                    intakeReleaseState = 0;
                }
                else {
                    intakeReleaseState = 1;
                }
                firstPressx2 = false;
            }
        }
        else {
            firstPressx2 = true;
        }

        if (intakeReleaseState == 0) {
            IntakeReleaseServo.setPosition(.15);
        }
        else {
            IntakeReleaseServo.setPosition(.6);
        }
    }

    private void intake(boolean intakeButton) {
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

    private void initializeStraightener() {
        OrientationSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "OrientationSensor");
        StonePresenceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "StonePresenceSensor");
        OrientationServoLeft = hardwareMap.get(Servo.class, "OrientationServoLeft");
        OrientationServoRight = hardwareMap.get(Servo.class, "OrientationServoRight");
        OrientationServoLeft.setPosition(lDisengage);
        OrientationServoRight.setPosition(rDisengage);
    }

    private void checkStraightener() {
        stoneDistance = StonePresenceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Stone Distance", stoneDistance);
        stoneFullyInStraightener = stoneDistance < 1.5;

        orientStone();
        manualOverride();
    }


    private void senseOrientation() {
        //orientDistance = OrientationSensor.getDistance(DistanceUnit.INCH);
        orientDistance = OrientationSensor.cmOptical()/2.54;
        telemetry.addData("Orient Distance", orientDistance);

        // StoneOrientation is assigned relative to the side of the robot that the studs of the stone
        // are on when looking at the robot from the back.
        // this is true for the servos as well; left and right are assigned relative to the back

        if(!straightenerBusy) {
            /*if (orientDistance > .25 && orientDistance <= .55) {
                stoneOrientation = "left";
            }
            else if (orientDistance > .75 && orientDistance <= 2.3) {
                stoneOrientation = "center";
            }
            else if (orientDistance > 4) {
                stoneOrientation = "empty";
            }
            else if (orientDistance > 2.5 && orientDistance < 3){
                stoneOrientation = "right";
            }*/

            if (orientDistance > .35 && orientDistance <= .6) {
                stoneOrientation = "left";
            }
            else if (orientDistance > 1.2 && orientDistance <= 1.8) {
                stoneOrientation = "center";
            }
            else if (orientDistance > 3) {
                stoneOrientation = "empty";
            }
            else if (orientDistance > 2 && orientDistance < 2.4){
                stoneOrientation = "right";
            }
            else {
                stoneOrientation = "notInThreshold";
            }
        }
        telemetry.addData("orientPosition", stoneOrientation);
        telemetry.addData("StraightenerBusy", straightenerBusy);
    }

    private void orientStone() {
        senseOrientation();

        if (!straightenerBusy) {
            OrientationServoLeft.setPosition(lDisengage);
            OrientationServoRight.setPosition(rDisengage);
        }

        if (stoneFullyInStraightener) {
            if (stoneOrientation.equals("right")) {
                runLeftServo();
                readyToGrab = false;
            }
            else if (stoneOrientation.equals("left")) {
                runRightServo();
                readyToGrab = false;
            }
            else if (stoneOrientation.equals("empty")) {
                readyToGrab = false;
                OrientationServoLeft.setPosition(lDisengage);
                OrientationServoRight.setPosition(rDisengage);
            }
            else if(stoneOrientation.equals("center")) {
                readyToGrab = true;
                OrientationServoLeft.setPosition(lDisengage);
                OrientationServoRight.setPosition(rDisengage);
            }
            else if (stoneOrientation.equals("notInThreshold")) {
                readyToGrab = false;
                OrientationServoLeft.setPosition(lDisengage);
                OrientationServoRight.setPosition(rDisengage);
            }
            telemetry.addData("actualLeftTime", actualLeftTime);
            telemetry.addData("actualRightTime", actualRightTime);
            telemetry.addData("target time", targetTime);
            telemetry.addData("orientPosition", stoneOrientation);
            telemetry.addData("StraightenerBusy", straightenerBusy);
        }
        else {
            readyToGrab = false;
            if (!straightenerBusy) {
                stoneOrientation = "empty";
            }
        }
    }

    private void manualOverride() {
        if (manualLeftServoButton) {
            runLeftServo();
        }
        if (manualRightServoButton) {
            runRightServo();
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

        if(actualRightTime > targetTime) {
            OrientationServoRight.setPosition(rDisengage);
            firstRightRun = true;
        }

        if (actualRightTime > targetTime*2) {
            OrientationServoRight.setPosition(rDisengage);
            firstRightRun = true;
            straightenerBusy = false;
        }
        else {
            straightenerBusy = true;
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

        if(actualLeftTime > targetTime) {
            OrientationServoLeft.setPosition(lDisengage);
            firstLeftRun = true;
        }

        if (actualLeftTime > targetTime*2) {
            OrientationServoLeft.setPosition(lDisengage);
            firstLeftRun = true;
            straightenerBusy = false;
        }
        else {
            straightenerBusy = true;
        }
    }
}