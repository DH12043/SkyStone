package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.view.View;
import android.graphics.Color;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.util.Range;
import java.io.File;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "PowerSurgeTeleOp")
public class PowerSurgeTeleOp extends OpMode {

    private String robotAlliance = "null";

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

    private double StartingFoundationXPosition;
    private double StartingFoundationYPosition;
    private double StartingFoundationRotation;

    private double ScoringXPosition;
    private double ScoringYPosition;
    private double ScoringRotation;

    private File startingXpositionFile = AppUtil.getInstance().getSettingsFile("startingXposition.txt");
    private File startingYpositionFile = AppUtil.getInstance().getSettingsFile("startingYposition.txt");
    private File startingθpositionFile = AppUtil.getInstance().getSettingsFile("startingθposition.txt");

    final double COUNTS_PER_INCH = 307.699557;

    private static final double DECELERATION_START_POINT = 24;
    private static final double DECELERATION_ZERO_POINT = -6;
    private static final double TURNING_DECELERATION_START_POINT = 100;
    private static final double TURNING_DECELERATION_ZERO_POINT = -5;
    private static final double X_SPEED_MULTIPLIER = 1;

    private boolean firstRunRemoveFoundation = true;

    private double lastDistanceToTarget = 0;

    private double movement_x;
    private double movement_y;
    private double movement_turn;

    private double distanceToTarget;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    // MOTORS SERVOS SENSORS

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor IntakeMotor;
    private DcMotor LiftMotor;
    private DcMotor TapeMeasureMotor;
    private DcMotor IntakeAssistMotor;
    private Servo lFoundationator;
    private Servo rFoundationator;
    private Servo GrabberServo;
    private Servo OrientStoneServo;
    private Servo MoveArmServo;
    private Servo OrientationServoLeft;
    private Servo OrientationServoRight;
    private Servo IntakeReleaseServo;
    private Servo LiftSlapServo;

    private ModernRoboticsI2cRangeSensor OrientationSensor;
    private ModernRoboticsI2cRangeSensor StonePresenceSensor;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private TouchSensor AllianceSwitch;

    // BUTTONS

    private double LiftUpButton;
    private double LiftDownButton;
    private boolean LiftManualToggleButton;
    private boolean LiftEncoderResetButton;
    private boolean LiftOverideDownButton;
    private boolean LiftOverideUpButton;
    private boolean LiftHeightResetButton;
    private double slapServoButton;

    private boolean currentWaffleState;

    private double grabStoneButton;
    private double releaseStoneButton;
    private int loopCount;
    private double loopStartTime;
    private int loopsPerSecond;
    private boolean emergencyEjectButton;
    private boolean capstoneButton;
    private boolean grabberManualButton;
    private boolean armManualButton;
    private double quickUpDownButton;

    private boolean intakeButton;
    private boolean outtakeButton;

    private boolean manualLeftServoButton;
    private boolean manualRightServoButton;

    private boolean halfSpeedDriveButton;
    private boolean autoDriveButton;
    private boolean autoRemoveFoundationButton;
    //private double deliverStoneButton;
    private double liftRaiseOrLowerButton;

    private double tapeMeasureButton;

    private boolean switchPositionButton;

    // FIRST PRESS AND RUN

    private boolean liftUpCommand;
    private boolean liftDownCommand;
    private boolean firstPressDpadUp = true;
    private boolean firstPressBumpers = true;
    private boolean liftEncoderState = true;
    private boolean firstPressy = true;
    private boolean firstPressx = true;
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
    private boolean firstPressCapstoneButton = true;
    private boolean firstDeliverStoneButton = true;
    private boolean firstSwitchPositionButton = true;
    private boolean firstPressQuickUpDown = true;
    private boolean firstPressLiftRaiseOrLower = true;

    // WAFFLE STUFF

    private boolean lastWaffleState = false;
    private boolean isWaffleStateRaised = true;

    private boolean foundationNotInPosition = true;

    // INTAKE STUFF

    private int intakeState = 0;
    private int intakeReleaseState = 1;
    private int liftHeight = -1;

    private int intakeEncoderPosition = 0;
    private int lastIntakeEncoderPosition = 0;
    private int jammedIntakeLoopCounter = 0;

    private boolean firstRunIntakeReverse = false;
    private boolean runIntakeReverse = false;

    private double firstIntakeReverseTime;
    private double intakeReverseTime;

    // LIFT STUFF

    static final double countsPerMotor          = 383.6;
    static final double gearReduction           = 1.0 ;
    static final double wheelDiameter           = 1.75; //1.72
    static final double countsPerInch           = (countsPerMotor * gearReduction) / (wheelDiameter * Math.PI);
    static final double liftOffset = (2.5 * countsPerInch);
    private double globalLiftOffset = 0.0;

    static final double bottomLiftPosition = 4; //measured in inches was 2.5

    // ORIENTER STUFF

    private int flipCounter = 0;
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
    private int autoDeliverStoneState = 0;
    private int grabberReturnState = 0;
    private int quickUpDownState = 0;
    private int grabberReturnType = 2;
    private int emergencyStoneEjectState = 0;
    private double grabberWideOpenPosition = .9;
    private double grabberOpenPosition = .45;
    private double grabberClosedPosition = 0;
    private double armInsidePosition = 1;
    private double armOutsidePosition = 0.02;
    private double slapDownPosition = .3;
    private double slapUpPosition = .7;
    private boolean grabberManualClosed = false;
    private boolean armManualClosed = true;
    private boolean readyToRelease = false;
    private boolean readyToReleaseFromDelivery = false;
    private int capstoneState = 0;
    private boolean readyToReleaseStone = false;
    private double slapCurrentTime;
    private double slapStartTime;

    //last update time
    private long lastUpdateTime = 0;

    //Color sensor Stuff
    public NormalizedColorSensor SkyStoneSensor;

    View relativeLayout;

    public boolean isSkyStoneInView = false;

    float hsvValues[] = {0F, 0F, 0F};

    final float values[] = hsvValues;

    final double SCALE_FACTOR = 255;

    public float redValues = 0;
    public float blueValues = 0;
    public float greenValues = 0;
    public float alphaValues = 0;

    public boolean red = false;
    public boolean blue = false;
    public boolean green = false;

    private boolean switchIsInverted = false;

    @Override
    public void init() {
        telemetry.addData("Version Number", "2-13-20 900pm");
        initializeVerticalLift();
        initializeFoundationator();
        initializeGrabber();
        initializeDriveTrain();
        initializeOdometry();
        initializeIntakeMechanism();
        initializeStraightener();
        initializeSkyStoneColorSensor();
        initializeTapeMeasure();
        AllianceSwitch = hardwareMap.touchSensor.get("AllianceSwitch");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES;
        blinkinLedDriver.setPattern(pattern);

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
        double currentTime = getRuntime();
        loopCount++;
        if (currentTime > loopStartTime + 5) {
            loopsPerSecond = loopCount;
            loopCount = 0;
            loopStartTime = currentTime;
        }
        telemetry.addData("LPS: ", loopsPerSecond);

        switchPositionButton = gamepad2.right_bumper;

        /*if (switchPositionButton) {
            if (firstSwitchPositionButton) {
                switchIsInverted = !switchIsInverted;
                firstSwitchPositionButton = false;
            }
        }
        else {
            firstSwitchPositionButton = true;
        }*/

        if (!switchIsInverted) {
            if (AllianceSwitch.isPressed()) {
                telemetry.addData("Switch Position", "Blue");
                robotAlliance = "Blue";
            } else {
                telemetry.addData("Switch Position", "Red");
                robotAlliance = "Red";
            }
        }
        else {
            if (AllianceSwitch.isPressed()) {
                telemetry.addData("Switch Position", "Red");
                robotAlliance = "Red";
            } else {
                telemetry.addData("Switch Position", "Blue");
                robotAlliance = "Blue";
            }
        }

        if (capstoneState != 0) {
            telemetry.addData("CAPSTONE", "READY TO RELEASE");
        }

        LiftUpButton = gamepad1.right_trigger;
        LiftDownButton = gamepad1.left_trigger;
        LiftManualToggleButton = gamepad2.y;
        LiftEncoderResetButton = gamepad2.x;
        LiftOverideDownButton = gamepad2.dpad_down;
        LiftOverideUpButton = gamepad2.dpad_up;
        LiftHeightResetButton = gamepad2.left_bumper;

        currentWaffleState = gamepad1.a;

        grabStoneButton = gamepad1.right_trigger;
        releaseStoneButton = gamepad1.left_trigger;
        emergencyEjectButton = gamepad1.y;
        grabberManualButton = gamepad2.b;
        armManualButton = gamepad2.a;
        capstoneButton = gamepad1.x;
        quickUpDownButton = gamepad2.right_trigger;

        intakeButton = gamepad1.dpad_up;
        outtakeButton = gamepad1.dpad_down;

        manualLeftServoButton = gamepad2.dpad_right;
        manualRightServoButton = gamepad2.dpad_left;
        slapServoButton = gamepad2.left_trigger;

        halfSpeedDriveButton = gamepad1.left_bumper;
        autoDriveButton = gamepad1.right_bumper;
        autoRemoveFoundationButton = gamepad1.b;
        liftRaiseOrLowerButton = gamepad2.right_stick_y;

        tapeMeasureButton = gamepad2.left_stick_y;

        checkDriveTrain();
        checkOdometry();
        checkVerticalLift();
        checkFoundationator();
        checkStraightener();
        checkIntakeMechanism();
        checkGrabber();
        checkTapeMeasure();
        telemetry.update();
    }


    @Override
    public void stop() {
        globalPositionUpdate.stop();
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
        LiftMotor.setTargetPosition((int)(bottomLiftPosition * countsPerInch));
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
                if (liftHeight <= -1) {
                    liftHeight = -1;
                }
                else {
                    liftHeight--;
                }
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

        if (LiftHeightResetButton) {
            if (firstPressLiftHeightReset) {
                liftHeight = -1;
                firstPressLiftHeightReset = false;
            }
        }
        else {
            firstPressLiftHeightReset = true;
        }

        if (liftEncoderState) {

            if (liftRaiseOrLowerButton < -.5) {
                if (firstPressLiftRaiseOrLower) {
                    globalLiftOffset = globalLiftOffset + .5;
                    firstPressLiftRaiseOrLower = false;
                }
            }
            else if (liftRaiseOrLowerButton > .5) {
                if (firstPressLiftRaiseOrLower) {
                    globalLiftOffset = globalLiftOffset - .5;
                    firstPressLiftRaiseOrLower = false;
                }
            }
            else {
                firstPressLiftRaiseOrLower = true;
            }
            telemetry.addData("globalLiftOffset", globalLiftOffset);

            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor.setPower(1);

            if (liftUpCommand) {
                if (liftHeight >= 10) {
                    liftHeight = 10;
                }
                else {
                    liftHeight++;
                }
                LiftMotor.setPower(1);
                LiftMotor.setTargetPosition((int)((liftHeight * (4 * countsPerInch)) + (globalLiftOffset * countsPerInch) + liftOffset + (1 * countsPerInch)));
                liftUpCommand = false;
            }
            else if (liftDownCommand) {
                LiftMotor.setPower(.7);
                LiftMotor.setTargetPosition((int)((bottomLiftPosition * countsPerInch) + (globalLiftOffset * countsPerInch)));
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
        LiftSlapServo = hardwareMap.servo.get("LiftSlapServo");
        GrabberServo = hardwareMap.servo.get("GrabberServo");
        OrientStoneServo = hardwareMap.servo.get("OrientStoneServo");
        MoveArmServo = hardwareMap.servo.get("MoveArmServo");
    }

    private void startGrabber() {
        LiftSlapServo.setPosition(slapDownPosition);
        MoveArmServo.setPosition(armInsidePosition);
        GrabberServo.setPosition(grabberOpenPosition);
        slapStartTime = getRuntime();
    }

    private void checkGrabber() {
        telemetry.addData("liftGrabberState", liftGrabberState);
        telemetry.addData("grabberReturnState", grabberReturnState);
        telemetry.addData("EmergencyStoneEjectState", emergencyStoneEjectState);
        telemetry.addData("capstoneState", capstoneState);
        telemetry.addData("readyToRelease", readyToRelease);
        telemetry.addData("readyToReleaseFromDelivery", readyToReleaseFromDelivery);


        /*slapCurrentTime = getRuntime();
        if (slapCurrentTime - slapStartTime > 1 && slapServoButton < .5) {
            LiftSlapServo.setPosition(slapDownPosition);
        }
        else {
            LiftSlapServo.setPosition(slapUpPosition);
        }*/

        if (liftEncoderState) {
            telemetry.addData("readyToGrab", readyToGrab);

            if (liftGrabberState == 0 && grabberReturnState == 0 && emergencyStoneEjectState == 0 && !readyToRelease && !readyToReleaseFromDelivery) {
                if (grabStoneButton > .5) {
                    if (firstPressRightTrigger) {
                        LiftMotor.setPower(.75);
                        LiftMotor.setTargetPosition((int)(globalLiftOffset * countsPerInch));
                        liftGrabberState = 1;
                        startGrabberTime = getRuntime();
                        firstPressRightTrigger = false;
                    }
                }
                else {
                    firstPressRightTrigger = true;
                }
            }
            grabRotateRaiseStone();

            if (capstoneButton) {
                capstoneState = 1;
                if (firstPressx) {
                    if (grabberReturnState == 1 && readyToReleaseStone) {
                        GrabberServo.setPosition(grabberWideOpenPosition);
                        grabberReturnState = 2;
                    }
                }
            }
            else {
                firstPressx = false;
            }

            if (readyToRelease) {
                if (releaseStoneButton > .5) {
                    if (firstPressLeftTrigger) {
                        if (grabberReturnState == 0) {
                            liftGrabberState = 0;
                            grabberReturnState = 1;
                            startGrabberTime = getRuntime();
                            firstPressLeftTrigger = false;
                        }
                        else if (grabberReturnState == 1 && readyToReleaseStone) {
                            if (capstoneState == 0) {
                                GrabberServo.setPosition(grabberOpenPosition);
                            }
                            else {
                                GrabberServo.setPosition(grabberWideOpenPosition);
                            }
                            grabberReturnState = 2;
                        }
                    }
                } else {
                    firstPressLeftTrigger = true;
                }
                if (grabStoneButton > .5) {
                    if (grabberReturnState == 1 && readyToReleaseStone) {
                        LiftMotor.setPower(.5);
                        LiftMotor.setTargetPosition((int)(liftHeight * (4 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset));
                        grabberReturnState = 0;
                        readyToReleaseStone = false;
                    }
                }
            }
            else if (readyToReleaseFromDelivery) {
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
            }
            returnGrabberArm();

            if (emergencyEjectButton) {
                if (firstPressy) {
                    emergencyStoneEjectState = 1;
                    startGrabberTime = getRuntime();
                    firstPressy = false;
                }
            } else {
                firstPressy = true;
            }
            emergencyStoneEject();

            if (quickUpDownButton > .5) {
                if (liftGrabberState == 0 && grabberReturnState == 0 && !readyToRelease && !readyToReleaseFromDelivery && emergencyStoneEjectState == 0) {
                    if (firstPressQuickUpDown) {
                        quickUpDownState = 1;
                        firstPressQuickUpDown = false;
                    }
                }
            }
            else {
                firstPressQuickUpDown = true;
            }
            quickUpDownLift();

            if ((grabberManualButton || armManualButton) && liftGrabberState == 0 && grabberReturnState == 0 && autoDeliverStoneState == 0 && !readyToRelease && !readyToReleaseFromDelivery && emergencyStoneEjectState == 0 && quickUpDownState == 0) {
                LiftMotor.setPower(.7);
                LiftMotor.setTargetPosition(0);
            }
            else if (liftGrabberState == 0 && grabberReturnState == 0 && !readyToRelease && !readyToReleaseFromDelivery && autoDeliverStoneState == 0 && emergencyStoneEjectState == 0 && quickUpDownState == 0){
                LiftMotor.setPower(.7);
                LiftMotor.setTargetPosition((int)((bottomLiftPosition * countsPerInch) + (globalLiftOffset * countsPerInch)));
            }
        }
        else {
            if (grabberManualButton) {
                if (firstPressGrabberManual) {
                    startGrabberTime = getRuntime();
                    firstPressGrabberManual = false;
                    if (grabberManualClosed) {
                        GrabberServo.setPosition(grabberOpenPosition);
                    }
                    else {
                        GrabberServo.setPosition(grabberClosedPosition);
                    }
                    grabberManualClosed = !grabberManualClosed;
                }
                currentGrabberTime = getRuntime();
                if (currentGrabberTime - startGrabberTime > 1.5) {
                    GrabberServo.setPosition(grabberWideOpenPosition);
                }
            }
            else {
                firstPressGrabberManual = true;
            }

            if (armManualButton) {
                if (firstPressArmManual) {
                    firstPressArmManual = false;
                    if (armManualClosed) {
                        MoveArmServo.setPosition(armOutsidePosition);
                    }
                    else {
                        MoveArmServo.setPosition(armInsidePosition);
                    }
                    armManualClosed = !armManualClosed;
                }
            }
            else {
                firstPressArmManual = true;
            }
        }
    }

    private void grabRotateRaiseStone() {
        if(liftGrabberState == 1) {
            flipCounter = 0;
            if (LiftMotor.getCurrentPosition() < (int)(.25*countsPerInch) + (int)(globalLiftOffset * countsPerInch)) {
                startGrabberTime = getRuntime();
                if (switchPositionButton) {
                    GrabberServo.setPosition(grabberOpenPosition);
                }
                else {
                    GrabberServo.setPosition(grabberClosedPosition);
                }
                liftGrabberState++;
            }
        }
        if(liftGrabberState == 2) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > .35) {
                liftGrabberState++;
                startGrabberTime = getRuntime();
            }
        }
        else if(liftGrabberState == 3) {
            currentGrabberTime = getRuntime();
            if (liftHeight <= 1) {
                LiftMotor.setPower(1);
                LiftMotor.setTargetPosition((int)((10 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset));
                if(LiftMotor.getCurrentPosition() > (int)((9 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset)) {
                    MoveArmServo.setPosition(armOutsidePosition);
                    if (currentGrabberTime - startGrabberTime > 2) {
                        liftUpCommand = true;
                        readyToRelease = true;
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
        else if(liftGrabberState == 4) {
            currentGrabberTime = getRuntime();
            if (LiftMotor.getCurrentPosition() > (int)((liftHeight * (4 * countsPerInch)) + (int)(globalLiftOffset * countsPerInch) + (liftOffset + (-.5 * countsPerInch)))) {
                liftGrabberState++;
            }
            else if (LiftMotor.getCurrentPosition() > ((9 + (int)(globalLiftOffset * countsPerInch)))) {
                MoveArmServo.setPosition(.6);
            }
        }
        else if(liftGrabberState == 5) {
            MoveArmServo.setPosition(armOutsidePosition);
            readyToRelease = true;
            liftGrabberState = 0;
        }
    }

    private void returnGrabberArm() {
        if (readyToRelease && grabberReturnState == 0) {
            LiftMotor.setPower(.5);
            LiftMotor.setTargetPosition((int)(liftHeight * (4 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset + (1 * countsPerInch)));
        }
        else if (readyToRelease || grabberReturnType == 0) {
            if (grabberReturnState == 1) {
                LiftMotor.setPower(.25);
                LiftMotor.setTargetPosition((int) ((liftHeight * (4 * countsPerInch)) + (int)(globalLiftOffset * countsPerInch) + liftOffset - (2 * countsPerInch)));
                if (LiftMotor.getCurrentPosition() < (int) (liftHeight * (4 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset - (1.75 * countsPerInch))) { //was 1.5
                    startGrabberTime = getRuntime();
                    readyToReleaseStone = true;
                }
                grabberReturnType = 0;
            }
            if (grabberReturnState == 2) {
                if (readyToReleaseStone) {
                    ScoringXPosition = RobotXPosition;
                    ScoringYPosition = RobotYPosition;
                    ScoringRotation = RobotRotation;
                }
                readyToReleaseStone = false;
                currentGrabberTime = getRuntime();
                if (capstoneState == 0) {
                    if (currentGrabberTime - startGrabberTime > .3) {
                        LiftMotor.setPower(1);
                        LiftMotor.setTargetPosition((int) ((liftHeight * (4 * countsPerInch)) + (int)(globalLiftOffset * countsPerInch) + liftOffset + (1.5 * countsPerInch)));
                        if (LiftMotor.getCurrentPosition() > (int) (liftHeight * (4 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset + (.75 * countsPerInch))) {
                            grabberReturnState++;
                            startGrabberTime = getRuntime();
                        }
                    }
                }
                else {
                    if (currentGrabberTime - startGrabberTime > .75) {
                        LiftMotor.setPower(1);
                        LiftMotor.setTargetPosition((int) ((liftHeight * (4 * countsPerInch)) + (int)(globalLiftOffset * countsPerInch) + liftOffset + (2 * countsPerInch)));
                        if (LiftMotor.getCurrentPosition() > (int) (liftHeight * (4 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset + (.75 * countsPerInch))) {
                            grabberReturnState++;
                            startGrabberTime = getRuntime();
                        }
                    }
                }
            }
            if (grabberReturnState == 3) {
                readyToRelease = false;
                currentGrabberTime = getRuntime();
                if (currentGrabberTime - startGrabberTime > .5) {
                    //readyToRelease = false;

                    if (liftHeight <= 1) {
                        LiftMotor.setPower(1);
                        LiftMotor.setTargetPosition((int) ((6 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset));
                        if (LiftMotor.getCurrentPosition() > (int) ((5 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset)) {
                            MoveArmServo.setPosition(armInsidePosition);
                            grabberReturnState++;
                            startGrabberTime = getRuntime();
                        }
                    } else {
                        MoveArmServo.setPosition(armInsidePosition);
                        grabberReturnState++;
                        startGrabberTime = getRuntime();
                    }
                }

            } else if (grabberReturnState == 4) {
                currentGrabberTime = getRuntime();
                if (currentGrabberTime - startGrabberTime > .7) {
                    GrabberServo.setPosition(grabberOpenPosition);
                    liftDownCommand = true;
                    grabberReturnState = 0;
                    grabberReturnType = 2;
                    capstoneState = 0;
                }
                else if (currentGrabberTime - startGrabberTime > .25 && liftHeight >= 3) {
                    GrabberServo.setPosition(grabberOpenPosition);
                    liftDownCommand = true;
                    grabberReturnState = 0;
                    grabberReturnType = 2;
                    capstoneState = 0;
                }
            }
        }
        else if(readyToReleaseFromDelivery || grabberReturnType == 1) {
            if (grabberReturnState == 1) {
                GrabberServo.setPosition(grabberOpenPosition);
                ScoringXPosition = RobotXPosition;
                ScoringYPosition = RobotYPosition;
                ScoringRotation = RobotRotation;
                grabberReturnState++;
                startGrabberTime = getRuntime();
                grabberReturnType = 1;
            }
            if (grabberReturnState == 2) {
                currentGrabberTime = getRuntime();
                if (currentGrabberTime - startGrabberTime > .5) {
                    LiftMotor.setPower(1);
                    LiftMotor.setTargetPosition((int) ((6 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset));
                    if (LiftMotor.getCurrentPosition() > (int) ((5 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset)) {
                        MoveArmServo.setPosition(armInsidePosition);
                        grabberReturnState++;
                        startGrabberTime = getRuntime();
                    }
                }
            }
            if (grabberReturnState == 3) {
                currentGrabberTime = getRuntime();
                if (currentGrabberTime - startGrabberTime > 1) {
                    readyToReleaseFromDelivery = false;
                    liftDownCommand = true;
                    grabberReturnState = 0;
                    grabberReturnType = 2;
                }
            }
        }
    }

    private void emergencyStoneEject() {
        if(emergencyStoneEjectState == 1) {
            LiftMotor.setPower(.75);
            LiftMotor.setTargetPosition((int)(globalLiftOffset * countsPerInch));
            if(LiftMotor.getCurrentPosition() < (int)(countsPerInch / 4) + (int)(globalLiftOffset * countsPerInch)) {
                GrabberServo.setPosition(grabberClosedPosition);
                emergencyStoneEjectState++;
                startGrabberTime = getRuntime();
            }
        }
        else if(emergencyStoneEjectState == 2) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > .3) {
                LiftMotor.setPower(1);
                LiftMotor.setTargetPosition((int)((9 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset));
                if(LiftMotor.getCurrentPosition() > (int)((7 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset)) {
                    MoveArmServo.setPosition(.55);
                    startGrabberTime = getRuntime();
                    emergencyStoneEjectState++;
                }
            }
        }
        else if(emergencyStoneEjectState == 3) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > .5) {
                GrabberServo.setPosition(grabberOpenPosition);
                MoveArmServo.setPosition(armInsidePosition);
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

    private void quickUpDownLift() {
        if(quickUpDownState == 1) {
            LiftMotor.setPower(1);
            LiftMotor.setTargetPosition((int)((5 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset));
            if(LiftMotor.getCurrentPosition() > (int)((4 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset)) {
                quickUpDownState++;
            }
        }
        if(quickUpDownState == 2) {
            liftDownCommand = true;
            quickUpDownState = 0;
        }
    }

    private void autoDeliverStone() {
        if(autoDeliverStoneState == 1) {
            if (LiftMotor.getCurrentPosition() < (int)(.25*countsPerInch) + (int)(globalLiftOffset * countsPerInch)) {
                startGrabberTime = getRuntime();
                GrabberServo.setPosition(grabberClosedPosition);
                autoDeliverStoneState++;
            }
        }
        if(autoDeliverStoneState == 2) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > .3) {
                autoDeliverStoneState++;
                startGrabberTime = getRuntime();
            }
        }
        else if(autoDeliverStoneState == 3) {
            currentGrabberTime = getRuntime();
            LiftMotor.setPower(1);
            LiftMotor.setTargetPosition((int)((9 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset));
            if(LiftMotor.getCurrentPosition() > (int)((8 * countsPerInch) + (int)(globalLiftOffset * countsPerInch) + liftOffset)) {
                MoveArmServo.setPosition(armOutsidePosition);
                if (currentGrabberTime - startGrabberTime > 1.5) {
                    LiftMotor.setPower(.25);
                    LiftMotor.setTargetPosition((int)(globalLiftOffset * countsPerInch));
                    readyToReleaseFromDelivery = true;
                    autoDeliverStoneState = 0;
                }
            }
        }
    }

    //
    // DRIVE TRAIN
    //

    private void initializeDriveTrain() {
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
        if (autoDriveButton) {
            if (firstPressBumpers) {
                firstPressBumpers = false;
            }
            goToPositionMrK(ScoringXPosition,ScoringYPosition,1,.75, ScoringRotation);
        }
        else {
            firstPressBumpers = true;
        }

        /*if (autoRemoveFoundationButton) {
            if (firstRunRemoveFoundation) {
                lowerFoundationator();
                StartingFoundationXPosition = RobotXPosition;
                StartingFoundationYPosition = RobotYPosition;
                StartingFoundationRotation = RobotRotation;
                firstRunRemoveFoundation = false;
                foundationNotInPosition = true;
            }
            else {
                if (foundationNotInPosition) {
                    if (readyToRelease) {
                        LiftMotor.setTargetPosition((int) ((liftHeight * (4 * countsPerInch)) + liftOffset - (2.5 * countsPerInch)));
                        if (LiftMotor.getCurrentPosition() < (int) (liftHeight * (4 * countsPerInch) + liftOffset - (2.25 * countsPerInch))) { //was 1.5
                            goToPositionMrK((StartingFoundationXPosition), (StartingFoundationYPosition + 30), .25, 0, Math.toDegrees(AngleWrap(Math.toRadians(StartingFoundationRotation))));
                            if (Math.abs(distanceToTarget) < 1) {
                                foundationNotInPosition = false;
                            }
                        }
                    } else {
                        goToPositionMrK((StartingFoundationXPosition), (StartingFoundationYPosition + 30), .25, 0, Math.toDegrees(AngleWrap(Math.toRadians(StartingFoundationRotation))));
                        if (Math.abs(distanceToTarget) < 1) {
                            foundationNotInPosition = false;
                        }
                    }
                }
                else {
                    releaseStoneButton = 1;
                    if (grabberReturnState >= 2) {
                        raiseFoundationator();
                    }
                }
            }
        }
        else {
            firstRunRemoveFoundation = true;
        }*/

        if (autoRemoveFoundationButton) {
            if (firstDeliverStoneButton) {
                LiftMotor.setPower(.75);
                LiftMotor.setTargetPosition((int)(globalLiftOffset * countsPerInch));
                autoDeliverStoneState = 1;
                startGrabberTime = getRuntime();
                firstDeliverStoneButton = false;
            }
        }
        else {
            firstDeliverStoneButton = true;
        }
        autoDeliverStone();

        if (!autoDriveButton && !autoRemoveFoundationButton) {
            movement_y = DeadModifier(-gamepad1.left_stick_y);
            movement_x = DeadModifier(gamepad1.left_stick_x);
            movement_turn = DeadModifier(.75 * gamepad1.right_stick_x);

            if (halfSpeedDriveButton || readyToRelease || liftGrabberState >= 1 || !isWaffleStateRaised) {
                movement_y = movement_y / 3;
                movement_x = movement_x / 3;
                movement_turn = movement_turn / 3;
            }

            applyMovement();
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
     * @param y global target coordinate 'y' component
     * @param maxMovementSpeed max speed value to be given to any one drivetrain direction of motion
     * @param maxTurnSpeed max turning speed to be given to drivetrain rotation
     * @param preferredAngle global target coordinate theta component
     */
    private void goToPositionMrK(double x, double y, double maxMovementSpeed, double maxTurnSpeed, double preferredAngle) {
        distanceToTarget = Math.hypot(x-RobotXPosition, y-RobotYPosition);
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

        StartingFoundationXPosition = 0;
        StartingFoundationYPosition = 0;
        StartingFoundationRotation = 0;
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
        IntakeAssistMotor = hardwareMap.dcMotor.get("IntakeAssistMotor");
        IntakeReleaseServo = hardwareMap.servo.get("IntakeReleaseServo");
        IntakeMotor.setPower(0);
        //IntakeAssistServo.setPower(0);
        IntakeAssistMotor.setPower(0);
    }
    private void startIntakeMechanism() {
        IntakeReleaseServo.setPosition(.6);
        IntakeAssistMotor.setPower(-1);
    }

    private void checkIntakeMechanism() {
        intake(intakeButton, outtakeButton);
    }

    private void intake(boolean intakeButton, boolean outtakeButton) {
        if (intakeButton || outtakeButton) {
            if (intakeButton) {
                if (firstPressDpadUp) {
                    if (intakeState == 1) {
                        intakeState = 0;
                    } else {
                        intakeState = 1;
                    }
                    firstPressDpadUp = false;
                }
            }
            else {
                firstPressDpadUp = true;
            }

            if (outtakeButton) {
                IntakeMotor.setPower(-1);
            }
            else {
                IntakeMotor.setPower(1);
            }
        }
        else {
            if ((stoneDistance < 1.5 && stoneDistance != 0.0) || !isWaffleStateRaised) {
                IntakeMotor.setPower(0);
            }
            else if (!readyToGrab && liftGrabberState == 0 && emergencyStoneEjectState == 0 && (grabberReturnState >= 2 || (!readyToRelease && grabberReturnState == 0))) {
                if (checkForJammedStone()) {
                    firstRunIntakeReverse = true;
                }

                if (firstRunIntakeReverse) {
                    firstIntakeReverseTime = getRuntime();
                    runIntakeReverse = true;
                    firstRunIntakeReverse = false;
                }
                if (runIntakeReverse) {
                    intakeReverseTime = getRuntime();

                    if (intakeReverseTime - firstIntakeReverseTime >= .25) {
                        runIntakeReverse = false;
                    }
                    else {
                        IntakeMotor.setPower(-1);
                    }
                }
                else {
                    IntakeMotor.setPower(1);
                }
            }
        }
    }

    private boolean checkForJammedStone() {
        lastIntakeEncoderPosition = intakeEncoderPosition;
        intakeEncoderPosition = IntakeMotor.getCurrentPosition();
        telemetry.addData("IntakeEncoderPosition", intakeEncoderPosition);
        telemetry.addData("IntakeEncoderDifference", intakeEncoderPosition - lastIntakeEncoderPosition);

        if (intakeEncoderPosition - lastIntakeEncoderPosition < 30 && !runIntakeReverse) {
            jammedIntakeLoopCounter++;
        }
        else {
            jammedIntakeLoopCounter = 0;
        }
        telemetry.addData("JammedIntakeLoopCounter", jammedIntakeLoopCounter);

        if (jammedIntakeLoopCounter >= 3) {
            jammedIntakeLoopCounter = 0;
            return true;
        }
        else {
            return false;
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
        if (stoneDistance < 1.5) {
            IntakeMotor.setPower(0);
        }
        if (slapServoButton > .5) {
            OrientationServoLeft.setPosition(lDisengage);
            OrientationServoRight.setPosition(rDisengage);
        }
        else {
            orientStone();
        }
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
        senseSkyStone();

        if (!manualLeftServoButton && !manualRightServoButton) {

            if (!straightenerBusy) {
                OrientationServoLeft.setPosition(lDisengage);
                OrientationServoRight.setPosition(rDisengage);
            }

            if (stoneFullyInStraightener) {
                if (stoneOrientation.equals("right")) {
                    runLeftServo();
                    readyToGrab = false;
                } else if (stoneOrientation.equals("left")) {
                    runRightServo();
                    readyToGrab = false;
                } else if (stoneOrientation.equals("empty")) {
                    readyToGrab = false;
                    OrientationServoLeft.setPosition(lDisengage);
                    OrientationServoRight.setPosition(rDisengage);
                } else if (stoneOrientation.equals("center")) {
                    readyToGrab = true;
                    OrientationServoLeft.setPosition(lDisengage);
                    OrientationServoRight.setPosition(rDisengage);
                } else if (stoneOrientation.equals("notInThreshold")) {
                    readyToGrab = false;
                    OrientationServoLeft.setPosition(lDisengage);
                    OrientationServoRight.setPosition(rDisengage);
                }
                telemetry.addData("actualLeftTime", actualLeftTime);
                telemetry.addData("actualRightTime", actualRightTime);
                telemetry.addData("target time", targetTime);
                telemetry.addData("orientPosition", stoneOrientation);
                telemetry.addData("StraightenerBusy", straightenerBusy);
            } else {
                OrientationServoLeft.setPosition(lDisengage);
                OrientationServoRight.setPosition(rDisengage);
                readyToGrab = false;
                if (!straightenerBusy) {
                    stoneOrientation = "empty";
                } else {
                    straightenerBusy = false;
                }
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
        if(firstRightRun && !straightenerBusy && !isSkyStoneInView && flipCounter <= 2) {
            OrientationServoRight.setPosition(rEngage);
            startRightTime = getRuntime();
            firstRightRun = false;
            straightenerBusy = true;
            flipCounter++;
        }
        currentRightTime = getRuntime();
        actualRightTime = (currentRightTime-startRightTime);

        if(actualRightTime > targetTime) {
            OrientationServoRight.setPosition(rDisengage);
            //firstRightRun = true;
        }

        if (actualRightTime > targetTime*2.5) {
            firstRightRun = true;
            straightenerBusy = false;

        }
        else {
            straightenerBusy = true;
        }
    }

    private void runLeftServo() {
        if(firstLeftRun && !straightenerBusy && !isSkyStoneInView && flipCounter <= 2) {
            OrientationServoLeft.setPosition(lEngage);
            startLeftTime = getRuntime();
            firstLeftRun = false;
            straightenerBusy = true;
            flipCounter++;
        }
        currentLeftTime = getRuntime();
        actualLeftTime = (currentLeftTime-startLeftTime);

        if(actualLeftTime > targetTime) {
            OrientationServoLeft.setPosition(lDisengage);
            firstLeftRun = true;
        }

        if (actualLeftTime > targetTime*2.5) {
            OrientationServoLeft.setPosition(lDisengage);
            firstLeftRun = true;
            straightenerBusy = false;
        }
        else {
            straightenerBusy = true;
        }
    }

    private void initializeSkyStoneColorSensor(){
        SkyStoneSensor =  hardwareMap.get(NormalizedColorSensor.class, "SkyStoneSensor");
    }

    private void senseSkyStone() {
        NormalizedRGBA SkyStoneValues = SkyStoneSensor.getNormalizedColors();

        redValues = SkyStoneValues.red;
        blueValues = SkyStoneValues.blue;
        greenValues = SkyStoneValues.green;
        alphaValues = SkyStoneValues.alpha;

        Color.RGBToHSV((int) (SkyStoneValues.red * SCALE_FACTOR),
                (int) (SkyStoneValues.green * SCALE_FACTOR),
                (int) (SkyStoneValues.blue * SCALE_FACTOR),
                hsvValues);

        telemetry.addData("red", SkyStoneValues.red);
        telemetry.addData("green", SkyStoneValues.green);
        telemetry.addData("blue", SkyStoneValues.blue);
        telemetry.addData("alpha", SkyStoneValues.alpha);
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);

        telemetry.addData("skystone", isSkyStoneInView);
        telemetry.addData("red bool", red);

        if (hsvValues[0] > 110 && hsvValues[1] > .25) {
            //just changed from .4 to .25 02/09/20 if problems flipping normal stone turn it back to .4
            isSkyStoneInView = true;
        }
        else {
            isSkyStoneInView = false;
        }
    }

    private void initializeTapeMeasure() {
        TapeMeasureMotor = hardwareMap.dcMotor.get("TapeMeasureMotor");
        TapeMeasureMotor.setPower(0);
        TapeMeasureMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void checkTapeMeasure() {
        TapeMeasureMotor.setPower(-tapeMeasureButton);
    }
}