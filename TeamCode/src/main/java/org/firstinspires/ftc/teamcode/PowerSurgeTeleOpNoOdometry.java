package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "PowerSurgeTeleOp")
public class PowerSurgeTeleOpNoOdometry extends OpMode {

    // DRIVETRAIN

    public static final double DEADZONE = 0.15;

    DcMotor verticalRight, verticalLeft, horizontal;

    String verticalLeftEncoderName = "FrontLeft";
    String verticalRightEncoderName = "BackRight";
    String horizontalEncoderName = "BackLeft";

    final double COUNTS_PER_INCH = 307.699557;

    private double movement_x;
    private double movement_y;
    private double movement_turn;

    private double rightBackupDistance;
    private double leftBackupDistance;

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
    private double capstoneButton;
    private boolean grabberManualButton;
    private boolean armManualButton;

    private boolean outputButton;
    private boolean intakeButton;
    private boolean intakeReleaseButton;

    private boolean manualLeftServoButton;
    private boolean manualRightServoButton;
    private double readyToGrabOverideButton;

    private boolean halfSpeedDriveButton;

    private boolean killSwitch;
    private boolean killSwitch2;

    // FIRST PRESS AND RUN

    private boolean liftUpCommand;
    private boolean liftDownCommand;
    private boolean firstPressDpadUp = true;
    private boolean liftEncoderState = true;
    private boolean firstPressy = true;
    private boolean firstPressx2 = true;
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
    static final double wheelDiameter           = 1.7; //1.771653543307087
    static final double countsPerInch           = (countsPerMotor * gearReduction) / (wheelDiameter * Math.PI);
    static final double liftOffset = (2.5 * countsPerInch);

    // ORIENTER STUFF

    private String stoneOrientation = "empty";
    private double foundationatorPosition = .335;
    private double orientDistance = 0;
    private double lDisengage = .2;
    private double rDisengage = .9;
    private double lEngage = .85;
    private double rEngage = .25;
    private double startRightTime = 0;
    private double currentRightTime = 0;
    private double actualRightTime = 1.5;
    private double startLeftTime = 0;
    private double currentLeftTime = 0;
    private double actualLeftTime = 1.5;
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
    private double grabberClosedPosition = 0;
    private double armInsidePosition = 1;
    private double armOutsidePosition = 0;
    private boolean grabberManualClosed = false;
    private boolean armManualClosed = true;
    private boolean readyToRelease = false;
    private int capstoneState = 0;

    //last update time
    private long lastUpdateTime = 0;

    //Color sensor Stuff
    public NormalizedColorSensor SkyStoneSensor;

    //View relativeLayout;

    public boolean isSkyStoneInView = false;

    public float redValues = 0;
    public float blueValues = 0;
    public float greenValues = 0;

    public boolean red = false;

    // Calculate # of loops per second for testing
    private int loopCount = 0;
    private double loopStartTime = 0.0;
    private int loopsPerSecond = 0;

    @Override
    public void init() {
        telemetry.addData("Version Number", "1-3-20 1000pm");
        initializeVerticalLift();
        initializeFoundationator();
        initializeGrabber();
        initializeDriveTrain();
        initializeIntakeMechanism();
        initializeStraightener();
        initializeSkyStoneColorSensor();
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        startFoundationator();
        startIntakeMechanism();
        startVerticalLift();
        startGrabber();
        telemetry.addData("Status", "Odometry System has started");
        telemetry.update();

        // Calculate # of loops per second for testing
        loopStartTime = getRuntime();
    }

    @Override
    public void loop() {

        // Calculate # of loops per second for testing
        double currentTime = getRuntime();
        loopCount++;
        if (currentTime > loopStartTime + 5) {
            loopsPerSecond = loopCount;
            loopCount = 0;
            loopStartTime = currentTime;
        }
        telemetry.addData("LPS: ", loopsPerSecond);

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
        capstoneButton = gamepad2.right_trigger;

        outputButton = gamepad1.dpad_down;
        intakeButton = gamepad1.dpad_up;
        intakeReleaseButton = gamepad1.x;

        manualLeftServoButton = gamepad2.dpad_right;
        manualRightServoButton = gamepad2.dpad_left;
        readyToGrabOverideButton = gamepad2.left_trigger;

        halfSpeedDriveButton = gamepad1.left_bumper;

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
        MoveArmServo.setPosition(armInsidePosition);
        GrabberServo.setPosition(grabberOpenPosition);
    }

    private void checkGrabber() {
        if (readyToGrabOverideButton > .5) {
            readyToGrab = true;
        }

        if (liftEncoderState) {
            telemetry.addData("readyToGrab", readyToGrab);
            if (readyToGrab && !readyToRelease && liftGrabberState == 0 && grabberReturnState == 0 && emergencyStoneEjectState == 0) {
                if (firstRunZeroLiftPosition) {
                    startGrabberTime = getRuntime();
                    firstRunZeroLiftPosition = false;
                }
                currentGrabberTime = getRuntime();
                if (currentGrabberTime - startGrabberTime > .75) {
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

            if (readyToRelease) {
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

            if (capstoneButton > .5) {
                if (firstPressCapstoneButton) {
                    capstoneState = 1;
                    LiftMotor.setTargetPosition((int)((10 * countsPerInch)));
                    firstPressCapstoneButton = false;
                }
            }
            else {
                firstPressCapstoneButton = true;
            }
            scoreCapstone();

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
                    MoveArmServo.setPosition(armOutsidePosition);
                }
                else {
                    MoveArmServo.setPosition(armInsidePosition);
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
        else if(liftGrabberState == 3) {
            currentGrabberTime = getRuntime();
            if (LiftMotor.getCurrentPosition() > (int)(liftHeight * (4 * countsPerInch) + (liftOffset-(.5 * countsPerInch)))) {
                liftGrabberState++;
            }
        }
        else if(liftGrabberState == 4) {
            MoveArmServo.setPosition(armOutsidePosition);
            readyToRelease = true;
            liftGrabberState = 0;
        }
    }

    private void returnGrabberArm() {
        if (grabberReturnState == 1) {
            LiftMotor.setPower(.5);
            LiftMotor.setTargetPosition((int)((liftHeight * (4 * countsPerInch)) + liftOffset - (2 * countsPerInch)));
            if (LiftMotor.getCurrentPosition() < (int)(liftHeight * (4 * countsPerInch) + liftOffset - (1.5 * countsPerInch))) {
                GrabberServo.setPosition(grabberOpenPosition);
                grabberReturnState++;
                startGrabberTime = getRuntime();
            }
        }
        if (grabberReturnState == 2) {
            LiftMotor.setPower(1);
            LiftMotor.setTargetPosition((int)((liftHeight * (4 * countsPerInch)) + liftOffset + (1 * countsPerInch)));
            if (LiftMotor.getCurrentPosition() > (int)(liftHeight * (4 * countsPerInch) + liftOffset + (.5 * countsPerInch))) {
                grabberReturnState++;
                startGrabberTime = getRuntime();
            }
        }
        if (grabberReturnState == 3) {
            currentGrabberTime = getRuntime();

            if(currentGrabberTime - startGrabberTime > .5) {
                if (liftHeight <= 1) {
                    LiftMotor.setPower(1);
                    LiftMotor.setTargetPosition((int)((6 * countsPerInch) + liftOffset));
                    if(LiftMotor.getCurrentPosition() > (int)((5 * countsPerInch) + liftOffset)) {
                        MoveArmServo.setPosition(armInsidePosition);
                        grabberReturnState++;
                        startGrabberTime = getRuntime();
                    }
                }
                else {
                    MoveArmServo.setPosition(armInsidePosition);
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

    private void scoreCapstone() {
        if(capstoneState == 1) {
            if (LiftMotor.getCurrentPosition() > (int)((9.5 * countsPerInch) + liftOffset)) {
                MoveArmServo.setPosition(.25);
                startGrabberTime = getRuntime();
                capstoneState++;
            }
        }
        if(capstoneState == 2) {
            if(currentGrabberTime - startGrabberTime > .5) {
                LiftMotor.setTargetPosition((int)((8 * countsPerInch)));
                if (LiftMotor.getCurrentPosition() < (int)((8.5 * countsPerInch) + liftOffset)) {
                    startGrabberTime = getRuntime();
                    GrabberServo.setPosition(grabberClosedPosition);
                    capstoneState++;
                }
            }
        }
        if(capstoneState == 3) {
            if (currentGrabberTime - startGrabberTime > .5) {
                liftUpCommand = true;
                capstoneState++;
                startGrabberTime = getRuntime();
            }
        }
        if(capstoneState == 4) {
            if (LiftMotor.getCurrentPosition() > (int)(liftHeight * (4 * countsPerInch) + (liftOffset-(.5 * countsPerInch)))) {
                MoveArmServo.setPosition(armOutsidePosition);
                startGrabberTime = getRuntime();
                capstoneState++;
            }
        }
        if(capstoneState == 5) {
            if (currentGrabberTime - startGrabberTime > 1) {
                readyToRelease = true;
                capstoneState++;
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

        movement_y = DeadModifier(-gamepad1.left_stick_y);
        movement_x = DeadModifier(gamepad1.left_stick_x);
        movement_turn = DeadModifier(gamepad1.right_stick_x);

        if (halfSpeedDriveButton) {
            movement_y = movement_y / 5;
            movement_x = movement_x / 3;
            movement_turn = movement_turn / 5;
        }

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

    // By Paul

    private double DeadModifier(double joystickValue) {
        if(joystickValue < DEADZONE && joystickValue > -DEADZONE)
            return 0;
        else {
            return joystickValue;
        }
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
            else if(stoneOrientation.equals("center") || isSkyStoneInView) {
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

    private void initializeSkyStoneColorSensor(){
        SkyStoneSensor =  hardwareMap.get(NormalizedColorSensor.class, "SkyStoneSensor");
    }

    private void senseSkyStone() {
        NormalizedRGBA SkyStoneValues = SkyStoneSensor.getNormalizedColors();

        redValues = SkyStoneValues.red;
        blueValues = SkyStoneValues.blue;
        greenValues = SkyStoneValues.green;

        telemetry.addData("red", SkyStoneValues.red);
        telemetry.addData("green", SkyStoneValues.green);
        telemetry.addData("blue", SkyStoneValues.blue);
        telemetry.addData("skystone", isSkyStoneInView);
        telemetry.addData("red bool", red);

        if(redValues > .008 && redValues < .01) {
            isSkyStoneInView = true;
        }
        else {
            isSkyStoneInView = false;
        }
    }
}