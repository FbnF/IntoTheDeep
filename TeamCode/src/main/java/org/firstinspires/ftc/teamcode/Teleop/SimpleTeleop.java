package org.firstinspires.ftc.teamcode.Teleop;

// - - - - - - - - - - Imports - - - - - - - - - - - - -

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.ArmControl;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;


//@Config
@TeleOp(group = "Teleop")
public class SimpleTeleop extends LinearOpMode {

    //- - - - - - - - - - - - - - Initialization of Variables - - - - - - - - - - - -
    // - - - Timer for the teleop period - - - //
    private ElapsedTime teleopTimer = new ElapsedTime();
    private static final float TELEOP_TIME_OUT = 140; // Match time limit
    // - - - Timer for the teleop period - - - //

    // - - - Constants + Variables - - - //
    private static final double RAISE_SPEED = 1.0; // Full speed for raising
    private ArmControl armControl;
    private Gripper gripper;
    private SliderControl sliderControl;
    private double speedFactor = 0.6;
    private double RuntoPositionPower =0.4;
    private double DepositAngle=55;
    private int ArmHangInd=0;
    private int ArmLatchInd= 0;
    private int ArmDepositInd=0;
    private int ArmIntakeInd=0;
    private int SliderDepositInd=0;
    private int SliderRetractInd=0;
    private double SliderLenLimit=14.5;
    private double SliderCurLen;
    private double ArmCurPosDeg;
    private int GripperRollInInd=0;
    private int setmode=0;
    private PIDFCoefficients Default_Pid;
    //FtcDashboard dashboard;
    // - - - Constants + Variables - - - //
    //- - - - - - - - - - - - - - Initialization - - - - - - - - - - - -

    @Override
    public void runOpMode() throws InterruptedException {

        // - - - - - - - - - - Initialize components - - - - - - - - - -

        // - - - Setting up Mecanum Drive - - - //
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // - - - Setting up Arm motors - - - //
        armControl = new ArmControl(this);
        armControl.init(hardwareMap);

        // - - - Setting up Slider motors - - - //
        sliderControl = new SliderControl(this);
        sliderControl.init(hardwareMap);
        

        // - - - Set up dashboard telemetry - - - //
        //dashboard = FtcDashboard.getInstance();
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());



        // - - - Initialize gripper to starting position - - - //
        gripper = new Gripper(this);
        gripper.init(hardwareMap);
        gripper.gripperStopped();
        gripper.setAnglerDown();
        
        // - - - Waiting for start signal from driver station - - - //
        waitForStart();
        teleopTimer.reset();
        // - - - Waiting for start signal from driver station - - - //

        // - - - - - - - - - - Initialize components - - - - - - - - - -


        // - - - - - - - - - - Main Teleop Loop - - - - - - - - - -

        while (!isStopRequested()) {

            // - - - Mecanum drive control - - - //
            // Control the robot's movement with the gamepad 1's sticks
            if(gamepad1.y){
                speedFactor = 0.7;
            }
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.right_stick_y * speedFactor, // Forward/Backward Movement
                    -gamepad1.left_stick_x * speedFactor, // Strafing Left/right
                    -gamepad1.right_stick_x * speedFactor // Rotation
            ));


            // - - - Arm Control - - - /
 
            // A button for latching in current position
            if (gamepad2.a) {
                // Set latching indication to 1 to maintain the arm at the current position
                ArmLatchInd = 1;
                ArmIntakeInd=0;
                ArmDepositInd=0;
                ArmHangInd=0;
                ArmCurPosDeg= armControl.getActArmPosDeg();
            }

            if(ArmLatchInd ==1){
                // Holding the Arm in position when ArmLatchInd is 1
                armControl.setDesArmPosDeg(ArmCurPosDeg);
            }

            // Left bumper to set Arm to the Deposit angle for depositing the sample
            if (gamepad2.left_bumper) {
                ArmDepositInd=1;
                ArmIntakeInd=0;
                ArmLatchInd=0;
                ArmHangInd=0;
            }
            if(ArmDepositInd==1){
                armControl.setArmDeposit();
            }
            // Gamepad 1 a button: set arm power to 0.7 to hang the robot
            if (gamepad1.a) {
                ArmDepositInd=0;
                ArmIntakeInd=0;
                ArmLatchInd=0;
                ArmHangInd=1;
                ArmCurPosDeg= armControl.getActArmPosDeg();
            }
            if(ArmHangInd==1){
                armControl.setArmHanging(ArmCurPosDeg);

            }
            // Allow user to control the arm position once it is pushed more than 0.1 in magnitude
            if (Math.abs(gamepad2.right_stick_y) > 0.2) {
                    // Reset all the position indicators
                    ArmIntakeInd = 0;
                    ArmLatchInd=0;
                    ArmDepositInd=0;
                    ArmHangInd=0;
                    armControl.ArmRunModReset();
                    armControl.setArmPower(-1.0 * gamepad2.right_stick_y * 0.8);
                } else {
                if( ArmIntakeInd==0 && ArmLatchInd==0 && ArmDepositInd==0 && ArmHangInd==0) {
                    // zero power plus run mode reset
                    armControl.ArmRunModReset();
                }
            }

            // - - - Slider motor control - - - //
            // gamepad1 B button will set gripper rolling in for depositing the specimen
            if(gamepad1.b) {
                GripperRollInInd=1;
            }
            if(GripperRollInInd==1){
                gripper.gripperForward(0.3);
            }

            // gamepad1 X button will stop the Gripper power
            if(gamepad1.x) {
                GripperRollInInd=0;
                gripper.gripperStopped();
            }

            // right bumper to set slider to the full extension for deposit the sample
            if (gamepad2.right_bumper) {
                SliderDepositInd= 1;
                SliderRetractInd=0;
            }
            if (SliderDepositInd==1) {
                sliderControl.setSliderDeposit();;
            }
            // gamepad2 b button for set slider length to 2in less
            if (gamepad2.b) {
                SliderRetractInd=1;
                SliderDepositInd=0;
                SliderCurLen= sliderControl.getSliderLen();
            }
            if( SliderRetractInd==1){
                sliderControl.setDesSliderLen(SliderCurLen);
            }
            // Controlling the slider motor using game pad2's left and right
            // triggers once magnitude > 0.1
            if (gamepad2.left_trigger > 0.2) {
                // Retracting the slider through driver control
                // Reset the run to position indicators

                SliderDepositInd=0;
                SliderRetractInd=0;
                sliderControl.SliderRunModEncoder();
                sliderControl.setSliderPower(-gamepad2.left_trigger * 0.6);

            } else if (gamepad2.right_trigger > 0.2) {

                // extending the slider through driver control
                // Reset the run to position indicators
                SliderDepositInd = 0;
                SliderRetractInd = 0;
                sliderControl.SliderRunModEncoder();
                SliderCurLen = sliderControl.getSliderLen();
                sliderControl.setSliderPower(gamepad2.right_trigger * 0.6);
            } else {
                if (SliderDepositInd==0 && SliderRetractInd==0) {
                    // zero power plus run mode reset
                    sliderControl.SliderRunModReset();
                }
            }


            // - - - Gripper control - - - //
            // Control gripper angler and position using gamepad2's buttons and left stick
            if (gamepad2.x) gripper.setAnglerUP(); // Set angler to up position
            if (gamepad2.y) gripper.setAnglerDown(); // Set angler to down position

            if (gamepad2.left_stick_x > 0.2) {
                gripper.gripperForward(gamepad2.left_stick_x); // Move gripper forward
            } else if (gamepad2.left_stick_x < -0.2) {
                gripper.gripperReverse(gamepad2.left_stick_x); // Move gripper backwards
            } else {
                if(GripperRollInInd==0){
                    gripper.gripperStopped(); // Stop gripper movement
                }
            }


            /*
            // - - - Telemetry Updates - - - //
            // Sending important data to telemetry to monitor
            telemetry.addData("Arm Actual Position in Degree","%.3f", armControl.getActArmPosDeg());
            telemetry.addData("Arm Tgt Position in Ticks", armControl.getTgtArmTick());
            telemetry.addData("Arm Current Position in Ticks", armControl.getActArmTick());
            telemetry.addData("Arm Motor Power", "%.2f",armControl.getArmPower());
            telemetry.addData("Elapsed Time", "%.2f", teleopTimer.time());
            telemetry.addData("TwoStage Position", sliderControl.getSliderLen());
            telemetry.addData("Gripper Roll In Indicator", GripperRollInInd);
            telemetry.addData("Active Speed Factor: ", speedFactor);
            telemetry.update();

             */

        }
    }

}