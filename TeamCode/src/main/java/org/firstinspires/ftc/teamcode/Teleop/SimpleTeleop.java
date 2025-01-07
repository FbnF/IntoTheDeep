package org.firstinspires.ftc.teamcode.Teleop;

// - - - - - - - - - - Imports - - - - - - - - - - - - -

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
    private int ArmLowerInd=0;
    private int ArmFixAngleInd=0;
    private double SliderCurLen;
    private double ArmCurPosDeg;
    private int GripperRollInInd=0;
    private double GripperTeleOpClosePos;
    private PIDFCoefficients Default_Pid;
    private int TighterGripAdjustInd=0;
    private int LooserGripAdjustInd=0;
    private double GripperTeleOpOpenPos;
    // Slider adjust constant for dpad up and down pushes: set to 0.5 inch
    private double SliderAdjust=0.5;
    // Arm lower angle with gamepad 2 dpad_down
    private double ArmLowerAngle=7;
    // Arm hanging power constant: must be negative to close the Arm
    private double ArmHangPower=-0.08;


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
        // Set the Hang servo up to put the blocking plate in place to hold the arm up
        armControl.setHangServoUp();

        // - - - Setting up Slider motors - - - //
        sliderControl = new SliderControl(this);
        sliderControl.init(hardwareMap);


        // - - - Initialize gripper to starting position - - - //
        gripper = new Gripper(this);
        gripper.init(hardwareMap);
        // Obtain the default calibrated gripper close position to assign it to the
        // Teleop Close Position since the gripper does get loose after several usage.
        GripperTeleOpClosePos= gripper.getGripperClosePos();
        // Teleop Open Position for gripper calibration
        GripperTeleOpOpenPos = gripper.getGripperOpenPos();
        //Gripper open state
        //gripper.setGripperOpen();
        //Gripper holder to the side
        //gripper.setGripperHolderParallel();
        //gripper.setAnglerSide();
        
        // - - - Waiting for start signal from driver station - - - //
        waitForStart();
        teleopTimer.reset();


        // - - - - - - - - - - Main Teleop Loop - - - - - - - - - -

        while (!isStopRequested()) {

            // - - - Mecanum drive control - - - //
            // Control the robot's movement with the gamepad 1's sticks
            if(gamepad1.y){
                speedFactor = 0.7;
            }

            // Gamepad1 A button to set to the super slow Mode
            if (gamepad1.a) {
                speedFactor = 0.6;
            }

            // Gamepad1 B button to set to the super slow Mode
            if (gamepad1.b) {
                speedFactor = 0.2;
            }

            // Gamepad1 left_bumper to set the arm blocking down
            if (gamepad1.left_bumper) {
               armControl.setHangServoSide();
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
                ArmLowerInd =0;
                ArmFixAngleInd=0;
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
                ArmLowerInd =0;
                ArmFixAngleInd=0;
            }
            if(ArmDepositInd==1){
                armControl.setArmDeposit();
            }
            // dpad_up to set to desired angle for in/out of the submersible zone
            if (gamepad2.dpad_up) {
                ArmFixAngleInd= 1;
                ArmDepositInd=0;
                ArmIntakeInd=0;
                ArmLatchInd=0;
                ArmHangInd=0;
                ArmLowerInd =0;

            }
            if(ArmFixAngleInd==1) {
                armControl.setDesArmPosDeg(-7);
            }
            // dpad_down to lower the arm by ArmLowerAngle degree of angle
            if (gamepad2.dpad_down) {
                ArmLowerInd = 1;
                ArmDepositInd=0;
                ArmIntakeInd=0;
                ArmLatchInd=0;
                ArmHangInd=0;
                ArmFixAngleInd=0;
                ArmCurPosDeg= armControl.getActArmPosDeg();
            }
            if(ArmLowerInd==1) {
                armControl.setDesArmPosDeg(ArmCurPosDeg-ArmLowerAngle);
            }

            // Gamepad 2 left stick down push: set arm power to ArmHangPower to hang the robot
            if (gamepad2.left_stick_y>0.2) {
                ArmDepositInd=0;
                ArmIntakeInd=0;
                ArmLatchInd=0;
                ArmLowerInd=0;
                ArmHangInd=1;
                ArmFixAngleInd=0;

                ArmCurPosDeg= armControl.getActArmPosDeg();
            }
            if(ArmHangInd==1){
                armControl.ArmRunModEncoder();
                armControl.setArmPower(ArmHangPower);
            }


            // Allow user to control the arm position once it is pushed more than 0.1 in magnitude
            if (Math.abs(gamepad2.right_stick_y) > 0.2 ) {
                    // Reset all the position indicators
                    ArmIntakeInd = 0;
                    ArmLatchInd=0;
                    ArmDepositInd=0;
                    ArmHangInd=0;
                    ArmLowerInd=0;
                    ArmFixAngleInd=0;
                    armControl.ArmRunModEncoder();
                    armControl.setArmPower(-1.0 * gamepad2.right_stick_y * 0.8);
                } else {
                if( ArmIntakeInd==0 && ArmLatchInd==0 && ArmDepositInd==0
                        && ArmHangInd==0 && ArmLowerInd==0 && ArmFixAngleInd==0) {
                    // zero power plus run mode reset
                    armControl.ArmRunModReset();
                }
            }

            // gamepad1.x: Reset Arm motor encode when the arm is fully down:
            // this is to compensate the build up error.
            if (gamepad1.x){
                armControl.ArmEncoderReset();
            }



            // - - - Slider motor control - - - //

            // Controlling the slider motor using game pad2's left and right
            // triggers once magnitude > 0.1
            if (gamepad2.left_trigger > 0.2) {
                sliderControl.SliderRunModEncoder();
                sliderControl.setSliderPower(-gamepad2.left_trigger * 0.6);

            } else if (gamepad2.right_trigger > 0.2) {

                // extending the slider through driver control
                // Reset the run to position indicators
                sliderControl.SliderRunModEncoder();
                SliderCurLen = sliderControl.getSliderLen();
                sliderControl.setSliderPower(gamepad2.right_trigger * 0.6);
            } else {
                    // zero power plus run mode reset
                    sliderControl.SliderRunModReset();

            }


            // - - - Gripper control - - - //
            // gamepad2 b button for open the gripper
            if (gamepad2.b) {
                gripper.setGripperPosition(GripperTeleOpOpenPos);
            }
            // gamepad2 x button for close the gripper
            if (gamepad2.x){
                gripper.setGripperPosition(GripperTeleOpClosePos);
            }

            // - - - Gripper Holder control - - - //
            // gamepad2 y button for setting gripper holder forward
            if (gamepad2.y) {
                gripper.setGripperHolderPerpendicular();
            }
            // gamepad2 right bumper to turn the Gripper Holder to the side
            if (gamepad2.right_bumper) {
                gripper.setGripperHolderParallel();
            }


            // angler control using gamepad2 dpad left and right (Hat)
            // dpad_left to for the Gripper system to face forward
            if (gamepad2.dpad_left) {
                gripper.setAnglerForward();
            }
            // dpad_right to set the Gripper system to the Side position
            if (gamepad2.dpad_right) {
                gripper.setAnglerSide();
            }


        }
    }

}