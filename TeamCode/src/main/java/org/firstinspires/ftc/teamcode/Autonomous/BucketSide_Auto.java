package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.ArmControl;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BucketSide_Auto extends LinearOpMode {

    private ElapsedTime AutoTimer = new ElapsedTime();
    private ArmControl armControl;
    private Gripper gripper;
    private SliderControl sliderControl;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // - - - Setting up Arm motors - - - //
        armControl = new ArmControl(this);
        armControl.init(hardwareMap);
        // Set the Hang servo up to put the blocking plate in place to hold the arm up
        armControl.setHangServoUp();
        //armControl.setDesArmPosDeg(-5);

        // - - - Setting up Slider motors - - - //
        sliderControl = new SliderControl(this);
        sliderControl.init(hardwareMap);

        // - - - Initialize gripper to starting position - - - //
        gripper = new Gripper(this);
        gripper.init(hardwareMap);
        //Gripper closed state
        gripper.setGripperClosed();
        //Gripper holder to the side
        gripper.setGripperHolderParallel();
        gripper.setAnglerSide();

        // Initialize telemetry
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Define starting position
        Pose2d startPos = new Pose2d(8, 87, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        Pose2d SpecimenDropoffPos = new Pose2d(35, 79, Math.toRadians(0));
        Pose2d SampleDropoffPos1 = new Pose2d(20, 123, Math.toRadians(135));
        Pose2d PushPos1 = new Pose2d(56, 100, Math.toRadians(0));
        Pose2d SamplePickUpPos1 = new Pose2d(29.3, 120, Math.toRadians(0));

        // Define the trajectory sequence
        TrajectorySequence StageRedBucket = drive.trajectorySequenceBuilder(startPos)

                // Step 1: Set the gripper and arm in the right position for Specimen drop off
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(74);})
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{gripper.setAnglerForward();})
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{gripper.setGripperHolderPerpendicular();})
                .waitSeconds(0.5)
                // Step 2: Approach the Specimen drop off position and move forward to approach
                // the top bar
                .lineToLinearHeading(SpecimenDropoffPos)
                .forward(3.8)

                // Step 3: Set Arm downward to prepare for placing the Specimen
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(40);})
                .waitSeconds(0.2)

                // Step 4: Move backward and open the Gripper to place and release the Specimen.
                // At the same time, drop the arm all the way down and set its power to zero
                // afterwards
                .back(7.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(-20))
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setArmPower(0))

                // Step 5: Move to Sample 1 and extend the slide to pick up the Sample 1
                .lineToLinearHeading(SamplePickUpPos1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(5.5))
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> gripper.setGripperClosed())
                .waitSeconds(1.2)

                // Step 6: Retract the slide and set Arm to Deposit angle
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(0))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> armControl.setArmDeposit())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> sliderControl.setSliderDeposit())
                .waitSeconds(1)

                // Step 7: Turn 135 degrees to face the bucket and go to the bucket to
                // drop off Sample 1
                .turn(Math.toRadians(135))
                .waitSeconds(1)
                .lineToLinearHeading(SampleDropoffPos1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripper.setGripperHolderParallel())
                .waitSeconds(0.8)
                .forward(5)
                // Lower the Arm angle and turn the Gripper perpendicular once above the bucket
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(82))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setGripperHolderPerpendicular())
                // Open the gripper to drop the sample and raise the Arm, then close the gripper
                // and retract the slide
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> gripper.setGripperOpen())
                //.UNSTABLE_addTemporalMarkerOffset(0.8, () -> armControl.setArmDeposit())
                //.UNSTABLE_addTemporalMarkerOffset(1.0, () -> gripper.setGripperClosed())
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> gripper.setGripperHolderParallel())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> gripper.setGripperClosed())
                .waitSeconds(1)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {sliderControl.setDesSliderLen(7);})
                .waitSeconds(0.8)

                // Step 8: Go Back to the position PushPos1 for level 1 ascent
                .lineToLinearHeading(PushPos1)
                .turn(-Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(25);})
                //.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {sliderControl.setDesSliderLen(8);})
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setArmPower(-0.4);})
                .waitSeconds(2)

                //final build
                .build();

        // Wait for start signal
        waitForStart();

        // Execute the trajectory sequence
        drive.followTrajectorySequence(StageRedBucket);

    }
}