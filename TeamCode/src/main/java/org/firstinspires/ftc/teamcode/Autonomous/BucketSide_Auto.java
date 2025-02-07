package org.firstinspires.ftc.teamcode.Autonomous;

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
        //Gripper holder perpendicular to the bar
        gripper.setGripperHolderParallel();
        gripper.setAnglerInit();

        // Initialize telemetry
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Define starting position
        Pose2d startPos = new Pose2d(8, 87, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        Pose2d SpecimenDropoffPos = new Pose2d(32, 79, Math.toRadians(0));
        Pose2d SampleDropoffPos1 = new Pose2d(20, 115, Math.toRadians(40));
        Pose2d SampleDropoffPos2 = new Pose2d(16.2, 126, Math.toRadians(150));
        Pose2d AssentPos = new Pose2d(64, 106, -Math.toRadians(90));
        Pose2d SamplePickUpPos1 = new Pose2d(35.5, 122, Math.toRadians(0));
        Pose2d SamplePickUpPos2 = new Pose2d(34, 117, Math.toRadians(0));

        // Define the trajectory sequence
        TrajectorySequence BucketTrajectory = drive.trajectorySequenceBuilder(startPos)

                // Step 1: Set the gripper and arm in the right position for Specimen drop off
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(71);})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerSample();})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{gripper.setGripperHolderPerpendicular();})

                // Step 2: Approach the Specimen drop off position and move forward to approach
                // the top bar
                .lineToLinearHeading(SpecimenDropoffPos)
                .waitSeconds(0.15)
                .forward(5.5)

                // Step 3: Set Arm downward to prepare for placing the Specimen
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(40);})
                .waitSeconds(0.1)

                // Step 4: Move backward and open the Gripper to place and release the Specimen.
                // At the same time, drop the arm all the way down and set its power to zero
                // afterwards
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(-20))
                //.waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> armControl.setArmPower(0)) //???
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> gripper.setAnglerSpecimen())

                // Step 5: Move to Sample 1 and extend the slide to pick up the Sample 1
                .lineToLinearHeading(SamplePickUpPos1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setAnglerSample())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setGripperClosed())
                .waitSeconds(0.2)
                .back(9)

                // Step 6: Set Arm to Deposit angle and extend slides for deposit
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setArmDeposit())

                //.UNSTABLE_addTemporalMarkerOffset(0.2, () -> sliderControl.setSliderDeposit())
                //.waitSeconds(0.1)

                // Step 7: Go to the bucket to drop off Sample 1
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> sliderControl.setSliderDeposit())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> gripper.setAnglerSpecimen())
                .waitSeconds(0.1)

                // Go to the target position then final position: split into two section to ensure
                // the arm and slider and gripper are all in position before turn

                .lineToLinearHeading(SampleDropoffPos1)
                .lineToLinearHeading(SampleDropoffPos2)
                //.forward(3)

                // Lower the Arm angle and turn the Gripper perpendicular once above the bucket
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(82))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> gripper.setAnglerSample())
                // Open the gripper to drop the sample and raise the Arm, then close the gripper
                // and retract the slide
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setGripperOpen())
                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripper.setGripperHolderParallel())
                .UNSTABLE_addTemporalMarkerOffset(0.65, () -> gripper.setAnglerSpecimen()) //???
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> armControl.setArmDeposit())
                .waitSeconds(0.7)
                //.back(5)

                //Increase the back travel to avoid hitting the side wall
                .back(10)
                // Retract the two stage slides and lower the arm and set the gripper holder position
                // to prepare for sample 2 pick up
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {sliderControl.setDesSliderLen(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {armControl.setDesArmPosDeg(-20);})
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setGripperHolderPerpendicular())
                .waitSeconds(0.2)

                // Potential Sample 2 pick up code should be placed here
                // Step 5: Move to Sample 2 and extend the slide to pick up the Sample 1
                .lineToLinearHeading(SamplePickUpPos2)
                .strafeLeft(14)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setAnglerSample())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setGripperClosed())
                .waitSeconds(0.3)
                .strafeRight(11)


                // Step 6: Set Arm to Deposit angle and extend slides for deposit
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setArmDeposit())

                //.UNSTABLE_addTemporalMarkerOffset(0.2, () -> sliderControl.setSliderDeposit())
                //.waitSeconds(0.1)

                // Step 7: Go to the bucket to drop off Sample 1
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> sliderControl.setSliderDeposit())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> gripper.setAnglerSpecimen())
                .waitSeconds(0.2)

                // Go to the target position then final position: split into two section to ensure
                // the arm and slider and gripper are all in position before turn

                .lineToLinearHeading(SampleDropoffPos1)
                .lineToLinearHeading(SampleDropoffPos2)
                //.forward(3)
                // Lower the Arm angle and turn the Gripper perpendicular once above the bucket
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(82))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> gripper.setAnglerSample())
                // Open the gripper to drop the sample and raise the Arm, then close the gripper
                // and retract the slide
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> gripper.setGripperOpen())
                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripper.setGripperHolderParallel())
                .UNSTABLE_addTemporalMarkerOffset(0.65, () -> gripper.setAnglerSpecimen()) //???
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> armControl.setArmDeposit())
                .waitSeconds(0.7)

                .back(25)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {sliderControl.setDesSliderLen(11);})

                // Go to the sample 1 pick up position
                //.lineToLinearHeading(SamplePickUpPos1)

                // Go to the position  for level 1 ascent
                .lineToLinearHeading(AssentPos)
                //.turn(-Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(20);})
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {armControl.ArmRunModReset();})
                .waitSeconds(0.3)


                //final build
                .build();

        // Wait for start signal
        waitForStart();

        // Execute the trajectory sequence
        drive.followTrajectorySequence(BucketTrajectory);

    }
}