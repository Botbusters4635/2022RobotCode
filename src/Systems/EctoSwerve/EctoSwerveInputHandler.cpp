//
// Created by abiel on 1/2/20.
//

#include "EctoSwerveInputHandler.h"
bool EctoSwerveInputHandler::registeredJoysticks = false;

EctoSwerveInputHandler::EctoSwerveInputHandler(const std::shared_ptr<EctoSwerve> &swerveIn,
                                               VisionManager *visionManagerIn) {
    this->swerve = swerveIn;
    this->visionManager = visionManagerIn;

    AddRequirements({swerve.get()});
}

void EctoSwerveInputHandler::Initialize() {
    if(!registeredJoysticks){
        //movement on right joystick and turning in left.
        input.registerAxis(strafeAxis, "rightX");
        input.registerAxis(forwardAxis, "rightY");
        input.registerAxis(rotationAxis, "leftX");
        //lavandole el coco a pato para volterle los joysticks y que pueda usar mejor los botones.
//        input.registerAxis(strafeAxis, "leftX");
//        input.registerAxis(forwardAxis, "leftY");
//        input.registerAxis(rotationAxis, "rightX");
//
//        input.registerAxis(shootWhileMove, "rightTrigger");
//        input.registerButton(shootAndMoveButton, "rightBumper");
        //input.registerAxis(fieldOrientedAxis, "rightTrigger");
        input.registerButton(resetYaw, "select");
        input.registerButton(visionButton, "leftBumper");
//        input.registerButton(alignYaw, "leftBumper");
        input.registerButton(brake, "Y");
        registeredJoysticks = true;
    }


    table = ntInstance.GetTable("EctoSwerveInputHandler");
    visionTable = ntInstance.GetTable("photonvision/gloworm");
    aprilTagTable = ntInstance.GetTable("aprilTags");
    aprilTagYaw = aprilTagTable->GetEntry("tYaw");
    visionAngle = visionTable->GetEntry("targetYaw");
    hasTarget = aprilTagTable->GetEntry("validTarget");
    aprilTagX = aprilTagTable->GetEntry("targets");

//    visionAnglePID.EnableContinuousInput(units::radian_t(-M_PI), units::radian_t(M_PI));
//    visionAnglePID.Reset(0_rad);
//    visionAnglePID.SetTolerance(0.05_rad);

//    frc::SmartDashboard::PutData("VisionPIDController", &visionAnglePID);

    table->GetEntry("FlywheelVelLoss").SetDouble(5);
}

void EctoSwerveInputHandler::Execute() {
    Twist2D joystickInput(-forwardAxis.get(), -strafeAxis.get(), -rotationAxis.get());

    joystickInput.setDx(xFilter.Calculate(units::meters_per_second_t(joystickInput.getDx())).to<double>());
    joystickInput.setDy(yFilter.Calculate(units::meters_per_second_t(joystickInput.getDy())).to<double>());
    joystickInput.setDtheta(thetaFilter.Calculate(units::radians_per_second_t(joystickInput.getDtheta())).to<double>());

    if(brake.get()){
        double brakePercent = 0.5;
        joystickInput.setDx(joystickInput.getDx() * brakePercent);
        joystickInput.setDy(joystickInput.getDy() * brakePercent);
        joystickInput.setDtheta(joystickInput.getDtheta() * brakePercent);
    }

    table->GetEntry("ForwardAxis").SetDouble(forwardAxis.get());
    table->GetEntry("StrafeAxis").SetDouble(strafeAxis.get());
    table->GetEntry("RotationAxis").SetDouble(rotationAxis.get());

    Twist2D output = joystickInput;
    output.setDtheta(output.getDtheta() * 0.7);

    output.setDtheta(swerve->calculateRotationFF(units::radians_per_second_t(output.getDtheta() * 2.0 * M_PI)).value() / 12.0);

    if (resetYaw.get()) {
        swerve->zeroYaw();
    }

    Point2D centerOfRotation(0,0);

    /**
     * Shoot while move test
     */
//vision Align
//    if (visionButton.get() && hasTarget.GetBoolean(false)){
//        frc::SmartDashboard::PutString("aprilTags","running tag align");
//        double setpoint = 0;
//        double state = aprilTagYaw.GetDoubleArray({0})[0];
//        double visionOut = -yawPID.Calculate(state, setpoint);
//        output.setDtheta(visionOut);
//    }

//    if (hasTarget.GetBoolean(false)){
//        newSetPoint = aprilTagX.GetDoubleArray({0})[0];
//    }
//
//    if (visionButton.get()){
//        double state = swerve->getYaw();
//        state = EctoMath::radiansToDegrees(state) - -90;
//        double out = yawPID.Calculate(state, newSetPoint);
//        output.setDtheta(out);
//    }

//    if (visionButton.get() && hasTarget.GetBoolean(false)){
//        enableRobotOriented = true;
//        frc::SmartDashboard::PutString("aprilTags","running tag align");
//        double setpoint = 0;
//        double state = aprilTagX.GetDoubleArray({0})[0];
//        double visionOut = xPID.Calculate(state, setpoint);
//        output.setDx(visionOut);
//    }else{
//        enableRobotOriented = false;
//    }

    if(shootAndMoveButton.get()){
        output.setDx(output.getDx() * 0.31);
        output.setDy(output.getDy() * 0.31);

    }

    lastVisionAlign = visionButton.get();

    frc::ChassisSpeeds chassisSpeeds;
    if (enableRobotOriented) {
        chassisSpeeds.vx = units::meters_per_second_t(output.getDx());
        chassisSpeeds.vy = units::meters_per_second_t(output.getDy());
        chassisSpeeds.omega = units::radians_per_second_t(output.getDtheta());
    } else {
        chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(output.getDx()),
                                                                    units::meters_per_second_t(output.getDy()),
                                                                    units::radians_per_second_t(
                                                                            output.getDtheta()),
                                                                    frc::Rotation2d(
                                                                            units::radian_t(swerve->getYaw())));
    }

    swerve->setPercent(chassisSpeeds, centerOfRotation);
}


void EctoSwerveInputHandler::End(bool interrupted) {
    ;
}

bool EctoSwerveInputHandler::IsFinished() {
    return false;
}

void EctoSwerveInputHandler::setVisionYawOffset(double offset) {
    this->visionOffset = offset;
}