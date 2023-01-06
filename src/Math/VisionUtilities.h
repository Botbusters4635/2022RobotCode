//
// Created by abiel on 2/1/22.
//

#ifndef BOTBUSTERS_REBIRTH_VISIONUTILITIES_H
#define BOTBUSTERS_REBIRTH_VISIONUTILITIES_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <optional>
#include <frc/geometry/Translation2d.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Core/VisionManager/VisionSource.h"
//#include "Core/EctoModule/Module.h"


class VisionUtilities {
public:
    struct Circle {
        std::pair<double,double> midpoint;
        double radius;
    };

    static std::optional<Circle> solveLeastSquaresCircle(const std::vector<std::pair<double,double>> &points);

    static std::pair<double,double> calculateTargetCenter(const std::vector<std::pair<double,double>> &corners);
    //Returns: topLeft, topRight, bottomLeft, bottomRight
    static std::vector<std::pair<double,double>> sortRectanglePoints(const std::vector<std::pair<double,double>> &points);

    /**
     *
     * @param corner
     * @param cameraResolution
     * @param vpw 59.6 for limelight
     * @param vph 45.7 for limelight
     * @param cameraPitch
     * @param goalHeight
     * @return
     */
    static std::optional<frc::Translation2d> cameraToTargetTranslation(std::pair<double, double> corner,
                                                                       const std::pair<double, double> &cameraResolution,
                                                                       double vpw,
                                                                       double vph,
                                                                       double cameraHeight,
                                                                       double cameraPitch,
                                                                       double goalHeight);

    static void publishCircle(nt::NetworkTableEntry entry, nt::NetworkTableEntry midpointEntry, const Circle &circle);

    static void publishPoints(nt::NetworkTableEntry entry, const std::vector<std::pair<double,double>> &points);

    static void publishPoint(nt::NetworkTableEntry entry, const std::pair<double,double> &points);

    static frc::Translation2d calculateFieldCentricFromTag(const frc::Transform3d &tagPose, frc::Transform2d &cameraPose, const frc::Pose2d &tagPoseToField);

    static bool filterTagPose(const AprilTag &tag, const std::shared_ptr<EctoSwerve> &swerve);

    static void publishVector(nt::NetworkTableEntry entry, const std::vector<int> &vector);

    template <typename Vector1, typename Vector2>
    static auto Dot(const Vector1& a, const Vector2& b) -> decltype(auto) {
        // (a_x i + a_y j) . (b_x i + b_y j)
        // = a_x b_x + a_y b_y
        return a.X() * b.X() + a.Y() * b.Y();
    }

private:
    static bool solveLeastSquaresCircle(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &points, Eigen::Vector2d &midpoint, double &radius);


};

#endif //BOTBUSTERS_REBIRTH_VISIONUTILITIES_H
