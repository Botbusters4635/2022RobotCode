//
// Created by abiel on 2/1/22.
//

#include "VisionUtilities.h"
#include <Eigen/SVD>
#include <algorithm>
#include <numeric>
#include <frc/geometry/Rotation2d.h>
#include <iostream>

//https://github.com/DLuensch/Least-Squares-Circle-Fitting-Kasa-Method-/blob/master/src/circleFitting.cpp
bool VisionUtilities::solveLeastSquaresCircle(
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &points,
        Eigen::Vector2d &midpoint, double &radius) {
    size_t length = points.size();
    double x1;
    double x2;
    double x3;
    Eigen::MatrixXd AFill(3, length);
    Eigen::MatrixXd A(length, 3);
    Eigen::VectorXd AFirst(length);
    Eigen::VectorXd ASec(length);
    Eigen::VectorXd AFirstSquared(length);
    Eigen::VectorXd ASecSquared(length);
    Eigen::VectorXd ASquaredRes(length);
    Eigen::VectorXd b(length);
    Eigen::VectorXd c(3);
    bool ok = true;

    if (length > 1) {
        for (size_t i = 0; i < length; i++) {
            AFill(0, i) = points[i](0);
            AFill(1, i) = points[i](1);
            AFill(2, i) = 1;
        }

        A = AFill.transpose();

        for (size_t i = 0; i < length; i++) {
            AFirst(i) = A(i, 0);
            ASec(i) = A(i, 1);
        }

        for (size_t i = 0; i < length; i++) {
            AFirstSquared(i) = AFirst(i) * AFirst(i);
            ASecSquared(i) = ASec(i) * ASec(i);
        }

        b = AFirstSquared + ASecSquared;

        c = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

        x1 = c(0);
        midpoint(0) = x1 * 0.5;
        x2 = c(1);
        midpoint(1) = x2 * 0.5;
        x3 = c(2);
        radius = sqrt((x1 * x1 + x2 * x2) / 4 + x3);
    } else {
        ok = false;
    }

    return ok;
}

std::optional<VisionUtilities::Circle>
VisionUtilities::solveLeastSquaresCircle(const std::vector<std::pair<double, double>> &points) {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> eigenPoints;
    Eigen::Vector2d midpoint;
    VisionUtilities::Circle circle;

    std::transform(points.begin(), points.end(), std::back_inserter(eigenPoints),
                   [](const std::pair<double, double> &point) {
                       return Eigen::Vector2d(point.first, point.second);
                   });

    bool ok = solveLeastSquaresCircle(eigenPoints, midpoint, circle.radius);
    if (!ok) return std::nullopt;
    circle.midpoint = std::pair(midpoint.x(), midpoint.y());

    return circle;
}

std::pair<double, double>
VisionUtilities::calculateTargetCenter(const std::vector<std::pair<double, double>> &corners) {
    //Get sum of all points
    auto targetCenter = std::accumulate(corners.begin(), corners.end(),
                                        std::pair(0.0, 0.0), [](const auto &p1, const auto &p2) {
                return std::pair(p1.first + p2.first, p1.second + p2.second);
            });
    //Average center
    targetCenter.first /= corners.size();
    targetCenter.second /= corners.size();

    return targetCenter;
}
std::vector<std::pair<double, double>>
VisionUtilities::sortRectanglePoints(const std::vector<std::pair<double, double>> &points) {
    if(points.size() != 4) return {};
    std::vector<std::pair<double,double>> sortedPoints;
    auto center = calculateTargetCenter(points);
    std::vector<std::pair<double, std::pair<double,double>>> positivePolarCoords, negativePolarCoords; //(angle, (x,y))
    //Gets polar coordinates of all points, using the center as the origin
    std::for_each(points.begin(), points.end(),
                   [&](const std::pair<double, double> &point) {
                        auto rot = frc::Rotation2d(point.first - center.first, point.second - center.second) - frc::Rotation2d(units::degree_t(90));
                        auto angle = rot.Radians().value();
                        if(angle > 0) positivePolarCoords.emplace_back(angle, point);
                        else negativePolarCoords.emplace_back(angle, point);
                   });

    //Top left should be max positive value, top right should be min positive value
    auto [bottomRight, topLeft] = std::minmax_element(positivePolarCoords.begin(), positivePolarCoords.end(), [](const auto &a, const auto &b){
       return a.first < b.first;
    });

    //Bottom left should be min negative value, bottom right should be max negative value
    auto [topRight, bottomLeft] = std::minmax_element(negativePolarCoords.begin(), negativePolarCoords.end(), [](const auto &a, const auto &b){
        return a.first < b.first;
    });

    return {topLeft->second, topRight->second, bottomLeft->second, bottomRight->second};
}

std::optional<frc::Translation2d> VisionUtilities::cameraToTargetTranslation(std::pair<double, double> corner,
                                                              const std::pair<double, double> &cameraResolution,
                                                              double vpw,
                                                              double vph,
                                                              double cameraHeight,
                                                              double cameraPitch,
                                                              double goalHeight) {
    double yPixels = corner.first;
    double zPixels = corner.second;

    // Robot frame of reference
    double nY = -((yPixels - (cameraResolution.first / 2)) / (cameraResolution.first / 2));
    double nZ = -((zPixels - (cameraResolution.second / 2)) / (cameraResolution.second / 2));

    frc::Translation2d xzPlaneTranslation =
        frc::Translation2d(1.0_m, units::meter_t(vph / 2.0 * nZ)).RotateBy(units::degree_t(cameraPitch));
    double x = xzPlaneTranslation.X().value();
    double y = vpw / 2.0 * nY;
    double z = xzPlaneTranslation.Y().value();

    double differentialHeight = cameraHeight - goalHeight;
    if ((z < 0.0) == (differentialHeight > 0.0)) {
      double scaling = differentialHeight / -z;
      double distance = std::hypot(x, y) * scaling;
      frc::Rotation2d angle = frc::Rotation2d(x, y);
      return frc::Translation2d(units::meter_t(distance * angle.Cos()),
          units::meter_t(distance * angle.Sin()));
    }
    return {};    
}

void VisionUtilities::publishCircle(nt::NetworkTableEntry entry, nt::NetworkTableEntry midpointEntry, const Circle &circle) {
    entry.SetString(fmt::format("C({},{})R{}C", circle.midpoint.first, circle.midpoint.second, circle.radius));
    VisionUtilities::publishPoint(midpointEntry, circle.midpoint);
}

void VisionUtilities::publishPoints(nt::NetworkTableEntry entry, const std::vector<std::pair<double,double>> &points){
    std::vector<std::string> strings;
    std::transform(points.begin(), points.end(), std::back_inserter(strings), [](const auto &point){
       return fmt::format("({}[{})", point.first, point.second);
    });

    entry.SetStringArray(strings);
}

void VisionUtilities::publishPoint(nt::NetworkTableEntry entry, const std::pair<double, double> &points) {
    //entry.SetDoubleArray({points.first, points.second});
    entry.SetString(fmt::format("({}[{})", points.first, points.second));
}

frc::Translation2d VisionUtilities::calculateFieldCentricFromTag(const frc::Transform3d &tagPose,
                                                                 frc::Transform2d &cameraPose,
                                                                 const frc::Pose2d &tagPoseToField) {

    frc::Translation2d cameraCentric = tagPose.Translation().ToTranslation2d().RotateBy(cameraPose.Rotation());
    frc::Translation2d robotCentric = frc::Translation2d(cameraCentric.X() + cameraPose.X(), cameraCentric.Y() + cameraPose.Y());
    frc::Translation2d fieldCentric = {tagPoseToField.X() - robotCentric.X(),
                                       tagPoseToField.Y() - robotCentric.Y()};

    return fieldCentric;
}

void VisionUtilities::publishVector(nt::NetworkTableEntry entry, const std::vector<int> &vector) {
    std::string message;
    for (auto &item : vector){
        message.append(fmt::format("{}, ", item));
    }
    entry.SetString(message);
}

bool VisionUtilities::filterTagPose(const AprilTag &tag, const std::shared_ptr<EctoSwerve> &swerve) {
    auto tagPose = tag.tagPose;
    auto distToPose = tagPose.Translation().ToTranslation2d().Distance(swerve->getPose().Translation());
    if(distToPose > 10_m) {
        std::cout << fmt::format("Pose filtered with distance of: {}", distToPose.value()) << std::endl;
        return false;
    }

    auto min = units::math::min(tagPose.X(), tagPose.Y());
    if(min < 0_m){
        std::cout << fmt::format("Pose with negative coordinate filtered: {}, {}, id: {}", tagPose.X().value(), tagPose.Y().value(), tag.id) << std::endl;
        return false;
    }

    auto max = units::math::max(tagPose.X(), tagPose.Y());
    if(max > 10_m){
        std::cout << fmt::format("Pose with coordinates more than 10m filtered: {}, {}, id: {}", tagPose.X().value(), tagPose.Y().value(), tag.id) << std::endl;
        return false;
    }

    auto angularSpeed = swerve->getChassisSpeeds().omega;
    if (angularSpeed > 50_deg_per_s){
        std::cout << fmt::format("rejected pose due to angular speed: {}", angularSpeed.value()) << std::endl;
        return{};
    }
    return true;
}