//
// Created by abiel on 2/1/22.
//

#include <random>
#include "gtest/gtest.h"
#include "Math/VisionUtilities.h"
#include <fmt/format.h>
#include <iostream>

TEST(CircleFitting, ZeroOriginCircle){
    std::vector<std::pair<double,double>> circlePoints;
    double angle = 0;
    while(angle <= M_PI){
        circlePoints.emplace_back(std::cos(angle), std::sin(angle));
        angle += 0.25;
    }

    //sin and cos should generate circle of r=1, midpoint 0,0
    auto circle = VisionUtilities::solveLeastSquaresCircle(circlePoints);
    ASSERT_TRUE(circle.has_value());

    ASSERT_NEAR(circle.value().radius, 1.0, 0.001);
    ASSERT_NEAR(circle.value().midpoint.first, 0, 0.001);
    ASSERT_NEAR(circle.value().midpoint.second, 0, 0.001);
}

TEST(CircleFitting, NoisyCircleTest){
    std::random_device rd;
    std::mt19937  gen(rd());
    std::uniform_real_distribution<> dis(0.9, 1.1);

    std::vector<std::pair<double,double>> circlePoints;
    double angle = 0;
    double x0 = 2.0, y0 = -5;
    while(angle <= M_PI){
        double x = std::cos(angle) * dis(gen) + x0;
        double y = std::sin(angle) * dis(gen) + y0;
        circlePoints.emplace_back(x,y);
        angle += 0.05;
    }

    auto circle = VisionUtilities::solveLeastSquaresCircle(circlePoints);
    ASSERT_TRUE(circle.has_value());

    EXPECT_NEAR(circle.value().radius, 1.0, 0.4);
    EXPECT_NEAR(circle.value().midpoint.first, 2, 0.4);
    EXPECT_NEAR(circle.value().midpoint.second, -5, 0.4);
}

TEST(CornerSorting, RectangleCornerSort){
    std::vector<std::pair<double,double>> unsortedPoints = {
            {1, 1},
            {1, -1},
            {-1, -1},
            {-1, 1}
    };

    auto sorted = VisionUtilities::sortRectanglePoints(unsortedPoints);

    ASSERT_EQ(sorted.size(), unsortedPoints.size());
    EXPECT_TRUE(sorted[0] == std::pair(-1.0, -1.0)); //Top left
    EXPECT_TRUE(sorted[1] == std::pair(1.0,-1.0)); //Top right
    EXPECT_TRUE(sorted[2] == std::pair(1.0, 1.0)); //Bottom left
    EXPECT_TRUE(sorted[3] == std::pair(-1.0,1.0)); //Bottom right
}