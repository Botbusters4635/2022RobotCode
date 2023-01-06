//
// Created by andrew on 08/11/202
//

#include "gtest/gtest.h"
#include "Math/InterpolatingTable/InterpolatingTable.h"

TEST(InterpolatingTable, GettingTableValues) {
    InterpolatingTable table;
    for (double i = 0; i < 6; i++) {
        table.addPoint(i, 2.0 * i);
    }

    for(int i = 0; i < 5; i++){
        EXPECT_EQ(i * 2.0, i * 2.0);
    }
}


TEST(InterpolatingTable, InterpolatingValues) {
    InterpolatingTable table;
    for (int i = 0; i < 6; i++) {
        table.addPoint(i, 2.0 * i);
    }

    for(double i = 1.5; i <= 5; i++){
        EXPECT_EQ(table.get(i), i*2.0);
    }
}

TEST(InterpolatingTable, InterpolatingNonLinearValues){

    InterpolatingTable table;

    table.addPoint(0.0, 0.0);
    table.addPoint(11.0, 16.0);
    table.addPoint(1.0, 4.0);
    table.addPoint(5.0, 11.0);
    table.addPoint(4.0, 7.0);

    EXPECT_NEAR(table.get(1.1), 4.1, 0.01);
    EXPECT_NEAR(table.get(3.1), 6.1, 0.01);
    EXPECT_NEAR(table.get(4.3), 8.2, 0.01);
    EXPECT_NEAR(table.get(7.5), 13.083, 0.01);
}

TEST(InterpolatingTable, MinMaxValues){
    InterpolatingTable table;
    for (int i = 0; i < 6; i++) {
        table.addPoint(i, 2.0 * i);
    }

    EXPECT_NEAR(table.get(-1), 0, 0.001);
    EXPECT_NEAR(table.get(7), 10, 0.001);
}