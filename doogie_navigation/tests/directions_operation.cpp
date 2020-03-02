#include <gtest/gtest.h>

#include "doogie_navigation/move_in_maze.hpp"

using doogie_msgs::DoogieOrientation;
using doogie_msgs::DoogieMoveGoal;

TEST(DirectionsOperations, north) {
  EXPECT_EQ(DoogieOrientation::NORTH, doogie::computeResultDirection(DoogieOrientation::NORTH, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::SOUTH, doogie::computeResultDirection(DoogieOrientation::NORTH, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::EAST, doogie::computeResultDirection(DoogieOrientation::NORTH, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::WEST, doogie::computeResultDirection(DoogieOrientation::NORTH, DoogieMoveGoal::LEFT));  
}

TEST(DirectionsOperations, south) {
  EXPECT_EQ(DoogieOrientation::SOUTH, doogie::computeResultDirection(DoogieOrientation::SOUTH, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::NORTH, doogie::computeResultDirection(DoogieOrientation::SOUTH, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::WEST, doogie::computeResultDirection(DoogieOrientation::SOUTH, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::EAST, doogie::computeResultDirection(DoogieOrientation::SOUTH, DoogieMoveGoal::LEFT));  
}

TEST(DirectionsOperations, east) {
  EXPECT_EQ(DoogieOrientation::EAST, doogie::computeResultDirection(DoogieOrientation::EAST, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::WEST, doogie::computeResultDirection(DoogieOrientation::EAST, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::SOUTH, doogie::computeResultDirection(DoogieOrientation::EAST, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::NORTH, doogie::computeResultDirection(DoogieOrientation::EAST, DoogieMoveGoal::LEFT));  
}

TEST(DirectionsOperations, west) {
  EXPECT_EQ(DoogieOrientation::WEST, doogie::computeResultDirection(DoogieOrientation::WEST, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::EAST, doogie::computeResultDirection(DoogieOrientation::WEST, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::NORTH, doogie::computeResultDirection(DoogieOrientation::WEST, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::SOUTH, doogie::computeResultDirection(DoogieOrientation::WEST, DoogieMoveGoal::LEFT));  
}

int main (int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  
  static_assert(DoogieOrientation::NORTH + DoogieMoveGoal::FRONT == DoogieOrientation::NORTH, "NORTH + FRONT fails");
  static_assert(DoogieOrientation::NORTH + DoogieMoveGoal::BACK  == DoogieOrientation::SOUTH, "NORTH + BACK fails");
  static_assert(DoogieOrientation::NORTH + DoogieMoveGoal::RIGHT == DoogieOrientation::EAST,  "NORTH + RIGHT fails");
  
/*   static_assert(DoogieOrientation::NORTH + DoogieMoveGoal::LEFT  == DoogieOrientation::WEST,  "NORTH + LEFT fails");
  static_assert(DoogieOrientation::SOUTH + DoogieMoveGoal::BACK  == DoogieOrientation::NORTH,  "SOUTH + BACK fails");
  static_assert(DoogieOrientation::WEST  + DoogieMoveGoal::BACK  == DoogieOrientation::EAST,  "WEST + BACK fails");
  static_assert(DoogieOrientation::WEST  + DoogieMoveGoal::RIGHT == DoogieOrientation::NORTH, "WEST + RIGHT fails"); */

  return RUN_ALL_TESTS();
}
