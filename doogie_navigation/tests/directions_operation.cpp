#include <gtest/gtest.h>

#include "doogie_navigation/maze_pose.hpp"

using DoogieOrientation = doogie_navigation::MazePose::GlobalOrientation;
using DoogieMoveGoal = doogie_navigation::MazePose::Direction;
using doogie_navigation::MazePose;

TEST(DirectionsOperations, north) {
  EXPECT_EQ(DoogieOrientation::NORTH, MazePose::transformOrientation(DoogieOrientation::NORTH, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::SOUTH, MazePose::transformOrientation(DoogieOrientation::NORTH, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::EAST, MazePose::transformOrientation(DoogieOrientation::NORTH, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::WEST, MazePose::transformOrientation(DoogieOrientation::NORTH, DoogieMoveGoal::LEFT));  
}

TEST(DirectionsOperations, south) {
  EXPECT_EQ(DoogieOrientation::SOUTH, MazePose::transformOrientation(DoogieOrientation::SOUTH, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::NORTH, MazePose::transformOrientation(DoogieOrientation::SOUTH, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::WEST, MazePose::transformOrientation(DoogieOrientation::SOUTH, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::EAST, MazePose::transformOrientation(DoogieOrientation::SOUTH, DoogieMoveGoal::LEFT));  
}

TEST(DirectionsOperations, east) {
  EXPECT_EQ(DoogieOrientation::EAST, MazePose::transformOrientation(DoogieOrientation::EAST, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::WEST, MazePose::transformOrientation(DoogieOrientation::EAST, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::SOUTH, MazePose::transformOrientation(DoogieOrientation::EAST, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::NORTH, MazePose::transformOrientation(DoogieOrientation::EAST, DoogieMoveGoal::LEFT));  
}

TEST(DirectionsOperations, west) {
  EXPECT_EQ(DoogieOrientation::WEST, MazePose::transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::EAST, MazePose::transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::NORTH, MazePose::transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::SOUTH, MazePose::transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::LEFT));  
}

int main (int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  
  static_assert(DoogieOrientation::NORTH + DoogieMoveGoal::FRONT == DoogieOrientation::NORTH, "NORTH + FRONT fails");
  static_assert(DoogieOrientation::NORTH + DoogieMoveGoal::BACK  == DoogieOrientation::SOUTH, "NORTH + BACK fails");
  static_assert(DoogieOrientation::NORTH + DoogieMoveGoal::RIGHT == DoogieOrientation::EAST,  "NORTH + RIGHT fails");
  
  static_assert(MazePose::transformOrientation(DoogieOrientation::NORTH, DoogieMoveGoal::LEFT)  == DoogieOrientation::WEST,  "NORTH + LEFT fails");
  static_assert(MazePose::transformOrientation(DoogieOrientation::SOUTH, DoogieMoveGoal::BACK)  == DoogieOrientation::NORTH,  "SOUTH + BACK fails");
  static_assert(MazePose::transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::BACK)  == DoogieOrientation::EAST,  "WEST + BACK fails");
  static_assert(MazePose::transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::RIGHT) == DoogieOrientation::NORTH, "WEST + RIGHT fails");

  return RUN_ALL_TESTS();
}
