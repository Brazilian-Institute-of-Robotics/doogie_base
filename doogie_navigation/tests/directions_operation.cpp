#include <gtest/gtest.h>

#include "doogie_navigation/maze_pose.hpp"

using DoogieOrientation = doogie::GlobalOrientation;
using DoogieMoveGoal = doogie::Direction;
using doogie_navigation::MazePose;

constexpr DoogieOrientation transformOrientation(DoogieOrientation orientation, DoogieMoveGoal direction) {
  auto new_orientation = orientation + direction;
  if (new_orientation > 4) {
    new_orientation -= 4;
  }
  if (new_orientation < 1) {
    new_orientation += 4;
  }
  return DoogieOrientation(new_orientation);
}

TEST(DirectionsOperations, north) {
  EXPECT_EQ(DoogieOrientation::NORTH, transformOrientation(DoogieOrientation::NORTH, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::SOUTH, transformOrientation(DoogieOrientation::NORTH, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::EAST, transformOrientation(DoogieOrientation::NORTH, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::WEST, transformOrientation(DoogieOrientation::NORTH, DoogieMoveGoal::LEFT));  
}

TEST(DirectionsOperations, south) {
  EXPECT_EQ(DoogieOrientation::SOUTH, transformOrientation(DoogieOrientation::SOUTH, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::NORTH, transformOrientation(DoogieOrientation::SOUTH, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::WEST, transformOrientation(DoogieOrientation::SOUTH, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::EAST, transformOrientation(DoogieOrientation::SOUTH, DoogieMoveGoal::LEFT));  
}

TEST(DirectionsOperations, east) {
  EXPECT_EQ(DoogieOrientation::EAST, transformOrientation(DoogieOrientation::EAST, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::WEST, transformOrientation(DoogieOrientation::EAST, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::SOUTH, transformOrientation(DoogieOrientation::EAST, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::NORTH, transformOrientation(DoogieOrientation::EAST, DoogieMoveGoal::LEFT));  
}

TEST(DirectionsOperations, west) {
  EXPECT_EQ(DoogieOrientation::WEST, transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::FRONT));

  EXPECT_EQ(DoogieOrientation::EAST, transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::BACK));

  EXPECT_EQ(DoogieOrientation::NORTH, transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::RIGHT));

  EXPECT_EQ(DoogieOrientation::SOUTH, transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::LEFT));  
}

int main (int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  
  // static_assert(DoogieOrientation::NORTH + DoogieMoveGoal::FRONT == DoogieOrientation::NORTH, "NORTH + FRONT fails");
  // static_assert(DoogieOrientation::NORTH + DoogieMoveGoal::BACK  == DoogieOrientation::SOUTH, "NORTH + BACK fails");
  // static_assert(DoogieOrientation::NORTH + DoogieMoveGoal::RIGHT == DoogieOrientation::EAST,  "NORTH + RIGHT fails");
  
  // static_assert(transformOrientation(DoogieOrientation::NORTH, DoogieMoveGoal::LEFT)  == DoogieOrientation::WEST,  "NORTH + LEFT fails");
  // static_assert(transformOrientation(DoogieOrientation::SOUTH, DoogieMoveGoal::BACK)  == DoogieOrientation::NORTH,  "SOUTH + BACK fails");
  // static_assert(transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::BACK)  == DoogieOrientation::EAST,  "WEST + BACK fails");
  // static_assert(transformOrientation(DoogieOrientation::WEST, DoogieMoveGoal::RIGHT) == DoogieOrientation::NORTH, "WEST + RIGHT fails");

  return RUN_ALL_TESTS();
}
