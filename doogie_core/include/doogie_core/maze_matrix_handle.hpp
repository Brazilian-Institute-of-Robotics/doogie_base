#ifndef DOOGIE_CORE_MAZE_MATRIX_HANDLE_HPP_
#define DOOGIE_CORE_MAZE_MATRIX_HANDLE_HPP_

#include <ros/ros.h>
#include "doogie_msgs/MazeCell.h"
#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePosition.h"

#include <string>
#include <cstddef>

namespace doogie_core {

typedef doogie_msgs::MazeCellMultiArray MazeMatrix; 

/**
 * @brief Struct to represent the cells around the robot locally.
 * 
 */
typedef struct LocalCell{
  bool front_wall;
  bool back_wall;
  bool right_wall;
  bool left_wall;
} LocalCell;

/**
 * @brief Class to group methods to handle the maze obstacles matrix.
 * 
 * This class is intended to be used by the perception functionality and the
 * maze solver algorithms. For this, methods were developed to to deal with
 * the data of the maze obstacle matrix. This class can be instantiated as
 * well as used with its static methods. When used as an object, the data 
 * handling is performed in the internal matrix. To use the static methods,
 * a previously instantied matrix must be passed as an argument.
 * 
 * To work with the maze obstacle matrix, take in mind there are two 
 * references: local and global. The local reference is based on the robot
 * and the cell walls can be in the front, back, left and right of the robot.
 * The global reference is based in the cardinal directions and the cell
 * walls can be in the north, south, east and weast.
 */
class MazeMatrixHandle {
 public:
  /**
   * @brief Construct a new MazeMatrixHandle object.
   * 
   * When used this constructor, the method initMatrix must be called.
   * 
   * @see initMatrix
   * 
   */
  MazeMatrixHandle();
  
  /**
   * @brief Construct a new MazeMatrixHandle object.
   * 
   * @param row_qty Number of rows in the maze matrix.
   * @param column_qty Number of columns in the maze matrix.
   */
  MazeMatrixHandle(uint8_t row_qty, uint8_t column_qty);
  
  /**
   * @brief Initialize the internal maze matrix.
   * 
   * @param row_qty Number of rows in the maze matrix. 
   * @param column_qty Number of columns in the maze matrix. 
   */
  void initMatrix(uint8_t row_qty, uint8_t column_qty);

  /**
   * @brief Get a maze matrix cell holding walls information in the global reference.
   * 
   * @param maze_matrix The maze obstacle matrix.
   * @param row Row of the cell.
   * @param column Columns of the cell.
   * @return doogie_msgs::MazeCell Object holding walls data of the cell (row, column).
   */
  static doogie_msgs::MazeCell getMazeMatrixCell(MazeMatrix maze_matrix, uint8_t row, uint8_t column);

  /**
   * @brief Get a maze matrix cell holding walls information in the global reference.
   * 
   * @param row Row of the cell.
   * @param column Columns of the cell.\
   * @return doogie_msgs::MazeCell Object holding walls data of the cell (row, column).
   */
  doogie_msgs::MazeCell getMazeMatrixCell(uint8_t row, uint8_t column);

  /**
   * @brief Get the internal maze matrix.
   * 
   * @return MazeMatrix Internal maze matrix holding all information about maze walls.
   */
  MazeMatrix getMazeMatrix();

  /**
   * @brief Set cell walls information of the maze matrix cell in the global reference. 
   * 
   * @param row Row of the cell.
   * @param column Columns of the cell.
   * @param maze_cell_walls Object holding the walls information in the global reference.
   */
  void setMazeMatrixCell(uint8_t row, uint8_t column, doogie_msgs::MazeCell maze_cell_walls);

  /**
   * @brief Set cell walls information of the maze matrix in the global reference.
   * 
   * @param doogie_position Position of the robot in the maze matrix.
   * @param local_cell_walls Struct with walls information referenced in respect to the robot.
   * @param visited Flag to inform if the cell was visited.
   */
  void setMazeMatrixCell(doogie_msgs::DoogiePosition doogie_position, LocalCell local_cell_walls, bool visited = false);

  /**
   * @brief Set cell walls information of the maze matrix in the global reference.
   * 
   * @param row Row of the cell.
   * @param column Columns of the cell.
   * @param north_wall True if there is wall in the north. False otherwise.
   * @param south_wall True if there is wall in the south. False otherwise.
   * @param east_wall True if there is wall in the east. False otherwise.
   * @param west_wall True if there is wall in the west. False otherwise.
   * @param visited Flag to inform if the cell was visited.
   */
  void setMazeMatrixCell(uint8_t row, uint8_t column, bool north_wall = false, bool south_wall = false,
                         bool east_wall = false, bool west_wall = false, bool visited = false);
  
  /**
   * @brief Converts the cell walls information from global to local reference.
   * 
   * @param maze_matrix The maze obstacle matrix.
   * @param doogie_position Position of the robot in the maze matrix.
   * @return LocalCell Struct with walls information referenced in respect to the robot.
   */
  static LocalCell globalToLocalCell(MazeMatrix maze_matrix, doogie_msgs::DoogiePosition doogie_position);
  
  /**
   * @brief Converts the cell walls information from global to local reference.
   * 
   * @param doogie_position Position of the robot in the maze matrix.
   * @return LocalCell Struct with walls information referenced in respect to the robot.
   */
  LocalCell globalToLocalCell(doogie_msgs::DoogiePosition doogie_position);

  /**
   * @brief Converts the cell walls information from local to global reference.
   * 
   * @param orientation Orientation of the robot in the global reference.
   * @param local_cell Struct with walls information referenced in respect to the robot.
   * @return doogie_msgs::MazeCell Object holding the walls information in the global reference.
   */
  static doogie_msgs::MazeCell localToGlobalCell(uint8_t orientation, LocalCell local_cell);
  
  /**
   * @brief Get the number of rows of internal maze matrix.
   * 
   * @return uint8_t Number of rows of internal maze matrix.
   */
  uint8_t getNumberOfRows();

  /**
   * @brief Get the number of columns of internal maze matrix.
   * 
   * @return uint8_t Number of columns of internal maze matrix.
   */
  uint8_t getNumberOfColumns();

 private:
  /**
   * @brief Converts the wall information from global to local reference.
   * 
   * @param orientation Orientation of the robot in the global reference.
   * @param maze_cell_walls Object holding the walls information in the global reference.
   * @return LocalCell Struct with walls information referenced in respect to the robot.
   */
  static LocalCell globalToLocal(uint8_t orientation, doogie_msgs::MazeCell maze_cell_walls);
  
  /** Internal maze matrix. */
  doogie_msgs::MazeCellMultiArray maze_matrix_;
};

}

#endif  // #ifndef DOOGIE_CORE_MAZE_MATRIX_HANDLE_HPP_
