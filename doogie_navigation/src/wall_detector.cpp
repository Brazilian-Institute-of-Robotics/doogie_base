#include "doogie_navigation/WallDetector.hpp"

namespace doogie_navigation{

    MatrixHandle::MatrixHandle(){
        mat.layout.dim.resize(2); 
    }
    void MatrixHandle::initMatrix(int iSize, int jSize, std::string iLabel, std::string jLabel, int fill=0, int offset=0){
        mat.layout.dim[0].size = iSize;
        mat.layout.dim[1].size = jSize;
        mat.layout.dim[0].label = iLabel;
        mat.layout.dim[1]. label = jLabel;
        mat.layout.dim[0].stride = iSize*jSize;
        mat.layout.dim[1].stride = jSize;
        mat.layout.data_offset = offset;

        doogie_msgs::MazeCellMultiArray empty_list;
        for (std::size_t i = 0; i<iSize; i++){
            for(std::size_t j = 0; j<jSize; j++){
                mat.data.push_back(doogie_msgs::MazeCell());    
            }
        }
    }

    doogie_msgs::MazeCell MatrixHandle::getMatrixElement(doogie_msgs::MazeCellMultiArray matrix, int i, int j){
        int dstride1 = matrix.layout.dim[1].stride;
        int offset = matrix.layout.data_offset;
        return matrix.data[offset + i + dstride1*j];
    }

    void MatrixHandle::setMatrixElement(doogie_msgs::MazeCellMultiArray matrix, int i, int j, bool northElement, bool southElement, bool eastElement, bool westElement, bool visited){
        int dstride1 = matrix.layout.dim[1].stride;
        int offset = matrix.layout.data_offset;
        matrix.data[offset + i + dstride1*j].north_wall = northElement;
        matrix.data[offset + i + dstride1*j].south_wall = southElement;
        matrix.data[offset + i + dstride1*j].east_wall = eastElement;
        matrix.data[offset + i + dstride1*j].west_wall = westElement;
        matrix.data[offset + i + dstride1*j].visited = visited;
    }
}