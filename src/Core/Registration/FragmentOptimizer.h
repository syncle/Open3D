#pragma once

#include <vector>
#include <unordered_set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SparseCore>
#include <IO/ClassIO/PointCloudIO.h>
#include "HashSparseMatrix.h"
#include "PointCloudFragment.h"
#include "RGBDTrajectory.h"

typedef Eigen::SparseMatrix< double > SparseMatrix;

typedef std::pair< int, int > CorrespondencePair;

struct Correspondence {
public:
    int idx0_, idx1_;
    Eigen::Matrix4d trans_;
    std::vector< CorrespondencePair > corres_;
public:
    Correspondence( int i0, int i1 ) : idx0_( i0 ), idx1_( i1 ) {}
    void LoadFromFile( std::string filename ) {
        FILE * f = fopen( filename.c_str(), "r" );
        if ( f != NULL ) {
            char buffer[1024];
            CorrespondencePair pair;
            while ( fgets( buffer, 1024, f ) != NULL ) {
                if ( strlen( buffer ) > 0 && buffer[ 0 ] != '#' ) {
                    sscanf( buffer, "%d %d", &pair.first, &pair.second );
                    corres_.push_back( pair );
                }
            }
            fclose ( f );
        }
    }
};

namespace three {

    int FragmentOptimizer(FragmentOptimizerOption& option);

}
