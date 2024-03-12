//
//  main.cpp
//  Hungarian Algorithm
//
//  Created by Tianyuan Chen on 3/8/24.
//

#include <iostream>
#include <opencv2/core.hpp>
#include "hungarian_algorithm.hpp"

int main() {

    cv::Mat_<float> cost = (cv::Mat_<float>(5, 4) <<
            10, 19, 8, 15,
            10, 18, 7, 17,
            13, 16, 9, 14,
            12, 19, 8, 18,
            14, 17, 10, 19);

    HungarianAlgorithm HungAlgo = HungarianAlgorithm();
    HungAlgo.Load(cost);
    cv::Mat_<uchar> dest;
    HungAlgo.Solve(dest);
    
    // Display the original matrix
    std::cout << "Cost matrix : \n" << cost << "\n\n";
    std::cout << "Assignment matrix : \n" << dest << "\n\n";
    return 0;
}
