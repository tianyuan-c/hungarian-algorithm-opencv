//
//  hungarian_algorithm.hpp
//  Hungarian Algorithm
//
//  Created by Tianyuan Chen on 3/8/24.
//

#ifndef hungarian_algorithm_hpp
#define hungarian_algorithm_hpp

#include <iostream>
#include <opencv2/core.hpp>

class HungarianAlgorithm {
public:
    HungarianAlgorithm() {};
    ~HungarianAlgorithm() {};
    void Load(cv::Mat_<float>& source);
    void Solve(cv::Mat_<uchar>& dest);
    
private:
    void Process();
    // make sure that m_rows are always smaller or equal to m_cols;
    
    cv::Mat_<float> cost_matrix;
    cv::Mat_<uchar> mask_matrix;
    cv::Mat_<uchar> row_cover;
    cv::Mat_<uchar> col_cover;
    
    int m_cols    {1};
    int m_rows    {1};
    int next_step {0};
    bool rotated  {false};
    int path_row_0 {0};
    int path_col_0 {0};
    
    void step1();
    void step2();
    void step3();
    void step4();
    void step5();
    void step6();
};

#endif /* hungarian_algorithm_hpp */
