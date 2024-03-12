//
//  hungarian_algorithm.cpp
//  Hungarian Algorithm
//
//  Created by Tianyuan Chen on 3/8/24.
//

#include "hungarian_algorithm.hpp"

void HungarianAlgorithm::Load(cv::Mat_<float>& source)
{
    if (source.cols < source.rows) {
        cv::rotate(source, cost_matrix, cv::ROTATE_90_CLOCKWISE);
        rotated = true;
    }
    else {
        source.copyTo(cost_matrix);
        rotated = false;
    }
    m_rows = cost_matrix.rows;
    m_cols = cost_matrix.cols;
    row_cover = cv::Mat::zeros(1, m_rows, CV_8U);
    col_cover = cv::Mat::zeros(1, m_cols, CV_8U);
    mask_matrix = cv::Mat::zeros(cost_matrix.rows, cost_matrix.cols, CV_8U);
    next_step = 1;
}

void HungarianAlgorithm::Process()
{
    bool done = false;
    while (!done)
    {
        switch (next_step)
        {
        case 1:
            step1();
            break;
        case 2:
            step2();
            break;
        case 3:
            step3();
            break;
        case 4:
            step4();
            break;
        case 5:
            step5();
            break;
        case 6:
            step6();
            break;
        case 7:
            done = true;
            break;
        }
    }
}

void HungarianAlgorithm::Solve(cv::Mat_<uchar>& dest)
{
    Process();
    if (rotated)
        cv::rotate(mask_matrix, dest, cv::ROTATE_90_COUNTERCLOCKWISE);
    else
        mask_matrix.copyTo(dest);
}

// row reduction.
void HungarianAlgorithm::step1()
{
    cv::Mat minValues;
    cv::reduce(cost_matrix, minValues, 1, cv::REDUCE_MIN);
    for (int r = 0; r < cost_matrix.rows; ++r) {
        cost_matrix.row(r) -= minValues.at<float>(r);
    }
    next_step = 2;
}

// randomly initialize the star indices.
void HungarianAlgorithm::step2()
{
    cv::Mat zero_indices;
    cv::findNonZero(cost_matrix == 0, zero_indices);
    for (int i = 0; i < zero_indices.rows; ++i) {
        int r = zero_indices.at<cv::Point>(i).y;
        int c = zero_indices.at<cv::Point>(i).x;
        if (row_cover.at<uchar>(r) == 0 && col_cover.at<uchar>(c) == 0) {
            mask_matrix.at<uchar>(r, c) = 1;
            row_cover.at<uchar>(r) = 1;
            col_cover.at<uchar>(c) = 1;
        }
    }
    row_cover.setTo(0);
    col_cover.setTo(0);
    next_step = 3;
}

void HungarianAlgorithm::step3()
{
    cv::Mat nonzero_cols;
    cv::findNonZero(mask_matrix, nonzero_cols);
    for (int i = 0; i < nonzero_cols.rows; ++i) {
        int c = nonzero_cols.at<cv::Point>(i).x;
        col_cover.at<uchar>(c) = 1;
    }
    if (cv::sum(col_cover)[0] >= m_rows) {
        next_step = 7;
        return;
    }
    else {
        next_step = 4;
    }
}

void HungarianAlgorithm::step4()
{
    for (int r = 0; r < m_rows; r++) {
        for (int c = 0; c < m_cols; c++) {
            if (cost_matrix.at<float>(r, c) == 0
                && row_cover.at<uchar>(r) == 0
                && col_cover.at<uchar>(c) == 0)
            {
                mask_matrix.at<uchar>(r, c) = 2;
                bool temp = false;
                for (int c1 = 0; c1 < m_cols; c1++) {
                    if (mask_matrix.at<uchar>(r, c1) == 1) {
                        row_cover.at<uchar>(r) = 1;
                        col_cover.at<uchar>(c1) = 0;
                        temp = true;
                    }
                }
                if (temp == false) {
                    next_step = 5;
                    path_row_0 = r;
                    path_col_0 = c;
                    return;
                }
            }
        }
    }
    next_step = 6;
    return;
}

void HungarianAlgorithm::step5()
{
    int star_row = -1;
    int prime_col = -1;
    bool done = false;
    cv::Mat path = cv::Mat::zeros(m_cols*m_rows, 2, CV_8U);
    int path_count = 1;
    path.at<uchar>(path_count - 1, 0) = path_row_0;
    path.at<uchar>(path_count - 1, 1) = path_col_0;
    while (!done) {
        // find star in col
        star_row = -1;
        for (int i = 0; i < m_rows; i++)
            if (mask_matrix.at<uchar>(i, path.at<uchar>(path_count - 1, 1)) == 1)
                star_row = i;
        if (star_row > -1)
        {
            path_count += 1;
            path.at<uchar>(path_count - 1, 0) = star_row;
            path.at<uchar>(path_count - 1, 1) = path.at<uchar>(path_count - 2, 1);
        }
        else
            done = true;
        if (!done)
        {
            // find prime in row
            for (int j = 0; j < m_cols; j++) {
                if (mask_matrix.at<uchar>(path.at<uchar>(path_count - 1, 0), j) == 2)
                    prime_col = j;
            }
            path_count += 1;
            path.at<uchar>(path_count - 1, 0) = path.at<uchar>(path_count - 2, 0);
            path.at<uchar>(path_count - 1, 1) = prime_col;
        }
    }
    // augment path
    for (int p = 0; p < path_count; p++) {
        if (mask_matrix.at<uchar>(path.at<uchar>(p, 0), path.at<uchar>(p, 1)) == 1)
            mask_matrix.at<uchar>(path.at<uchar>(p, 0), path.at<uchar>(p, 1)) = 0;
        else
            mask_matrix.at<uchar>(path.at<uchar>(p, 0), path.at<uchar>(p, 1)) = 1;
    }
    row_cover.setTo(0);
    col_cover.setTo(0);
    for (int r = 0; r < m_rows; r++)
        for (int c = 0; c < m_cols; c++)
            if (mask_matrix.at<uchar>(r, c) == 2)
                mask_matrix.at<uchar>(r, c) = 0;
    next_step = 3;
}

void HungarianAlgorithm::step6()
{
    float minval = std::numeric_limits<float>::max();
    for (int r = 0; r < m_rows; r++) {
        for (int c = 0; c < m_cols; c++) {
            if (row_cover.at<uchar>(r) == 0 && col_cover.at<uchar>(c) == 0 && minval > cost_matrix.at<float>(r, c))
                minval = cost_matrix.at<float>(r, c);
        }
    }
    for (int r = 0; r < m_rows; r++)
        for (int c = 0; c < m_cols; c++) {
            if (row_cover.at<uchar>(r) == 1)
                cost_matrix.at<float>(r, c) += minval;
            if (col_cover.at<uchar>(c) == 0)
                cost_matrix.at<float>(r, c) -= minval;
        }
    next_step = 4;
}
