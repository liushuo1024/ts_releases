/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_POINT_SET_H_
#define amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_POINT_SET_H_
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "amr_common/geometry/point.h"

namespace amr_geometry {
class PointSet : public std::vector<Point> {
 public:
  /**
   * @brief  [calculate distance of every two points, stored in matrix m_]
   */
  void MakeDistanceMatrix() {
    if (this->empty()) {
      return;
    }

    m_.resize(this->size(), this->size());

    for (uint32_t i = 0; i < this->size(); i++) {
      for (uint32_t j = 0; j < this->size(); j++) {
        if (i == j) {
          m_(i, j) = 0;
        } else if (i < j) {
          m_(i, j) = std::hypot(this->at(i).x() - this->at(j).x(),
                                this->at(i).y() - this->at(j).y());
        } else {
          m_(i, j) = m_(j, i);
        }
      }
    }
  }

  PointSet GetPointsWithMaxDistance() {
    PointSet z;
    Eigen::Index max_row, max_col;
    m_.maxCoeff(&max_row, &max_col);
    z.push_back(this->at(static_cast<size_t>(max_row)));
    z.push_back(this->at(static_cast<size_t>(max_col)));
    return z;
  }

  /**
   * @brief  [find a pair of points according to distance]
   */
  std::vector<PointSet> GetCandidateWithDistance(double distance,
                                                 double threshold) {
    std::vector<PointSet> candidates;
    for (Eigen::Index i = 0; i < m_.rows(); i++) {
      for (Eigen::Index j = 0; j < m_.cols(); j++) {
        if (std::fabs(m_(i, j) - distance) < threshold) {
          PointSet candidate;
          candidate.push_back(this->at(static_cast<size_t>(i)));
          candidate.push_back(this->at(static_cast<size_t>(j)));
          candidates.push_back(candidate);
        }
      }
    }
    return candidates;
  }

  /**
   * @brief  [return true if distances of the last pairs are equal]
   */
  bool AddApointCorrespond(PointSet ptSet, const Point &pt, double threshold) {
    ptSet.push_back(pt);
    for (uint32_t i = 0; i < this->size() - 1; i++) {
      auto distPtSet = std::hypot(ptSet.back().x() - ptSet[i].x(),
                                  ptSet.back().y() - ptSet[i].y());
      auto distThis = std::hypot((*this).back().x() - (*this)[i].x(),
                                 (*this).back().y() - (*this)[i].y());
      if (std::abs(distPtSet - distThis) >= threshold) {
        return false;
      }
    }

    return true;
  }

  /**
   * @brief  [return if not contain pt]
   */
  bool NotContains(const Point &pt) {
    for (auto element : *this) {
      if (element == pt) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief  [choose k points from the pointSet]
   */
  std::vector<PointSet> choose(int k) {
    std::vector<int> input(k);
    std::vector<std::vector<int>> output;
    Cij(this->size(), k, &input, k, &output);

    std::vector<PointSet> ptSetList;
    for (auto i : output) {
      PointSet ptSet;
      for (auto j : i) {
        ptSet.push_back(this->at(j));
      }
      ptSetList.push_back(ptSet);
    }

    return ptSetList;
  }

  Eigen::MatrixXd m() { return m_; }
  double MaxDistance() { return m_.maxCoeff(); }

 private:
  /**
   * @brief  [choose i numbers from j numbers]
   */
  // TODO(qiant): refactor!!!
  void Cij(int i, int j, std::vector<int> *r, int num,
           std::vector<std::vector<int>> *result) {
    if (j == 1) {
      for (int a = 0; a < i; a++) {
        std::vector<int> temp(num);
        (*r)[num - 1] = a;
        for (int b = 0; b < num; b++) {
          temp[b] = r->at(b);
        }
        result->push_back(temp);
      }
    } else if (j == 0) {
      // do nothing!
    } else {
      for (int a = i; a >= j; a--) {
        (*r)[j - 2] = a - 1;
        Cij(a - 1, j - 1, r, num, result);
      }
    }
  }

  // distance matrix
  Eigen::MatrixXd m_;
};

}  // namespace amr_geometry

#endif  // amr_COMMON_INCLUDE_amr_COMMON_GEOMETRY_POINT_SET_H_
