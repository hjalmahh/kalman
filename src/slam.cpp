#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include "slam.h"
#include <ceres/ceres.h>



//page 72 MR 2019, bare rotasjon om z axe
Eigen::Matrix2d rotate(double radians)
{
    Eigen::Matrix2d matrix;
    matrix <<
        std::cos(radians), -std::sin(radians),
        std::sin(radians), std::cos(radians);
    return matrix;
}


Eigen::Vector2d transformation_matrix_point(const Eigen::Matrix2d &r,
                                            const Eigen::Vector2d &t,
                                            const Eigen::Vector2d &p)
{
    return r * p + t;
}

// ----- Submap class implementation ----- chat bot fra her

Submap::Submap(double resolution, int width, int height, double p_min, double p_max)
    : resolution_(resolution), width_(width), height_(height),
      p_min_(p_min), p_max_(p_max), grid_(width, height) {
    grid_.setConstant(0.5);  // initialize all cells to unknown probability 0.5
}

double Submap::odds(double p) const {
    return p / (1.0 - p);
}

double Submap::oddsInv(double o) const {
    return o / (1.0 + o);
}

Eigen::Vector2i Submap::worldToGrid(const Eigen::Vector2d& world_point) const {
    int i = static_cast<int>(world_point.x() / resolution_);
    int j = static_cast<int>(world_point.y() / resolution_);
    return Eigen::Vector2i(i, j);
}

void Submap::insertHit(const Eigen::Vector2d& world_point) {
    Eigen::Vector2i idx = worldToGrid(world_point);
    if (idx.x() >= 0 && idx.x() < width_ && idx.y() >= 0 && idx.y() < height_) {
        double old_p = grid_(idx.x(), idx.y());
        double new_odds = odds(old_p) * odds(0.7);  // assume hit prob = 0.7
        grid_(idx.x(), idx.y()) = std::clamp(oddsInv(new_odds), p_min_, p_max_);
    }
}

void Submap::insertMiss(const Eigen::Vector2d& world_point) {
    Eigen::Vector2i idx = worldToGrid(world_point);
    if (idx.x() >= 0 && idx.x() < width_ && idx.y() >= 0 && idx.y() < height_) {
        double old_p = grid_(idx.x(), idx.y());
        double new_odds = odds(old_p) * odds(0.4);  // assume miss prob = 0.4
        grid_(idx.x(), idx.y()) = std::clamp(oddsInv(new_odds), p_min_, p_max_);
    }
}

double Submap::getProbability(int i, int j) const {
    if (i >= 0 && i < width_ && j >= 0 && j < height_) {
        return grid_(i, j);
    }
    return 0.5;  // unknown if out of bounds
}

//--------------
#include <ceres/ceres.h>

struct ScanMatcherCost {
    Submap* submap;
    Eigen::Vector2d scan_point;

    ScanMatcherCost(Submap* m, const Eigen::Vector2d& p) : submap(m), scan_point(p) {}

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        T cos_theta = ceres::cos(pose[2]);
        T sin_theta = ceres::sin(pose[2]);

        T x_trans = pose[0] + cos_theta * T(scan_point.x()) - sin_theta * T(scan_point.y());
        T y_trans = pose[1] + sin_theta * T(scan_point.x()) + cos_theta * T(scan_point.y());

        double probability = submap->getInterpolatedProbability(double(x_trans), double(y_trans));
        residual[0] = T(1.0 - probability);

        return true;
    }
};
