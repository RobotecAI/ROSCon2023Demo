#pragma once
#include <Eigen/Dense>

const Eigen::Vector3d BoxDimension{ 0.3, 0.3, 0.2 };
const Eigen::Vector3d PalletDimensions{ 1.2, 0.769, 2.0*0.111 };
constexpr float DropRise = 1.2f;

constexpr float Separation = 1.05f;
const static Eigen::Vector3d PickupZStartOffset {0.1f * Eigen::Vector3d::UnitZ()};
const static Eigen::Vector3d PickupZStartOffset2 {0.3f * Eigen::Vector3d::UnitZ()};

const static Eigen::Vector3d PickupZStopOffset { 0.05f * Eigen::Vector3d::UnitZ()};

const static Eigen::Vector3d DropZStartOffset {0.25f * Eigen::Vector3d::UnitZ()};
const static Eigen::Vector3d PickupZOffset { +0.05f * Eigen::Vector3d::UnitZ()}; // pallet height / 2