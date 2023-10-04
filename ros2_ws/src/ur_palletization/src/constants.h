#pragma once
#include <Eigen/Dense>
namespace Palletization::Constants
{
    const Eigen::Vector3d BoxDimension{ 0.3, 0.3, 0.2 };
    const Eigen::Vector3d PalletDimensions{ 1.2, 0.769, 2.0*0.111 };
    constexpr float DropRise = 1.2f;

    constexpr float Separation = 1.1f;
    const static Eigen::Vector3d PickupZStartOffset {0.1f * Eigen::Vector3d::UnitZ()};
    const static Eigen::Vector3d PickupZStartOffset2 {0.3f * Eigen::Vector3d::UnitZ()};

    const static Eigen::Vector3d PickupZStopOffset { 0.05f * Eigen::Vector3d::UnitZ()};

    const static Eigen::Vector3d DropZStartOffset {0.25f * Eigen::Vector3d::UnitZ()};
    const static Eigen::Vector3d PickupZOffset { +0.05f * Eigen::Vector3d::UnitZ()}; // pallet height / 2

    const Eigen::Quaterniond OrientationDown0{ 0.5, 0.5, -0.5, 0.5 };
    const Eigen::Quaterniond OrientationDown2{ -0.5, -0.5, 0.5, -0.5 };
    const Eigen::Quaterniond OrientationDown1{ 0.0, 0.7071067690849304, 0.0, 0.7071067690849304 };
    const Eigen::Quaterniond OrientationDown3{ 0.0, 0.7071067690849304, 0.0, 0.7071067690849304};

    const std::vector<Eigen::Quaterniond> OrientationsDown{ OrientationDown0, OrientationDown1, OrientationDown2, OrientationDown3 };

    constexpr char BoxNamePrefix[] = "Box";
    constexpr char PalletNamePrefix[] = "EuroPallet";
    constexpr char PickedBoxName[] = "PickedBox";

    const std::vector<Eigen::Vector3f> Pattern{
            //    { -1.f, 0.5, 1.f }, { 0.f, 0.5f, 1.f }, { 1.f, 0.5f, 1.f },{ -1.f, -0.5f, 1.f }, { 0.f, -0.5f, 1.f }, { 1.f,
            //    -0.5f, 1.f },
            //    { -1.f, 0.5, 2.f }, { 0.f, 0.5f, 2.f }, { 1.f, 0.5f, 2.f }, { -1.f, -0.5f, 2.f }, { 0.f, -0.5f, 2.f }, { 1.f,
            //    -0.5f, 2.f },
            { -1.0f, 0.5f, 1.f },  { 0.0f, 0.5f, 1.f },  { 1.0f, 0.5f, 1.f },  { -1.0f, -0.5f, 1.f }, { 0.0f, -0.5f, 1.f },
            { 1.0f, -0.5f, 1.f },  { -1.0f, 0.5f, 2.f }, { 0.0f, 0.5f, 2.f },  { 1.0f, 0.5f, 2.f },   { -1.0f, -0.5f, 2.f },
            { 0.0f, -0.5f, 2.f },  { 1.0f, -0.5f, 2.f }, { -1.0f, 0.5f, 3.f }, { 0.0f, 0.5f, 3.f },   { 1.0f, 0.5f, 3.f },
            { -1.0f, -0.5f, 3.f }, { 0.0f, -0.5f, 3.f }, { 1.0f, -0.5f, 3.f },

    };
}
