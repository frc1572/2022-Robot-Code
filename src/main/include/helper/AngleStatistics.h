#pragma once

#include <functional>
#include <vector>

#include <Eigen/Core>
#include <frc/MathUtil.h>

template <int States>
std::function<Eigen::Vector<double, States>(const Eigen::Vector<double, States>&, const Eigen::Vector<double, States>&)>
AngleResidual(std::vector<int> angleStateIndices)
{
    return [=](auto a, auto b)
    {
        Eigen::Vector<double, States> ret = a - b;
        for (auto angleStateIdx : angleStateIndices)
        {
            ret[angleStateIdx] = frc::AngleModulus(units::radian_t{ret[angleStateIdx]}).value();
        }
        return ret;
    };
}

template <int States>
std::function<Eigen::Vector<double, States>(const Eigen::Vector<double, States>&, const Eigen::Vector<double, States>&)>
AngleAdd(std::vector<int> angleStateIndices)
{
    return [=](auto a, auto b)
    {
        Eigen::Vector<double, States> ret = a + b;
        for (auto angleStateIdx : angleStateIndices)
        {
            ret[angleStateIdx] = frc::InputModulus(ret[angleStateIdx], -wpi::numbers::pi, wpi::numbers::pi);
        }
        return ret;
    };
}

template <int CovDim, int States>
std::function<Eigen::Vector<double, CovDim>(
    const Eigen::Matrix<double, CovDim, 2 * States + 1>&, const Eigen::Vector<double, 2 * States + 1>&)>
AngleMean(std::vector<int> angleStateIndices)
{
    return [=](auto sigmas, auto Wm)
    {
        Eigen::Vector<double, CovDim> ret = sigmas * Wm;
        for (auto angleStateIdx : angleStateIndices)
        {
            double sumSin = sigmas.row(angleStateIdx).unaryExpr([](auto it) { return std::sin(it); }).sum();
            double sumCos = sigmas.row(angleStateIdx).unaryExpr([](auto it) { return std::cos(it); }).sum();
            ret[angleStateIdx] = std::atan2(sumSin, sumCos);
        }
        return ret;
    };
}