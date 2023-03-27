// Copyright (C) 2023 Rhys Mainwaring
//

// Adapted from the MooringForce system developed for the MBARI wave buoy
// including https://github.com/osrf/buoy_sim/pull/135
//
// Authors: Mabel Zhang <mabel@openrobotics.org>
//          Michael Anderson <anderson@mbari.org>
//          Rhys Mainwaring <rhys.mainwaring@me.com>

// Copyright 2022 Open Source Robotics Foundation, Inc.
//                and Monterey Bay Aquarium Research Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ASV_SIM_CATENARYSOLN_HH_
#define ASV_SIM_CATENARYSOLN_HH_

#include <eigen3/unsupported/Eigen/NonLinearOptimization>

#include <cmath>

#include <gz/common/Console.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{

/////////////////////////////////////////////////
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum { InputsAtCompileTime = NX, ValuesAtCompileTime = NY };
  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime>
    JacobianType;

  const int inputs_;
  const int values_;

  /// \brief Constructor.
  Functor()
      : inputs_(InputsAtCompileTime), values_(ValuesAtCompileTime)
  {
  }

  /// \brief Constructor.
  Functor(int _inputs, int _values)
      : inputs_(_inputs), values_(_values)
  {
  }

  /// \brief The number of inputs.
  int inputs() const
  {
    return inputs_;
  }

  /// \brief The number of values.
  int values() const
  {
    return values_;
  }

  // Define an operator overload in the subclass
  // int operator() (const InputType &_x, ValueType *_v, JacobianType *_j = 0)
  // const;
};

/////////////////////////////////////////////////
struct CatenaryFunction
{
  /// \brief Scaling factor for catenary chain
  /// \param V Vertical distance from buoy to anchor (metres)
  /// \param B Length of mooring chain lying on seafloor,
  /// start of catenary (metres).
  /// \param L Total length of mooring chain (metres).
  /// \return  Return scaling factor c, ratio of the horizontal component
  /// of chain tension and weight of cable per unit length
  public: static double CatenaryScalingFactor(double _V, double _B, double _L)
  {
    // Scaling factor c
    return (std::pow((_L - _B), 2.0) - std::pow(_V, 2.0)) / (2.0 * _V);
  }
};

/////////////////////////////////////////////////
struct CatenaryHSoln : Functor<double>
{
  /// \brief Vertical distance from buoy to anchor (metres).
  private: double V{std::nanf("")};

  /// \brief Horizontal distance from buoy to anchor (metres).
  private: double H{std::nanf("")};

  /// \brief Total length of mooring chain (metres).
  private: double L{std::nanf("")};

  /// \brief Constructor.
  public: CatenaryHSoln(double _V, double _H, double _L)
      : Functor<double>(1, 1),
      V(_V),
      H(_H),
      L(_L)
  {
  }

  /// \brief Invert the catenary equation and solve for the horizontal input.
  ///
  /// \param B length of chain on floor.
  public: double InverseCatenaryVSoln(double _B) const
  {
    double c = CatenaryFunction::CatenaryScalingFactor(this->V, _B, this->L);
    return c * std::acosh(this->V / c + 1.0) + _B;
  }

  /// \brief Evaluate the target function.
  ///
  /// Know 0 <= B < L - V. Take in B = L - V - b as initial guess
  public: int operator()(
      const Eigen::VectorXd &_B,
      Eigen::VectorXd &_fvec) const
  {
    if (_B.size() < 1)
    {
      gzerr << "Invalid input size for InverseCatenaryVSoln::operator()\n";
      return -1;
    }

    // Evaluate target function.
    _fvec[0] = this->InverseCatenaryVSoln(_B[0]) - this->H;

    return 0;
  }
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // ASV_SIM_CATENARYSOLN_HH_
