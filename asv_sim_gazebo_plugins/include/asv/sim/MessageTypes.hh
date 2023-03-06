// Copyright (C) 2019-2023 Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef ASV_SIM_MESSAGETYPES_HH_
#define ASV_SIM_MESSAGETYPES_HH_

#include <boost/shared_ptr.hpp>

#include "anemometer.pb.h"
#include "lift_drag.pb.h"


namespace asv
{
/// \brief Type definition for a pointer to a Anemometer message.
typedef const boost::shared_ptr<
  const asv_msgs::msgs::Anemometer>
    AnemometerPtr;

/// \brief Type definition for a pointer to a LiftDrag message.
typedef const boost::shared_ptr<
  const asv_msgs::msgs::LiftDrag>
    LiftDragPtr;

}  // namespace asv

#endif  // ASV_SIM_MESSAGETYPES_HH_
