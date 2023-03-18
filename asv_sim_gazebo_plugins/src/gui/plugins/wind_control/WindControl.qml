// Copyright (C) 2022-2023  Rhys Mainwaring
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

import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import "qrc:/qml"

GridLayout {
  columns: 6
  columnSpacing: 10
  Layout.minimumWidth: 300
  Layout.minimumHeight: 350
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  // Left spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  // wind speed
  Text {
    Layout.columnSpan: 2
    id: windSpeedText
    color: "dimgrey"
    text: "Wind speed (m/s)"
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: windSpeed
    maximumValue: 100.0
    minimumValue: 0.0
    value: 0.0
    decimals: 1
    stepSize: 0.5
    onEditingFinished: WindControl.UpdateWindSpeed(windSpeed.value)
  }

  // wind angle
  Text {
    Layout.columnSpan: 2
    id: windAngleText
    color: "dimgrey"
    text: "Wind angle (deg)"
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: windAngle
    maximumValue: 180
    minimumValue: -180
    value: 0
    decimals: 0
    stepSize: 10
    onEditingFinished: WindControl.UpdateWindAngle(windAngle.value)
  }

  // Bottom spacer
  Item {
    Layout.columnSpan: 4
    Layout.fillHeight: true
  }
}
