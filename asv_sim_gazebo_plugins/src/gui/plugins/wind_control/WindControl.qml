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
  id: windControl
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

  // wind topic
  Text {
    Layout.columnSpan: 2
    id: windTopicText
    color: "dimgrey"
    text: "Wind topic"
  }

  // todo(srmainwaring) add box for topic
  TextField {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: windTopic
    text: WindControl.topic
    selectByMouse: true
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
    // value: WindControl.windSpeed
    decimals: 1
    stepSize: 0.5
    onEditingFinished: {
      WindControl.SetWindSpeed(windSpeed.value)
    }
  }

  // wind direction (deg from North)
  Text {
    Layout.columnSpan: 2
    id: windDirectionText
    color: "dimgrey"
    text: "Wind direction (deg)"
  }

  // binding to WindControl.windDirection does not work as the field
  // cannot be edited...
  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: windDirection
    maximumValue: 180
    minimumValue: -180
    // value: WindControl.windDirection
    decimals: 0
    stepSize: 10
    onEditingFinished: {
      WindControl.SetWindDirection(windDirection.value)
    }
  }

  // Bottom spacer
  Item {
    Layout.columnSpan: 4
    Layout.fillHeight: true
  }
}
