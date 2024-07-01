import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.3
import QtQuick.Dialogs 1.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Extras 1.4


import "../../View"
import "../../Component"
import "../../Component/MaterialDesign"
import "../../Dialog"

Popup {
    id: inputDialog
    width: 1027
    height: inputColumn.height
    spacing: 0
    padding: 0
    signal timeChanged()
    property var unit: [
    {name: "Year", min: 2000, max: 2200},
    {name: "Month", min: 01, max: 12},
    { name: "Day", min: 01, max: 31},
    {name: "Hour", min: 00, max: 23},
    {name: "Minute", min: 00, max: 59},
    {name: "Second", min: 00, max: 59}]
    onVisibleChanged: {
        realTime.running = visible
    }
    property int yearValue
    property int monthValue
    property int dayValue
    property int hourValue
    property int minuteValue
    property int secondValue
    function updateTumblerValues(index) {
        var currentDate = new Date();
        switch(index) {
        case 0:
            var currentYeah = currentDate.getFullYear()
            settingVM.updateCurrentTime(index, currentYeah)
            return currentYeah - 2000
            break;
        case 1:
            var currentMonth = currentDate.getMonth()
            settingVM.updateCurrentTime(index, currentMonth + 1)
            return currentMonth
            break;
        case 2:
            var currentDay = currentDate.getDate()
            settingVM.updateCurrentTime(index, currentDay)
            return currentDay - 1
            break;
        case 3:
            var currentHour = currentDate.getHours()
            settingVM.updateCurrentTime(index, currentHour)
            return currentHour
            break;
        case 4:
            var currentMinute = currentDate.getMinutes()
            settingVM.updateCurrentTime(index, currentMinute)
            return currentMinute
            break;
        case 5:
            var currentSecond = currentDate.getSeconds()
            settingVM.updateCurrentTime(index, currentSecond)
            return currentSecond
            break;
        }
    }
    Timer{
        id: realTime
        interval: 1000
        repeat: true
        triggeredOnStart: true
        onTriggered:{
            inputDialog.timeChanged()
        }
    }
    Column{
        id: inputColumn
        width: parent.width
        Row{
            id: timerRow
            width: parent.width
            height: 200
            spacing: 0
            anchors.horizontalCenter: parent.horizontalCenter
            Repeater{
                id: syncRpt
                model: unit
                delegate: Rectangle{
                    width: inputDialog.width / 6
                    height: 200
                    border{
                        color: "black"
                        width: 2
                    }
                    Column{
                        width: 112
                        height: 150
                        anchors.centerIn: parent
                        spacing: 0
                        Text{
                            width: 100
                            height: 50
                            text: modelData.name
                            font.pixelSize: 20
                            horizontalAlignment: Text.AlignHCenter
                            anchors.horizontalCenter: parent.horizontalCenter
                        }
                        Item{
                            width: 112
                            height: 100
                            anchors.horizontalCenter: parent.horizontalCenter
                            Tumbler {
                                id: tumblerItem
                                anchors.fill: parent
                                style: TumblerStyle {
                                    id: tumblerStyle
                                    visibleItemCount: 4

                                    delegate: Item {
                                        implicitHeight: (tumblerItem.height - padding.top - padding.bottom) / tumblerStyle.visibleItemCount

                                        Rectangle{
                                            width: parent.width
                                            height: parent.height * 1.25
                                            anchors.centerIn: parent
                                            anchors.margins: 0
                                            border.color:styleData.current ? "black" : "transparent"
                                            border.width: 0.5
                                            color: "transparent"
                                        }

                                        Text {
                                            id: label
                                            text: styleData.value
                                            font.pixelSize: 18
                                            color: styleData.current ? "black" : "#666666"
                                            opacity: 0.4 + Math.max(0, 1 - Math.abs(styleData.displacement)) * 0.6
                                            anchors.centerIn: parent
                                            font.family: "Arial"
                                        }
                                    }
                                }
                                TumblerColumn {
                                    id: yearColumn
                                    width: 100
                                    model: ListModel {
                                        id: valueModel
                                    }
                                    Component.onCompleted: {
                                        // Populate the model with the next four years
                                        for(var i = modelData.min; i <= modelData.max; i++) {
                                            valueModel.append({ value: i });
                                        }
                                    }
                                    onCurrentIndexChanged:{
                                        settingVM.updateCurrentTime(index, valueModel.get(currentIndex).value)
                                    }
                                }   
                                Connections{
                                    target: inputDialog
                                    ignoreUnknownSignals: true
                                    onTimeChanged:{
                                        tumblerItem.setCurrentIndexAt(0,updateTumblerValues(index),0)
                                    }
                                }               
                            }
                        }
                    }
                }
            }
        }
        Row{
            width: 240
            height: 50
            anchors.horizontalCenter: parent.horizontalCenter
            CheckBox {
                id: syncCheck
                text: checked? "Automatically" : "Manually"
                checked: true
                width: 120
                height: 45
                anchors.verticalCenter: parent.verticalCenter
                indicator: Rectangle {
                    implicitWidth: 26
                    implicitHeight: 26
                    x: 90
                    y: parent.height / 2 - height / 2
                    radius: 3
                    border.color: syncCheck.down ? "#17a81a" : "#21be2b"

                    Rectangle {
                        width: 12
                        height: 12
                        x: parent.width / 2 - width / 2
                        y: parent.height / 2 - height / 2
                        radius: 2
                        color: syncCheck.down ? "#17a81a" : "#21be2b"
                        visible: syncCheck.checked
                    }
                }

                contentItem: Text {
                    text: syncCheck.text
                    color: "white"
                    font.pointSize: 14
                    font.family: "Arial"
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignRight
                    opacity: enabled ? 1.0 : 0.3
                    rightPadding: 30
                }
                onCheckedChanged: realTime.running = checked
            }
            Button{
                id: syncButton
                width: 120
                height: 45
                anchors.verticalCenter: parent.verticalCenter
                enabled: productionVM.enabledForRunLot
                contentItem: Text{
                    text: "Start Sync"
                    color: "white"
                    font.pointSize: 14
                    font.family: "Arial"
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                }
                onClicked: {
                    settingVM.startSyncTime()
                }
            }
        }
    }
    
}
