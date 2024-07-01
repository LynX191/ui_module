import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.0
import QtQml.Models 2.3
import QtQuick.Dialogs 1.2
import QtQuick.Controls.Styles 1.4

import "../../../Component"
import "../../../Component/MaterialDesign"

Item {
    id: stationLayout
    property string _sourceImage: ""
    property int _fontPixelSize: 20
    property string _fontFamily:  "Arial"
    property var backgroundColor: "#424242"
    property int _borderSize: 1
    property int _stationIndex: -1
    property bool isGripperChangeActionOpen: false
    property bool _parmEnabled: stationVM.enabledParmButton
    property bool firstRunning
    Connections{
        target: systemBarVM
        ignoreUnknownSignals: true
        onProcessStateChanged:{
            if(systemBarVM.processState === "Running" && !firstRunning){
                firstRunning = true
            }
        }
    }
    Connections{
        target: master_app
        ignoreUnknownSignals: true
        onPowerUpStateChanged:{
            if(!state){
                firstRunning = false
            }
        }
    }

    Item{ // Button control Gripper
        width: parent.width
        height: parent.height
        anchors.top: parent.top
        anchors.topMargin: 100
        visible: !parameterPopup.visible
        GridLayout{
            id: gridPBtn
            width: parent.width
            height: parent.height
            anchors.fill: parent
            columns: 4
            rows: 8
            columnSpacing: 0
            rowSpacing: 0
            Repeater{
                // model: ["Gripper Action","Check Gripper","Vacuum On","Check Vacuum","Vacuum Blow Toggle","Pressure Set","TomO I/O"]
                model: ["TomO I/O", "Parameters"]
                Layout.fillHeight: true
                Layout.fillWidth: true
                delegate: Item{
                    Layout.columnSpan: 2
                    Layout.rowSpan: 1
                    Layout.preferredWidth: gridPBtn.width / gridPBtn.columns * Layout.columnSpan    
                    Layout.preferredHeight: gridPBtn.height / gridPBtn.rows * Layout.rowSpan                   
                    Layout.alignment: Qt.AlignCenter | Qt.AlignTop  | Qt.AlignLeft
                    Button{
                        id: btnRpt
                        text: modelData
                        width: parent.Layout.preferredWidth * 0.85
                        height: parent.Layout.preferredHeight * 0.8
                        anchors.centerIn: parent
                        visible: text === "Parameters" ? _parmEnabled : true
                        enabled: text === "Parameters" ? ((!parameterPopup.visible && (loginVM.parametersEnabled || loginVM.superUserActive) && productionVM.firstRunning && systemBarVM.processState !== "Running")) : productionVM.enabledForRunLot
                        opacity: enabled? 1:0.5
                        contentItem: Text{
                            anchors.fill: parent
                            text: parent.text
                            font.pixelSize: _fontPixelSize
                            font.family: _fontFamily
                            font.bold: true
                            verticalAlignment: Text.AlignVCenter
                            horizontalAlignment: Text.AlignHCenter
                            wrapMode: Text.Wrap
                            color: "white"
                        }
                        onClicked: {
                            loginVM.updateIdleTimer()
                            switch(index){
                                // case 0: isGripperChangeActionOpen = !isGripperChangeActionOpen
                                //     break
                                // case 1: stationVM.tgCheckGripper()
                                //     break
                                // case 2: stationVM.tgVacuumOn()
                                //     break
                                // case 3: stationVM.tgCheckVacuum()
                                //     break
                                // case 4: stationVM.tgVacuumBlow()
                                //     break
                                // case 5: stationVM.tgPressureSet()
                                //     break
                                // case 6:
                                //     stationVM.goToIoTab(_stationIndex)
                                //     break
                                case 0:
                                    stationVM.goToIoTab(_stationIndex)
                                    break
                                case 1: {
                                        stationVM.requestParmList(_stationIndex)
                                        parameterPopup._currentStation = _stationIndex
                                        if(!parameterPopup.visible)
                                            parameterPopup.visible = true
                                    }
                                default:
                                    break
                            }
                        }
                    }
                }                    
            }
        }
    }   
    //Popup Recipe
    Popup{
        width: 600
        height: 150
        x: parent.x - width / 2
        y: parent.y + height / 2
        visible: isGripperChangeActionOpen? true : false
        onClosed: isGripperChangeActionOpen = false
        modal: true    
        Column{
            spacing: 10
            anchors.fill: parent
            Layout.fillHeight : true 
            Row{
                height: parent.height / 2
                width: parent.width
                Layout.alignment: Qt.AlignCenter
                spacing: 15 
                Text{
                    width: parent.width / 2 - 10
                    height: parent.height
                    leftPadding: 20
                    text: "Change Gripper Action"
                    font.pixelSize: _fontPixelSize
                    font.family: _fontFamily   
                    verticalAlignment: Text.AlignVCenter 
                }
                ComboBox {
                    id: gripperCbb
                    property var recipeModel: ["Gripper Open","Gripper Close","Gripper Stop"]
                    width: parent.width / 2 - 10
                    height: parent.height
                    model: recipeModel
                    delegate: ItemDelegate {
                        width: gripperCbb.width
                        height: gripperCbb.height
                        contentItem: Text {
                            text: modelData
                            color: "black"
                            font.pixelSize: _fontPixelSize - 2
                            font.family: _fontFamily   
                            elide: Text.ElideRight
                            verticalAlignment: Text.AlignVCenter
                        }
                        highlighted: gripperCbb.highlightedIndex === index
                    }
                    contentItem: Text {
                        width: gripperCbb.width
                        height: gripperCbb.height
                        leftPadding: 5
                        rightPadding: gripperCbb.indicator.width + gripperCbb.spacing
                        text: gripperCbb.displayText
                        font.pixelSize: _fontPixelSize - 2
                        font.family: _fontFamily   
                        color: "#17a81a"
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight            
                    }
                    onCurrentTextChanged: {                         
                        displayText = currentText
                        stationVM.tgGripperValue = currentIndex
                    }
                    onActivated: {
                        loginVM.updateIdleTimer()
                    }
                }     
            }
            Item{
                width: parent.width
                height: parent.height / 2
                Button{
                    width: parent.width / 2 - 20
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: "Confirm"
                    contentItem: Text {
                        text: parent.text
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        opacity: enabled ? 1.0 : 0.3
                        color: parent.down ? "#17a81a" : "#21be2b"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight
                    }
                    onClicked: {
                        loginVM.updateIdleTimer()
                        stationVM.tgGripperAction()
                        isGripperChangeActionOpen = false
                    }
                }
            }
        }
    }
    Parameters{
        id: parameterPopup
        x: stationLayout.width  / 2 - width  / 2 
        y: 50
		z: 20
    }   
}


