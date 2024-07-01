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

    Item{ // Button control AFolding
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
                // model: ["Set Carton\nPicked","Wait For\nPickup Station","Get Next\nPickup Station","Wait For\nPickup Fill","PIM I/O"]
                model: ["Carton Transfer I/O", "Parameters"]
                Layout.fillHeight: true
                Item{
                    Layout.rowSpan: 2
                    Layout.columnSpan: 1                
                    Layout.preferredWidth: gridPBtn.width / gridPBtn.columns * Layout.rowSpan
                    Layout.preferredHeight: gridPBtn.height / gridPBtn.rows * Layout.columnSpan                
                    Layout.alignment: Qt.AlignCenter | Qt.AlignTop  | Qt.AlignLeft
                    Button{
                        text: modelData
                        width: parent.width * 0.95
                        height: parent.height * 0.9
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
                                // case 0: stationVM.ctCartonPicked()
                                //     break
                                // case 1: stationVM.ctWaitStation()   
                                //     break
                                // case 2: stationVM.ctGetStation()   
                                //     break
                                // case 3: stationVM.ctWaitFill()     
                                //     break
                                // case 4: stationVM.goToIoTab(_stationIndex)
                                //     break
                                case 0: stationVM.goToIoTab(_stationIndex)
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
    Parameters{
        id: parameterPopup
        x: stationLayout.width  / 2 - width  / 2 
        y: 50
		z:  20
    }
}


