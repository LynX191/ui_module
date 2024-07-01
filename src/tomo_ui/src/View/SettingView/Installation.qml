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
    id: installPopup
    width: 600
    height: childrenFlick.height + 50 + closeButton.height
    padding: 0  
    closePolicy: Popup.NoAutoClose
    modal: true

    ListModel{
        id: xavierModel
        ListElement{
            name: "Xavier 1"
            xavier: 1
            connectState: "Idle"
            colorState: "orange"
        }
        ListElement{
            name: "Xavier 3"
            xavier: 3
            connectState: "Idle"
            colorState: "orange"
        }
    }

    onVisibleChanged:{
        if(visible){
            pingTimer.triggered()
        }
        pingTimer.running = visible
    }
    Timer{
        id: pingTimer
        interval: 5000
        repeat: true
        onTriggered:{
            var enableButton = true
            for(var i = 0; i < xavierModel.count ; i++){
                var pingCheck
                if(!master_app.powerUpState)
                    pingCheck = dashboardVM.checkConnection(xavierModel.get(i).xavier, false)
                if(pingCheck === 0){
                    xavierModel.set(i, {"connectState": "Connected", "colorState":"green"})
                }
                else{
                    xavierModel.set(i, {"connectState": "Disconnected", "colorState":"red"})
                    enableButton = false
                    closeButton.visible = true
                }
            }
            buttonGroup.enabled = enableButton
        }                                
    }

    Column{
        width: parent.width
        Flickable{
            id: childrenFlick
            clip: true
            width: parent.width
            contentHeight: parentColumn.height
            height: parentColumn.height > 600 ? 600 : parentColumn.height
            ScrollBar.vertical: ScrollBar {
                policy: ScrollBar.AlwaysOff
            }
            Column{
                id: parentColumn
                width: parent.width
                Repeater{
                    model: xavierModel
                    delegate: Column{
                        width: parent.width
                        height: 50
                        Row{
                            width: parent.width
                            Button{
                                width: parent.width
                                height: 50
                                contentItem: Text{
                                    id: folderText
                                    text: processText.text !== "" ? "" : name + " " + connectState
                                    color: colorState
                                    font.pointSize: 14
                                    font.family: "Arial"
                                    verticalAlignment: Text.AlignVCenter
                                    horizontalAlignment: Text.AlignHCenter
                                    leftPadding: 5
                                }
                                Item{
                                    id: processItem
                                    width: parent.width
                                    height: parent.height * 0.75
                                    anchors.centerIn: parent
                                    Rectangle{
                                        id: xavierProcess
                                        anchors.left: parent.left
                                        height: parent.height
                                        color: "transparent"
                                    }
                                    Rectangle{
                                        id: failRec
                                        anchors.left: xavierProcess.right
                                        height: parent.height
                                        color: "transparent"
                                    }
                                    Text{
                                        id: processText
                                        anchors.fill: parent
                                        anchors.centerIn: parent
                                        z: 2
                                        color: "black"
                                        font.pointSize: 14
                                        font.family: "Arial"
                                        verticalAlignment: Text.AlignVCenter
                                        horizontalAlignment: Text.AlignHCenter
                                    }
                                }
                            }
                            Connections{
                                target: settingVM
                                ignoreUnknownSignals: true
                                onReturnAction:{
                                    var currentFolder = "xc" + xavier
                                    if(action === "file_transferred" && currentFolder === folderName){
                                        xavierProcess.width = processItem.width / total * transferred
                                        xavierProcess.color = "green"
                                        processText.text = Math.round(transferred/total * 100) + " %" 
                                        failRec.visible = false
                                        pingTimer.restart()
                                        if(total === transferred)
                                            dashboardVM.returnAction("paused")
                                    }
                                    if(action === "start_copy"){
                                        xavierProcess.width = 0
                                        processText.text = ""
                                        failRec.visible = false
                                        xavierProcess.visible = true
                                        processText.visible = true
                                    }
                                    if(action === "transfer_fail" && currentFolder === folderName){
                                        if(xavierProcess.width === 0){
                                            xavierProcess.width = processItem.width
                                            xavierProcess.color = "red"
                                        }
                                        else if(xavierProcess.width > 0){
                                            failRec.visible = true
                                        }
                                        var currentText = processText.text
                                        if(currentText.indexOf("Failed") === -1){
                                            processText.text = processText.text + " Failed"
                                        }
                                    }                                                        
                                }
                            }
                        }
                    }
                }
            }
        }
        Row{
            id: buttonGroup
            width: parent.width
            height: 50
            layoutDirection: Qt.RightToLeft
            Button{
                id: startBtn
                width: parent.width
                height: 50
                text:"Start"
                visible: true
                onClicked: {
                    startBtn.visible = false
                    pauseBtn.visible = true
                    cancelBtn.visible = true
                    settingVM.setActionList(0)
                    pingTimer.triggered()
                }
            }
            Button{
                id: pauseBtn
                width: parent.width / 2
                height: 50
                text:"Pause"
                visible: false
                onClicked: {
                    visible = false
                    continueBtn.visible = true
                    settingVM.setActionList(1)
                }
            }
            Button{
                id: continueBtn
                width: parent.width / 2
                height: 50
                text:"Restart"
                visible: false
                onClicked: {
                    visible = false
                    pauseBtn.visible = true
                    settingVM.setActionList(2)
                }
            }
            Button{
                id: cancelBtn
                width: parent.width / 2
                height: 50
                text:"Cancel"
                visible: false
                onClicked: {
                    visible = false
                    pauseBtn.visible = false
                    continueBtn.visible = false
                    startBtn.visible = true
                    settingVM.setActionList(3)
                }
            }
            Connections{
                target: settingVM
                ignoreUnknownSignals: true
                onReturnAction:{
                    if(action === "finished_copy"){
                        cancelBtn.visible = false
                        pauseBtn.visible = false
                        continueBtn.visible = false
                        startBtn.visible = true
                    }
                }
            }
        }
        Button{
            id: closeButton
            text: "Close"
            width: parent.width / 2
            height: visible ? 50 : 0
            anchors.horizontalCenter: parent.horizontalCenter
            onClicked: installPopup.close()
            Connections{
                target: settingVM
                ignoreUnknownSignals: true
                onReturnAction:{
                    if(action === "start_copy"){
                        closeButton.visible = false
                    }
                    if(action === "finished_copy"){
                        closeButton.visible = true
                    }
                }
            }
        }
    }
}
