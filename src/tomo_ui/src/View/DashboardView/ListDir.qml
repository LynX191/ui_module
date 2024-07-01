import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Dialogs 1.2
import QtQuick.VirtualKeyboard 2.2

import "../../Component"
import "../../Component/MaterialDesign"

Popup {
    id: listDirPopup
    height: childrenFlick.height + 50 + closeButton.height
    padding: 0  
    closePolicy: Popup.NoAutoClose
    modal: true
    property var children0: dashboardVM.headModel
    property var children1: dashboardVM.shipperModel
    property var children2: dashboardVM.cartonModel
    property var children3: dashboardVM.exceptModel
    property var parentFolder: [
        {name: "CartonVision"},
        {name: "StimCartoner"},
        {name: "PimCartonCounting"},
        {name: "exception_images"}]

    signal chooseParent(int indexChanged ,bool valueChanged)

    Connections{
        target: dashboardVM
        ignoreUnknownSignals: true
        onReturnAction:{
            if(action === "start_copy"){
                pingTimer.restart()
                if(!pingTimer.repeat)
                    pingTimer.repeat = !pingTimer.repeat
            }
            if(action === "finished_copy"){
                pingTimer.stop()
                if(pingTimer.repeat)
                    pingTimer.repeat = !pingTimer.repeat
            }
        }
    }
    Timer{
        id: pingTimer
        interval: 10000
        onTriggered:{
            var pingCheck = dashboardVM.checkConnection(2, false)
            if(pingCheck !== 0){
                dashboardVM.setActionList(4)
            }
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
                    model: listDirPopup.parentFolder
                    delegate: Column{
                        width: parent.width
                        height: parentName.height + childrenName.height
                        Row{
                            width: parent.width
                            Button{
                                id: parentName
                                width: parent.width - whole.width
                                height: 50
                                property bool isExpand: false
                                property string parentFolder: modelData.name
                                contentItem: Text{
                                    id: folderText
                                    text: parentName.isExpand? "[–] "+ modelData.name : "[+] " + modelData.name
                                    color: "white"
                                    font.pointSize: 14
                                    font.family: "Arial"
                                    verticalAlignment: Text.AlignVCenter
                                    horizontalAlignment: Text.AlignLeft
                                    leftPadding: 5
                                }
                                onClicked: {
                                    parentName.isExpand = !parentName.isExpand
                                }
                                Connections{
                                    target: dashboardVM
                                    ignoreUnknownSignals: true
                                    onSetExpandDir:{
                                        parentName.isExpand = camDir === index
                                    }
                                }
                            }
                            CheckBox{
                                id: whole
                                width: 30
                                height: width
                                anchors.verticalCenter: parent.verticalCenter
                                onCheckedChanged: {
                                    listDirPopup.chooseParent(index, checked)
                                }
                            }
                            Connections{
                                target: dashboardVM
                                ignoreUnknownSignals: true
                                onReturnAction:{
                                    if(action === "start_copy"){
                                        whole.enabled = false
                                    }
                                    if(action === "finished_copy"){
                                        whole.enabled = true
                                        whole.checked = false
                                    }
                                }
                            }
                        }
                        Column{
                            id: childrenName
                            width: parent.width
                            height: visible ? implicitHeight : 0
                            visible: parentName.isExpand
                            property int currentIndex: index
                            Repeater{
                                id: childrenList
                                model: eval("children" +  index)
                                delegate: Row{
                                    width: parent.width
                                    height: 30
                                    Button {
                                        id: buttonCheck
                                        width: parent.width - each.width
                                        height: 30
                                        contentItem: Text{
                                            anchors.fill: parent
                                            anchors.centerIn: parent
                                            text: modelData.name
                                            color: "white"
                                            font.pointSize: 14
                                            font.family: "Arial"
                                            verticalAlignment: Text.AlignVCenter
                                            horizontalAlignment: Text.AlignLeft
                                            leftPadding: 30
                                        }
                                        background: null
                                        onClicked: each.checked = !each.checked
                                        Item{
                                            id: processItem
                                            width: parent.width / 2 - 30
                                            height: parent.height * 0.5
                                            anchors.right: parent.right
                                            anchors.verticalCenter: parent.verticalCenter
                                            anchors.rightMargin: 15
                                            Rectangle{
                                                id: processRec
                                                height: parent.height
                                                anchors.left: parent.left
                                                anchors.verticalCenter: parent.verticalCenter
                                                anchors.rightMargin: 15
                                                color: "green"
                                                z: 0
                                            }
                                            Rectangle{
                                                id: failRec
                                                height: parent.height
                                                anchors.right: parent.right
                                                anchors.verticalCenter: parent.verticalCenter
                                                width: parent.width - processRec.width
                                                visible: false
                                                z: -2
                                                color: "red"
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
                                            Connections{
                                                target: dashboardVM
                                                ignoreUnknownSignals: true
                                                onReturnAction:{
                                                    var currentFolder = "/" + parentName.parentFolder + "/" + modelData.name
                                                    if(action === "file_transferred" && currentFolder === folderName){
                                                        processRec.width = processItem.width / total * transferred
                                                        processRec.color = "green"
                                                        processText.text = Math.round(transferred/total * 100) + " %"
                                                        failRec.visible = false
                                                        if(total === transferred)
                                                            dashboardVM.returnAction("paused")
                                                    }
                                                    if(action === "start_copy"){
                                                        processRec.width = 0
                                                        processText.text = ""
                                                        failRec.visible = false
                                                    }
                                                    if(action === "transfer_fail" && currentFolder === folderName){
                                                        if(processRec.width === 0){
                                                            processRec.width = processItem.width
                                                            processRec.color = "red"
                                                        }
                                                        else if(processRec.width > 0){
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
                                    CheckBox{
                                        id: each
                                        width: 30
                                        height: width
                                        anchors.verticalCenter: parent.verticalCenter
                                        onCheckedChanged: {
                                            dashboardVM.setSelectList("/" + parentName.parentFolder + "/" + modelData.name, checked)
                                        }
                                    }
                                    Connections{
                                        target: listDirPopup
                                        ignoreUnknownSignals: true
                                        onChooseParent:{
                                            if(childrenName.currentIndex === indexChanged)
                                            each.checked = valueChanged
                                        }
                                    }
                                    Connections{
                                        target: dashboardVM
                                        ignoreUnknownSignals: true
                                        onReturnAction:{
                                            if(action === "start_copy"){
                                                buttonCheck.enabled = false
                                                each.enabled = false
                                            }
                                            if(action === "finished_copy"){
                                                each.enabled = true
                                                buttonCheck.enabled = true
                                                each.checked = false
                                            }
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
                    fileDialog.title = "Save to folder"
                    fileDialog.selectExisting = true
                    fileDialog.open()
                    // startBtn.visible = false
                    // pauseBtn.visible = true
                    // cancelBtn.visible = true
                    // dashboardVM.setActionList(0)
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
                    dashboardVM.setActionList(1)
                }
            }
            Button{
                id: continueBtn
                width: parent.width / 2
                height: 50
                text:"Continue"
                visible: false
                onClicked: {
                    visible = false
                    pauseBtn.visible = true
                    dashboardVM.setActionList(2)
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
                    dashboardVM.setActionList(3)
                }
            }
            FileDialog {
                id: fileDialog
                width: 1000
                height: 600
                folder: "/media/"
                selectMultiple: false
                selectFolder: true
                onAccepted: {
                    var selectedUrl = fileDialog.fileUrl
                    if (selectedUrl.toString().indexOf("file:///media/") !== -1) {
                        var dir = selectedUrl.toString()
                        dashboardVM.setDirectory(dir);
                        startBtn.visible = false
                        pauseBtn.visible = true
                        cancelBtn.visible = true
                        dashboardVM.setActionList(0)
                    } else {
                        modalDialogBoxVM.getModalDialogQML(false, "not_allow_dir");
                    }
                }
            }
            Connections{
                target: dashboardVM
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
            onClicked: listDirPopup.close()
            Connections{
                target: dashboardVM
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

