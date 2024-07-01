import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

import "../../Component"
import "../../Component/MaterialDesign"
Popup {
    id:listDialog
    visible: true
    x: mainWindow.x + mainWindow.width/2-width/2
    y: mainWindow.y + mainWindow.height/2-height/2
    width: 500
    height: 700
    background: Rectangle{
        color: "#202020"
    }
    property bool isList:false
    property string settingName:""

    onVisibleChanged: {
		if(visible)
			refreshList()
    }
    function refreshList(){
        settingList.clear()
        settingVM.scanLogFilesInFolder2()
    }
    Connections{
        target:settingVM
        ignoreUnknownSignals: true
        onSendFileName:{
            settingList.append({_settingName:name})
            if(!isList){
                isList=true
            }
        }
        onUpdateList:{
            listDialog.refreshList()
        }
    }
    Item {
        anchors.fill: parent
        focus: true
        Keys.onPressed: {
            loginVM.updateIdleTimer()
            if (event.key === Qt.Key_Escape) {
                listDialog.close();
            }
        }
    }
    Rectangle{
        id: dialogHeader
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        height: 40
        color: "#151617"
        Text {
            id: name
            anchors.fill: parent
            text: qsTr("Open")
            font.pointSize: 14
            color: "white"
            elide: Text.ElideRight
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
        }
    }
    ListModel{
        id:settingList
    }
    Item{
        anchors.top: dialogHeader.bottom
        anchors.bottom: dialogFooter.top
        anchors.left: parent.left
        anchors.right: parent.right
        clip: true

        Text {
            visible: !listDialog.isList
            text: "Empty"
            anchors.centerIn: parent
            color:"white"
            font.pointSize: 14
        }
        ListView{
            id:settingListView
            visible: isList
            anchors.fill: parent
            model:settingList
            ScrollBar.vertical: ScrollBar {width: 16}
            delegate: Button {
                id:histlogbtn
                width: parent.width-16
                height: 65
                Text{
                    text:_settingName
                    font.pointSize: 14
                    opacity: enabled ? 1.0 : 0.3
                    color: histlogbtn.down ? "gray" : "white"
                    anchors.top: parent.top
                    anchors.right:parent.right
                    anchors.topMargin: 20
                    anchors.rightMargin: 20
                }
                background: Rectangle {
                    id:btnBackground
                    anchors.fill:parent
                    opacity: enabled ? 1 : 0.3
                    color: histlogbtn.hovered ? histlogbtn.down ? "#26282e" : "#3b3b3b" : "#202020"
                    border.color: "#202020"
                    border.width: 1.5
                    radius: 7
                }
                onClicked:{
                    settingName=_settingName
                    applyDialog.open()
                }
                ButtonMaterial{
                    anchors.left:parent.left
                    anchors.leftMargin: 5
                    anchors.bottom: parent.bottom
                    anchors.bottomMargin: 5
                    _width: 30
                    _height: 30
                    _size: _height * 0.75
                    _iconSourceOn: "delete"
                    _iconSourceOff: ""
                    _enableEffect: true
                    _colorOverlayLow: "gray"
                    _colorOverlayHigh: "red"
                    onClicked: {
                        loginVM.updateIdleTimer()
                        settingName=_settingName
                        deleteDialog.open()
                    }
                }
            }
        }

    }


    Row{
        id: dialogFooter
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.leftMargin: spacing
        anchors.rightMargin: spacing
        height: 50
        spacing: 4
        Button{
            text: "Clear"
            height: parent.height
            width: (parent.width-parent.spacing)/2
            onClicked: {
                loginVM.updateIdleTimer()
                clearDialog.open()
            }
        }

        Button{
            text: "Close"
            height: parent.height
            width: (parent.width-parent.spacing)/2
            onClicked: {
                loginVM.updateIdleTimer()
                listDialog.close();
            }
        }
    }
    Dialog{
        id: applyDialog
        title: "Are you sure you want to recover the selected folder ?"
        width: 550
        height: 150
        x: parent.width / 2 - width / 2
        y: parent.height / 2 - height / 2
        standardButtons: Dialog.Yes | Dialog.Cancel
        modal: true
        onAccepted: {
            loginVM.updateIdleTimer()
            settingVM.restoreSavedConfig(settingName)
            listDialog.close();
        }
        onRejected:{
            loginVM.updateIdleTimer()
        }
    }
    Dialog{
        id: deleteDialog
        title: "Are you sure you want to remove this setting?"
        width: 475
        height: 150
        x: parent.width / 2 - width / 2
        y: parent.height / 2 - height / 2
        standardButtons: Dialog.Yes | Dialog.Cancel
        modal: true
        onAccepted: {
            loginVM.updateIdleTimer()
            settingVM.deleteFile(settingName)
			listDialog.refreshList()
        }
        onRejected:{
            loginVM.updateIdleTimer()
        }
    }
    Dialog {
        id: clearDialog
        title: "Are you sure you want to remove all saved setting?"
        width: 525
        height: 150
        x: parent.width / 2 - width / 2
        y: parent.height / 2 - height / 2
        standardButtons: Dialog.Yes | Dialog.Cancel
        modal: true
        onAccepted: {
            loginVM.updateIdleTimer()
            settingVM.clearAllBackup();
            listDialog.refreshList()
            isList=false
        }
        onRejected:{
            loginVM.updateIdleTimer()
        }
    }
}
