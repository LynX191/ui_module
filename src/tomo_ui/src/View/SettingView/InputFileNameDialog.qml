import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Window 2.3
import QtQuick.Layouts 1.3

import "../../View"
import "../../Component"
import "../../Component/MaterialDesign"
import "../../Dialog"

Popup {
    id:inputDialog
    width: 600
    height: 250
    property var listDrive: ["HOME"]
    property int driveCbBoxIndex: 0
    function replaceColonsWithHyphens(inputString) {
      var outputString = inputString.replace(/:/g, '-');
      return outputString;
    }
    onVisibleChanged: {
        if(visible){
            refreshFilelist()
			var dateName=new Date()
			fileNameInput.text= "default_folder"
        }
    }
    property var existedFile:[]
    function refreshFilelist(){
        existedFile=[]
        settingVM.scanLogFilesInFolder()
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
            text: qsTr("Save config as")
            font.pointSize: 14
            color: "white"
            elide: Text.ElideRight
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter

        }
    }

    Item{
        anchors.top: dialogHeader.bottom
        anchors.bottom: dialogFooter.top
        anchors.horizontalCenter: parent.horizontalCenter
        width: parent.width
        Row{
            id:inputArea
            anchors.centerIn: parent
            width: parent.width*0.95
            height: 40
            spacing: 15

            ComboBoxCustom{
                id:driveCombobox
                width: 200
                height: parent.height
                _model:listDrive
                _bgCombobox: "#303030"
                _bgPopup: "#141414"
                _opacityPopup: 1
                modelDataColor:"white"
                contentItem: Text{
                    text: parent.currentText
                    font.pointSize: 12
                    color: "white"
                    elide: Text.ElideRight
                    leftPadding: 15
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignLeft
                }
                onActivated: {
                    loginVM.updateIdleTimer()
                    settingVM.updateBackupPath(currentText)
                    inputDialog.refreshFilelist()
                }
            }
            FieldTextWithKeyboard{
                id: fileNameInput
                height: parent.height
                width: inputArea.width-driveCombobox.width-inputArea.spacing
                onlyNumber: false
                text: ""
                font.pixelSize: 18
                selectByMouse: true
                maximumLength: 30
                onTextChanged:{
                    text = text.replace(/[^a-zA-Z0-9_]/g, "")
                }
                Text {
                    id: warnInput
                    anchors.bottom: fileNameInput.top
                    anchors.horizontalCenter: parent.horizontalCenter
                    visible: fileNameInput.focus
                    width: parent.width
                    height: implicitHeight
                    textFormat: Text.RichText
                    wrapMode: Text.Wrap
                    text: (fileNameInput.text === '') ? "<p style='color:red;'><b>Please enter a name (1-30 characters)<br>Only alphanumeric characters and underscore</b></p>":"<p style='color:yellow;'><b>Name must be 1-30 characters<br>Allow only alphanumeric characters and underscore</b></p>"
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

        Button {
            id: saveBtn
            text: "Save"
            enabled: true
            height: parent.height
            width: (parent.width-parent.spacing)/2
			ToolTip.delay: 0
			ToolTip.timeout: 1000
			ToolTip.visible:  pressed && (fileNameInput.text === "")
			ToolTip.text: qsTr("The name folder can not empty !")
            onClicked: {
                loginVM.updateIdleTimer()
				if(fileNameInput.text === "")
					return
                for(var i=0;i<existedFile.length;i++){
                    if(fileNameInput.text===existedFile[i]){
                        overwriteDialog.open()
                        return;
                    }
                }
                settingVM.saveConfigAs(fileNameInput.text,false)
                inputDialog.visible = false
            }
        }
        Button{
            text: "Close"
            height: parent.height
            width: (parent.width-parent.spacing)/2
            onClicked: {
                loginVM.updateIdleTimer()
                inputDialog.close();
            }
        }
    }
    Connections{
        target:settingVM
        ignoreUnknownSignals: true
        onSendFileName:{
            existedFile.push(qsTr(name))
        }
        onUpdateList:{
            inputDialog.refreshFilelist()
        }
    }
    function refreshDrive(){
        driveCbBoxIndex=driveCombobox.currentIndex
        listDrive=["HOME"]
        settingVM.scanDrive()
    }
    Connections{
        target:settingVM
        ignoreUnknownSignals: true
        onSendDriveName:{
            listDrive.push(qsTr(name))
            driveCombobox._model=listDrive
        }
        onDriveListChanged:{
            driveCombobox._model=listDrive
            if(driveCbBoxIndex>listDrive.length-1){
                driveCombobox.currentIndex=0
            }else{
                driveCombobox.currentIndex=driveCbBoxIndex
            }
            settingVM.updateBackupPath(driveCombobox.currentText)

        }
    }
    Timer{
        interval: 1000
        repeat: true
        running: inputDialog.visible
        onTriggered: {
            refreshDrive()
        }
    }

    Dialog {
        id: overwriteDialog
        x: parent.width  / 2 - width  / 2
        y: parent.height / 2 - height / 2
        width: 300
        height: 120
        title: "Name existed!\nDo you want to overwrite it?"
        standardButtons: Dialog.Yes | Dialog.Cancel
        modal: true
        onAccepted: {
            loginVM.updateIdleTimer()
            settingVM.saveConfigAs(fileNameInput.text,true)
            inputDialog.visible = false
        }
        onRejected:{
            loginVM.updateIdleTimer()
        }
    }
}
