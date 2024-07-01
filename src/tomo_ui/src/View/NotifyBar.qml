import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

import "../Component"
import "../Component/MaterialDesign"
//import DataBarNotify 1.0

Rectangle{
    id: notifyBarAreaRoot
    clip: true
    color: "#202020"
    property string errorList

    function addNotify(_typeDialog,_messageID,_content,_time){

		// console.log("Inside Notify bar",_typeDialog,_messageID,_content,_time)

        if(_messageID === "reset") clearErrorMessage()
        var concat =  _content + "|"
        if(_typeDialog >= 0){
            for(var i = 0; i < notifyList.count ; i++){
                if( notifyList.get(i).messID === _messageID){
                    notifyView.addHistory(notifyList.get(i).type
                                          , notifyList.get(i).messID
                                          , notifyList.get(i).content
                                          , notifyList.get(i).time)
                    notifyList.remove(i,1)
                    i--
                }
            }
            errorList = errorList.replace(concat,"")
        }
        else{
            if(errorList.indexOf(concat) !== -1) {
                for(var messageFor = 0; messageFor < notifyList.count; messageFor++){
                    if(_messageID === notifyList.get(messageFor).messID){
                        notifyList.set(messageFor,{type:    _typeDialog,
                                        messID:   _messageID,
                                        content:  _content,
                                        time:     _time})
                    }
                }
            }
            else {
                notifyList.insert(0,{type:    _typeDialog,
                                messID:   _messageID,
                                content:  _content,
                                time:     _time})
                errorList = errorList + concat
                if (notifyList.count > 200) {
                    notifyList.remove(200, notifyList.count-200)
                }
            }
        }
        for(var messageModule = 0; messageModule < notifyList.count; messageModule++){
            modalDialogBoxVM.setModuleState(notifyList.get(messageModule).messID, notifyList.get(messageModule).type)
        }
        modalDialogBoxVM.checkModulesState()
    }
    function clearErrorMessage(){
        for(var i = 0; i < notifyList.count ; i++){
            if(notifyList.get(i).type === -3){
                productionVM.emergencyEvent(false, notifyList.get(i).messID)
                errorList = errorList.replace(notifyList.get(i).content + "|","")
                notifyView.addHistory( notifyList.get(i).type
                                      , notifyList.get(i).messID
                                      , notifyList.get(i).content
                                      , notifyList.get(i).time)
                notifyList.remove(i,1)
                i--
            }
        }
    }
    Connections{
        target: master_app
        ignoreUnknownSignals: true
        onPowerUpStateChanged:{
            for(var i = notifyList.count-1; i >= 0 ; i--){
                notifyView.addHistory( notifyList.get(i).type
                                      , notifyList.get(i).messID
                                      , notifyList.get(i).content
                                      , notifyList.get(i).time)
            }
            notifyList.clear()
            errorList = ""
            modalDialogBoxVM.checkModulesState()
            if(!state){
                productionVM.setEnabledForRunLot(true)
            }
        }
    }
    ListModel{
        id: notifyList
    }
    RowLayout{
        anchors.fill: parent
        spacing: 5

    ButtonText{
        id: notifyViewBtn
        Layout.fillHeight: true
        Layout.preferredWidth: 95
        opacity: enabled? 1:0.5
        _text: "Alarm\nHistory"
        _sizeFont: 11
        _btnColorDefault: "#ffc400"
        _btnColorMouseOver: "orange"
        _textColor: "black"
        onClicked: {
            loginVM.updateIdleTimer()
            notifyView.open()
        }
    }
    ListView{
        id: notifyLV
        clip:true
        Layout.fillHeight: true
        Layout.fillWidth: true
        orientation: ListView.Horizontal
        model: notifyList
        spacing: 4
        delegate: Button {
            id:notiLogBtn
            width: notifyLV.width/4-notifyLV.spacing
            height: parent.height
            Item{
                id: notiIcon
                width: 40
                height: parent.height
                Rectangle{
                    anchors.centerIn: parent
                    width: 20
                    height: width
                    radius: width/2
                    color:{
                        switch(parseInt(type)) {
                        case -1:
                            return "white"
                        case -2:
                            return "black"
                        case -3:
                            return "white"
                        default:
                            return "transparent"
                        }
                    }
                }
                MaterialDesignIcon {
                    name: {
                        switch(parseInt(type)) {
                        case -1:
                            return "alert-circle"
                        case -2:
                            return "alert-octagon"
                        case -3:
                            return "close-circle"
                        default:
                            return "alert-decagram"
                        }
                    }

                    color:{
                        switch(parseInt(type)) {
                        case -1:
                            return "#1976d2"
                        case -2:
                            return "yellow"
                        case -3:
                            return "red"
                        default:
                            return "transparent"
                        }
                    }
                    size: 32
                    anchors.centerIn: parent
                }
            }
            ColumnLayout{
                anchors.left:notiIcon.right
                anchors.right: parent.right
                anchors.top:parent.top
                anchors.bottom: parent.bottom
                spacing:0
                Item{
                    Layout.preferredWidth: parent.width
                    Layout.fillHeight: true
                    Text {
                        id: notiHeader
                        text:content
                        font.pointSize: 11
                        minimumPointSize: 9
                        color: "white"
                        textFormat: TextEdit.PlainText
                        wrapMode: TextEdit.Wrap
                        anchors.fill:parent
                        verticalAlignment :Text.AlignVCenter
                        horizontalAlignment: Text.AlignLeft
                        fontSizeMode:Text.VerticalFit
                        padding: 1
                    }
                }
                Item{
                    Layout.fillWidth: true
                    Layout.preferredHeight:16
                    Text{
                        text:Qt.formatDateTime(time, "dd/MM/yy ") + Qt.formatTime(time,"hh:mm:ss")
                        font.pointSize: 10
                        color: "gray"
                        padding: 3
                        elide: Text.ElideRight
                        width: parent.width
                        anchors.verticalCenter: parent.verticalCenter
                        anchors.right: parent.right
                        horizontalAlignment: Text.AlignRight
                    }
                }
            }
            background: Rectangle {
                id:btnBackground
                anchors.fill:parent
                opacity: enabled ? 1 : 0.3
                color: notiLogBtn.hovered ? notiLogBtn.down ? "#26282e" : "#3b3b3b" : "#2b2b2b"
                radius: 4
            }
            onClicked: {
                loginVM.updateIdleTimer()
            }
        }
    }
    }
}
