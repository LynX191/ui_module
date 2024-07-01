import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

import "../Component"
import "../Component/MaterialDesign"

import DataNotify 1.0
Popup{
    id:notifyHistoryPopup
    width: 1750
    height: 900
    modal: true
    background: Rectangle{
        color: "#232526"
        border.color: "gray"
        border.width: 2
    }
    ListModel{
        id: historyNotiHeader
        //total widthScale = 1
        ListElement{_info:"No."         ;widthScale:0.05}
        ListElement{_info:"Alarm Name"  ;widthScale:0.65}
        ListElement{_info:"Alarm Type"  ;widthScale:0.15}
        ListElement{_info:"Date & Time" ;widthScale:0.15}
    } 
    function addHistory(type, messID,content,time){
        if(type<(-1)){
            notifyVM.addNotifyToDatabase(type,
                                    messID,
                                    content,
                                    Qt.formatTime(time,"hh:mm:ss:zzz")+"\n"+Qt.formatDateTime(time, "dd/MM/yy"))
        }
    }
    Item{
        width: parent.width
        height: parent.height
        clip:true
        ListView{
            id: listNotiHistory
            anchors.fill: parent
            model: NotifyModel{
                list: notifyVM
            }
            headerPositioning: ListView.OverlayHeader
            ScrollBar.vertical: ScrollBar {id:vScrollbar;width: 16}
            delegate: Item{
                width: listNotiHistory.width-vScrollbar.width
                height: 60
                Row{
                    anchors.fill: parent
                    spacing: 0
                    Rectangle{
                        width: parent.width * historyNotiHeader.get(0).widthScale
                        height: parent.height
                        color: "black"
                        border{
                            width: 1
                            color: "gray"
                        }
                        Text{
                            anchors.fill: parent
                            font.pointSize: 13
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            text: index+1
                            color: "white"
                        }
                    }
                    Rectangle{
                        width: parent.width * historyNotiHeader.get(1).widthScale
                        height: parent.height
                        border.width: 1
                        border.color: "black"
                        color: index % 2 == 0 ? "#4c5052":"#36393b"
                        Text{
                            anchors.fill: parent
                            font.pointSize: 12
                            minimumPointSize: 10
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            textFormat: TextEdit.PlainText
                            wrapMode: TextEdit.Wrap
                            text: content
                            color: "white"
                            padding: 10
                            fontSizeMode:Text.VerticalFit
                        }
                    }
                    Rectangle{
                        width: parent.width * historyNotiHeader.get(2).widthScale
                        height: parent.height
                        border.width: 1
                        border.color: "black"
                        color: index % 2 == 0 ? "#4c5052":"#36393b"
                        Text{
                            anchors.fill: parent
                            font.pointSize: 12
                            minimumPointSize: 10
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            textFormat: TextEdit.PlainText
                            wrapMode: TextEdit.Wrap
                            text:{
                                switch(parseInt(type)) {
                                   case -1:
                                       return "Info"
                                   case -2:
                                       return "Warning"
                                   case -3:
                                       return "Error"
                                   default:
                                       return ""
                                }
                            }
                            color: {
                                switch(parseInt(type)) {
                                case -1:
                                    return "#659ffc"
                                case -2:
                                    return "yellow"
                                case -3:
                                    return "red"
                                default:
                                    return "white"
                                }
                            }
                            padding: 10
                            fontSizeMode:Text.VerticalFit
                        }
                    }
                    Rectangle{
                        width: parent.width * historyNotiHeader.get(3).widthScale
                        height: parent.height
                        border.width: 1
                        border.color: "black"
                        color: index % 2 == 0 ? "#4c5052":"#36393b"
                        Text{
                            anchors.fill: parent
                            font.pointSize: 12
                            minimumPointSize: 10
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            textFormat: TextEdit.PlainText
                            wrapMode: TextEdit.Wrap
                            text: time
                            color: "white"
                            padding: 10
                            fontSizeMode:Text.VerticalFit
                        }
                    }
                }
            }
            header: Row{
                width: listNotiHistory.width-vScrollbar.width
                height: 60
                z:5
                Repeater{
                    model: historyNotiHeader
                    delegate: Rectangle{
                        width: parent.width * widthScale
                        height: 60
                        color: "black"
                        border{
                            width: 1
                            color: "gray"
                        }
                        Text{
                            anchors.fill: parent
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            text: _info
                            wrapMode: Text.Wrap
                            font.pointSize: 14
                            color: "white"
                        }
                    }
                }
            }
        }
    }
}
