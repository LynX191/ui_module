import QtQuick 2.9
import QtQml 2.0
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Controls 2.2
import QtQuick.Window 2.3

//My include
import "../Component/MaterialDesign"
import "../Component"
//end

Popup {
    property int _typeDialog: -4  // -1:Info ; -2:Warm ; -3:Error
    property string _content: ""
    property string _detailContent: ""
    property bool _isFunctional: false
    property string _messageID: ""
    modal: true
    background: Rectangle {
        color: dialog.dynamicColor()
        border.color: "black"
    }

    id: dialog
    x: parent.x + parent.width / 2 - width / 2
    y: parent.y + parent.height / 2 - height / 2
    width: 500
    height: dialog.height=dialogIcon.height+dialogText.height+dialogBtn.height+dialogView.spacing*2+dialog.padding*2+4
    function sendNotify(){
        notifyView.addNotify(_typeDialog,_content,_detailContent)
    }

    function dynamicColor(){
        switch(_typeDialog) {
         case -1:
           return "#1976d2"
         case -2:
           return "#ffb300"
         case -3:
           return "red";
         default:
           return "transparent"
       }
    }

    Rectangle{
        id: dialogBg
        anchors.fill: parent
        color: "#232526"
        clip: true
        border.color: "gray"
        border.width: 1
        Column{
            id: dialogView
            anchors.top:parent.top
            anchors.topMargin: spacing
            anchors.left: parent.left
            width: parent.width
            spacing: 20
            Item{
                id: dialogIcon
                height: 100
                width: parent.width
                Rectangle{
                    anchors.centerIn: parent
                    width: 70
                    height: width
                    radius: width/2
                    color:{
                        switch(_typeDialog) {
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
                          switch(_typeDialog) {
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

                    color: dialog.dynamicColor()
                    size: 100
                    anchors.centerIn: parent
                }
            }
            Item{
                id: dialogText
                height:64
                width: parent.width
                Text {
                    id: contentText
                    text: _content
                    color:"white"
                    font.pointSize: 16
                    font.family: "Arial"
                    font.bold: true
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    anchors.fill: parent
                    wrapMode: Text.Wrap
                }
            }
            Item{
                id: detailItem
                width: parent.width
                height: detailItemText.implicitHeight+8
                visible: false
                Rectangle{
                    anchors.fill: parent
                    anchors.margins: 4
                    anchors.topMargin: 0
                    color: "black"
                    radius: 6
                    border.color: "gray"
                    border.width: 1
                    TextArea{
                        id: detailItemText
                        anchors.fill: parent
                        text: _detailContent
                        color: "yellow"
                        padding: 15
                        font.pixelSize: 16
                        font.family: "Arial"
                        font.bold: true
                        background: null
                        horizontalAlignment:Text.AlignLeft
                        wrapMode: TextEdit.Wrap
                        readOnly: true
                    }
                }
            }
        }
        Row{
            id: dialogBtn
            width: parent.width
            height: 50
            spacing: 4
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            anchors.margins: spacing
//            anchors.bottomMargin: 0
            property color _color: "#1F1F1F"
            ButtonText{
              height: parent.height
              width: (parent.width-parent.spacing)/2
              _text: _isFunctional? "Yes" :  (detailItem.visible? "Hide details <<": "More details >>")
              _btnColorClicked:"#4CAF50"
              _width: 100
              _sizeFont:11
              onClicked: {
                loginVM.updateIdleTimer()
                if(!_isFunctional){
                  detailItem.visible = !detailItem.visible
                  if(detailItem.visible){
                        dialog.height=dialogIcon.height+dialogText.height+dialogBtn.height+dialogView.spacing*3+detailItem.height+dialog.padding*2+4
                  }else
                        dialog.height=dialogIcon.height+dialogText.height+dialogBtn.height+dialogView.spacing*2+dialog.padding*2+4
                }
                else{
                  if(_messageID === "it_pick"){
                    ioControlVM.inputAccepted(_messageID)
                  }
                  else if(_messageID === "it_place"){
                    ioControlVM.inputAccepted(_messageID)
                  }
                  else{
                    master_app.confirmDialog(_messageID, true)
                  }
                  dashboardVM.isCovered = false
                  dialog.visible = false
                }
              }
            }
            ButtonText{
                height: parent.height
                width: (parent.width-parent.spacing)/2
                _text: _isFunctional? "No" : "OK"
                _btnColorClicked: "#1976d2"
                _width: 100
                _sizeFont:11
                onClicked: {
                  loginVM.updateIdleTimer()
                  if(_isFunctional){
                    master_app.confirmDialog(_messageID, false)
                  }
                  dashboardVM.isCovered = false
                  dialog.visible = false
                }
            }
        }
    }
    onAboutToHide: {
      dashboardVM.isCovered = false
      detailItem.visible = false
      dialog.height=dialogIcon.height+dialogText.height+dialogBtn.height+dialogView.spacing*2+dialog.padding*2+4
      dialog.visible = false
    }
}
