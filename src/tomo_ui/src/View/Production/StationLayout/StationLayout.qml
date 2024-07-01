import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.0
import QtQml.Models 2.3
import QtQuick.Dialogs 1.2


Popup {
    id: stationLayout
    property string _sourceImage: ""
    property int _fontPixelSize : 20
    property string _fontFamily :  "Arial"
    property int _borderSize: 1
    property int _stationIndex: -1
    property string _stationTitle: ""
    property bool serverReady: false    
    // closePolicy: Popup.NoAutoClose
    // width: parent.width
    // height: width / 1920 * 1080

    onVisibleChanged: {
        if(systemLogVM.serverReady){
            if(visible){
                stationVM.startReceivePosValue(_stationIndex)
                stationVM.requestParmButtonState(_stationIndex)
                // stationVM.requestParmList(_stationIndex)
            }
            else {
                stationVM.startReceivePosValue(-1)
            }
        }
    }
    Connections{
        target: systemBarVM
        ignoreUnknownSignals: true
        onProcessStateChanged:{
            if(systemBarVM.processState === "Running"){
                if(stationLayout.visible)
                    stationLayout.visible = false
            }

        }
    }
    Rectangle{
        anchors.fill: parent
        color: "#424242"
        Loader{
            id: loadFunction
            width: parent.width / 2
            height: parent.height * 0.9
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            opacity: enabled? 1:0.5
            source: _stationIndex > 0 ? "FunctionLayout" + _stationIndex  + ".qml" : ""
            property bool readyToMove
            onLoaded: {
                loadFunction.item._stationIndex = _stationIndex
            }
        }
        Rectangle{
            id: station_image
            width: parent.width/2
            height: parent.height
            color: "white"
            border.width: 5
            Rectangle{
                anchors{
                    top: parent.top
                    left: parent.left
                    leftMargin: parent.border.width
                }
                height: implicitHeight
                width: implicitWidth
                color: "red"
                Button{
                    anchors{
                        top: parent.top
                        left: parent.left
                    }
                    width: implicitWidth
                    height: implicitHeight
                    Image {
                        anchors.fill: parent
                        source: "../imageStation/back.png"
                        fillMode: Image.PreserveAspectFit
                    }
                    onClicked: {
                        loginVM.updateIdleTimer()
                        stationLayout.visible = false
                    }
                }
            }
            Image{
                width: parent.width
                height: parent.height
                source: _sourceImage
                fillMode: Image.PreserveAspectFit
            }
        }
        Rectangle{
            id: titleRec
            width: parent.width/4
            height: parent.height * 0.1
            color: "transparent"
            border.color: "white"
            border.width: _borderSize
            anchors{
                top: parent.top
                right: parent.right
                topMargin: 40
                rightMargin: parent.width/8
            }
            Text {
                id: tilteStation
                text: _stationTitle
                font.pixelSize : _fontPixelSize
                font.family : _fontFamily
                color: "white"
                font.bold: true
                anchors.centerIn: parent
            }
        }
    }
}
