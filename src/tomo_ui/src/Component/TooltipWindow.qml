import QtQuick 2.9
import QtQuick.Window 2.3

Window {
    id:tooltipWindow
    property bool _visible: false
    property string _textTooltip: ""
    visible: false
	flags: Qt.ForeignWindow | Qt.FramelessWindowHint | Qt.BypassWindowManagerHint
    width: tooltip.implicitWidth+tooltip.font.pointSize*2
    height: tooltip.implicitHeight+tooltip.font.pointSize*2
    // color: "transparent"
    opacity: 1
    on_VisibleChanged: {
        if(_visible){
            delayTooltip.restart()
        }
        else{
            delayTooltip.stop()
            tooltipWindow.visible=false
            tooltipWindow.opacity=0
        }
    }

    Behavior on opacity {NumberAnimation{duration:300}}
    Rectangle{
        anchors.fill: parent
        color:"#171717"
        // radius:10
        border.color: "gray"
        border.width: 1
//        gradient: Gradient {
//            GradientStop { position: 0.0; color: "#232526" }
//            GradientStop { position: 1.0; color: "#414345" }
//        }
    }
    Text {
        id:tooltip
        text: qsTr(_textTooltip)
        font.pointSize:10
        font.weight: Font.DemiBold
        font.family: "Adobe Gothic Std B"
        anchors.centerIn:parent
        color: "#B2C8DF"
    }
    Timer{
        id: delayTooltip
        interval: 200
        onTriggered: {
            tooltipWindow.visible=true
            tooltipWindow.opacity=1
        }
    }
}
