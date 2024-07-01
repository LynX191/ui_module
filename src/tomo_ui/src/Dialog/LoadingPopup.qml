import QtQuick 2.9
import QtQml 2.0
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Controls 2.2
import QtQuick.Window 2.3


Popup {
	id: dialog
    property string _text: ""
	x: Screen.width / 2 - width / 2
    y: Screen.height / 2 - height / 2
    width: 520
    height: 400
    closePolicy: Popup.NoAutoClose
	modal: true
    background: Rectangle {
        color: "#000000"
        border.color: "black"
    }
    Rectangle{
        anchors.fill: parent
        color: "transparent"
        border.color: "gray"
        border.width: 1
		Text{
            text: _text
            font.bold: true
            font.pixelSize: 30
            color: "cyan"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
        }

        AnimatedImage {
            id: gifImage
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 20
            height: 256
            width: height
            source: "../Resources/loading.gif"
			onStatusChanged: playing = (status == AnimatedImage.Ready)
        }
	}
}
