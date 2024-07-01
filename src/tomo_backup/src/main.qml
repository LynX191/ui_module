import QtQuick 2.9
import QtQuick.Window 2.2
import "Dialog"

Window {
    visible: true
    width: 520
    height: 360
    flags: Qt.SplashScreen | Qt.WindowStaysOnTopHint
    x: Screen.desktopAvailableWidth / 2 - width / 2
    y: Screen.desktopAvailableHeight / 2 - height / 2
    color: "#000000"
    id: window
    Rectangle{
        anchors.fill: parent
        color: "transparent"
        border{
            width: 5
            color: "gray"
        }

        Text{
            text: "Recovering..."
            font.bold: true
            font.pixelSize: 36
            color: "cyan"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
        }

        AnimatedImage {
            id: gifImage
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 10
            height: 256
            width: 256
            source: ":/../tomo_ui/src/Resources/loading.gif"
            onStatusChanged: playing = (status == AnimatedImage.Ready)
        }
		ModalDialogBox{
			id:modalDialogPopup
		}
        Connections{
            target: recoveryVM
            ignoreUnknownSignals: true
            onPopupModalDialog:{
                gifImage.playing = false
                var splitResult = content.split(" - ")
                modalDialogPopup._typeDialog = type
				modalDialogPopup._content = splitResult[0]
				modalDialogPopup._detailContent = splitResult[1]
                modalDialogPopup._messageID= messageID
                modalDialogPopup._isFunctional= isFunctional
				modalDialogPopup.open()
            }
        }
    }
}
