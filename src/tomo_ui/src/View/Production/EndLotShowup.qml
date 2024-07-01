import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.3

import "../../Component"
import "../../Component/MaterialDesign"

Popup {
    id: endlotConfirm
	// flags: Qt.ForeignWindow | Qt.FramelessWindowHint | Qt.BypassWindowManagerHint
    width: 720
    height: 450
	modal: true
	closePolicy: Popup.NoAutoClose

    // x: 600
    // y: (Screen.desktopAvailableHeight - height) / 2
    // color: "#202124"
    // background: Rectangle{
    //     color: "#202124"
    //     border.color: "gray"
    //     border.width: 2
    // }
    visible: false
    Column{
        width: parent.width
        height: implicitHeight
        anchors.centerIn: parent
        Item{
            width: parent.width
            height: 100
            Rectangle{
                width: parent.width / 2.5
                height: parent.height
                anchors.centerIn: parent
                color: "transparent"
                Rectangle{
                    width: parent.width
                    height:  parent.height
                    radius: 200
                    color: "red"
                    Text{
                        text: "WARNING"
                        width: parent.width
                        height: parent.height
                        anchors.centerIn: parent
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        color: "black"
                        font.pointSize: 40
                        font.family: "Arial"
                        font.bold: true
                    }
                }
            }
        }
        Item{
            width: parent.width
            height: 100
            Text{
                text: "End Current Lot"
                height: parent.height
                anchors.centerIn: parent
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                color: "red"
                font.pointSize: 22
                font.family: "Arial"
            }
        }
        Item{
            width: parent.width
            height: 100
            Row {
                width: parent.width
                height: parent.height
                Text{
                    text: "Current Model Selected"
                    width: parent.width / 2
                    height: parent.height
                    horizontalAlignment: Text.AlignRight
                    verticalAlignment: Text.AlignVCenter
                    rightPadding: 20
                    color: "white"
                    font.pointSize: 20
                    font.family: "Arial"
                }
                Item{
                    width: parent.width / 2
                    height: parent.height
                    Rectangle{
                        width: children[0].width * 1.15
                        height: parent.height / 3 + 10
                        color: "cyan"
                        anchors.left: parent.left
                        anchors.verticalCenter: parent.verticalCenter
                        Text{
                            text: productionVM.currentModel +" ("+productionVM.productName +")"
                            width: implicitWidth
                            height: parent.height
                            leftPadding: 20
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            color: "black"
                            font.pointSize: 20
                            font.family: "Arial"
                        }
                    }
                }
            }
        }
        Item{
            width: parent.width
            height: 100
            Row{
                width: parent.width
                height: parent.height
                anchors.centerIn: parent
                spacing: 0
                Item{
                    width: parent.width * 0.2
                    height: parent.height
                }
                Button{
                    width: parent.width * 0.25
                    height: parent.height
                    contentItem: Text{
                        text: "Confirm"
                        width: parent.width / 2
                        height: parent.height
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        color: "black"
                        font.pointSize: 26
                        font.family: "Arial"
                        font.bold: true
                    }
                    onClicked: {
                        loginVM.updateIdleTimer()
                        endlotConfirm.visible = false
						productionVM.endLotClicked(true)
                    }
                }
                Item{
                    width: parent.width * 0.1
                    height: parent.height
                }
                Button{
                    width: parent.width * 0.25
                    height: parent.height
                    opacity: enabled? 1:0.5
                    contentItem: Text{
                        text: "Cancel"
                        width: parent.width / 2
                        height: parent.height
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        color: "black"
                        font.pointSize: 26
                        font.family: "Arial"
                        font.bold: true
                    }
                    onClicked: {
                        loginVM.updateIdleTimer()
						productionVM.endLotClicked(false)
                        endlotConfirm.visible = false
                    }
                }
                Item{
                    width: parent.width / 5
                    height: parent.height
                }
            }
        }
    }
}
