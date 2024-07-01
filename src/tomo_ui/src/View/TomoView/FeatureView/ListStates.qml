import QtQml 2.0
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

//My import
import "../../../Component/MaterialDesign"
//end

Frame {
    property QtObject _viewModel: QtObject{}
    id: item
    Column{
        width : parent.width
        height: parent.height
        clip: true


        ListView {
            id: listView
            property int presentCheckedIndex: -1
            property int delegateHeight: Math.round(listView.pointSizeText*1.5)
            property int pointSizeText: 12
            Rectangle{
                width: Math.round(title.contentWidth*1.5)
                height: title.contentHeight
                color: "#90CAF9"
                radius: 5
                z: 2
                anchors.right: parent.right
                anchors.rightMargin: 10
                Text{
                    id: title
                    text: "States"
                    color: "black"
                    font.pointSize:12
                    font.family:"Adobe Gothic Std B"
                    verticalAlignment:Text.AlignVCenter
                    horizontalAlignment:Text.AlignHLeft
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                }
            }
            headerPositioning:ListView.OverlayHeader
            width : parent.width
            height: parent.height-headerBar.height
            model: _viewModel === null ? null : _viewModel.model
            clip: true
            highlightFollowsCurrentItem: true
            focus: true
            ScrollBar.vertical: ScrollBar {
                id: stateScrollBar
                height: 30
                width: 10
                active: true
                policy: ScrollBar.AsNeeded
                snapMode:ScrollBar.SnapOnRelease
                focusPolicy: Qt.WheelFocus
                Behavior on position {
                    NumberAnimation {
                        duration: 1000
                    }
                }
                contentItem: Rectangle {
                    radius: implicitWidth / 2
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: "#00d2ff" }
                        GradientStop { position: 1.0; color: "#3a7bd5" }
                    }
                }
            }
            delegate: MouseArea{
                width: parent.width-stateScrollBar.width
                height: listView.delegateHeight
                hoverEnabled: true

                Rectangle{
                    id: buttonBackground
                    color: "#303030"
                    width: parent.width
                    height: buttonText.contentHeight
                    radius: height/4
                    anchors.left: parent.left

                    Text{
                        id: buttonText
                        text: name
                        anchors.left: parent.left
                        anchors.leftMargin: 10
                        width: parent.width
                        anchors.verticalCenter: parent.verticalCenter
                        font.pointSize:listView.pointSizeText
                        font.family: "Meiryo UI Regular"
                        verticalAlignment: Text.AlignVCenter
                        color: "gray"
                        elide: Text.ElideRight
                        property bool isSelected: false
                    }
                    ColorAnimation on color {
                        id: aminationColor
                    }
                    ToolTip{
                        id:listViewToolTip
                        x:parent.x
                        y:parent.y-height
                        text:name
                        visible: false
                        delay:500
                        contentItem: Text{
                            color: "#B2C8DF"
                            font.pointSize:10
                            font.family: "Adobe Gothic Std B"
                            text:listViewToolTip.text
                        }
                        background: Rectangle {
                            border.color: "#e0e0e0"
                            radius: 10
                            gradient: Gradient {
                                GradientStop { position: 0.0; color: "#232526" }
                                GradientStop { position: 1.0; color: "#414345" }
                            }
                        }
                    }

                }

                onPressed: {
                    aminationColor.stop()
                    _viewModel.stateUserChoiceSlots(index)
                    buttonText.color = "white"
                    buttonText.isSelected = true
                    buttonBackground.color = "gray"
                    listView.presentCheckedIndex = index
                }

                onEntered: {
                    forceActiveFocus(Qt.MouseFocusReason)
                    if (listView.presentCheckedIndex === index){
                        return
                    }
                    aminationColor.stop()
                    aminationColor.from = "#303030"
                    aminationColor.to = "#4a4a4a"
                    aminationColor.duration = 300
                    aminationColor.start()

                    if(buttonText.contentWidth+30>parent.width){
                        listViewToolTip.visible=true
                    }

                }

                onExited: {
                    if (listView.presentCheckedIndex === index){
                        return
                    }
                    aminationColor.stop()
                    aminationColor.from = "#4a4a4a"
                    aminationColor.to = "#303030"
                    aminationColor.duration = 500
                    aminationColor.start()

                    listViewToolTip.visible=false

                }
                Connections{
                    target: listView
                    ignoreUnknownSignals: true
                    onPresentCheckedIndexChanged:{
                        if ((index !== listView.presentCheckedIndex) && buttonText.isSelected){
                            buttonText.color = "gray"
                            buttonText.isSelected = false
                            aminationColor.stop()
                            aminationColor.from = "#4a4a4a"
                            aminationColor.to = "#303030"
                            aminationColor.duration = 1000
                            aminationColor.start()
                        }
                    }
                }
            }

            Connections{
                target: _viewModel
                ignoreUnknownSignals: true
                onInitStateChecked:{
                }
            }

            function resetHeight(){
                if (parent.height > delegateHeight*listView.count){
                    height = delegateHeight*listView.count
                    interactive = false
                }
                else {
                    height = parent.height
                    interactive = true
                }
            }

            onCountChanged: {
                resetHeight()
            }
        }
        onHeightChanged: {
            listView.resetHeight()
        }
    }
}
