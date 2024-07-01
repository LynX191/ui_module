import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.3

import "../../Component"
import "../../Component/MaterialDesign"

Rectangle {
    id: endLotPopupRoot
    property int currentHeight: 700
    width: 400
    height: currentHeight
    visible: false
    color: "#232526"
    signal runStatusUpdate()
    property var runStatus: [0,0,0,0,0,0,0,0,0,0]
    property int currentListCount: 9
    property string endLotState
    property bool openOnceBSTIM: false
    property int _fontPixelSize: 25
    property int _fontSubSize: 16
    property int _fontPointSize: 16
    property string _fontFamily:  "Arial"
    property bool isOpened: false
    onVisibleChanged:{
        if(visible && !isOpened){
            listRunningState.clear()
            runStatus = [0,0,0,0,0,0,0,0,0,0]
            currentListCount = 9
            endLotPopupRoot.runStatusUpdate()
            checkLotEnded()
            checkRemainBSTIM()
            openOnceBSTIM = false
            okButton.visible = false
            currentHeight = 700
            for (var i = 0; i <= 8; i++) {
                listRunningState.append({_state: listEndlotState[i]});
            }
            counter = 0
            isOpened = true
        }
    }
    ListModel{
        id:listRunningState
    }

    property var listEndlotState:[  "S03. Input Tray Pick & Place"
                                    ,"S04. Automated Folding (Afolding)"
                                    ,"S06. Carton Loader"
                                    ,"S07. Carton Transfer"
                                    ,"S10a. Infeed Carton Conveyor"
                                    ,"S10b. Carton Sampling Conveyor"
                                    ,"S08. Tray Transfer & Placement"
                                    ,"S09. Output Tray Pick & Place"
                                    ,"S11. Output Tray Stack"]
    Connections{
        target: productionVM
        ignoreUnknownSignals: true
        onEndLotProcessConverted:{
            runStatus = endLotVar
            endLotPopupRoot.runStatusUpdate()
            checkRemainBSTIM()
            checkLotEnded()
        }
    }
    Connections{
        target: master_app
        ignoreUnknownSignals: true
        onPowerUpStateChanged:{
            if(!state){
                listRunningState.clear()
                endLotPopupRoot.visible = false
                isOpened = false
            }
        }
    }
    function checkGifVisible(){
        for(var i = 0; i < currentListCount; i++){
            if(typeof runStatus !== "undefined"){
                listRepeater.itemAt(i).children[0].children[1].visible = runStatus[i] === 1
            }
        }
    }
    function checkLotEnded(){
        for(var i = 0; i < currentListCount; i++){
            if(typeof runStatus !== "undefined"){
                if(runStatus[i] !== 1) {
                    endLotState = "Lot Ending !"
                    okButton.visible = false
                    return false
                }
            }
            else{
                endLotState = "Lot Ending !"
                okButton.visible = false
                return false
            }
        }
        if(openOnceBSTIM){
            okButton.visible = true
            endLotState = "End Lot Completed"
            return true
        }
    }
    function checkRemainBSTIM(){
        for(var i = 0; i < currentListCount; i++){
            if(typeof runStatus !== "undefined"){
                if(runStatus[i] !== 1)
                {
                    popupBSTIM.visible = false
                    return false
                }
            }
            else{
                popupBSTIM.visible = false
                return  false
            }
        }
        // if(!openOnceBSTIM)
            popupBSTIM.visible = true
        return true
    }

    property int counter: 0
    property string currentEndlotTimer: counter
    Timer{
        id: endlotTimer
        interval: 1000
        running: true
        repeat: true
        onTriggered:{
            counter++
        }
    }
    Connections{
        target: productionVM
        ignoreUnknownSignals: true
        onEndLotTimerAction:{
            // TIMER_START = 0,
            // TIMER_RESET,
            // TIMER_STOP,
            // TIMER_RESUME,
            switch(action){
                case 0: {
                    // console.log("TIMER_START")
                    endlotTimer.start()
                    break
                }
                case 1: {
                    // console.log("TIMER_RESET")
                    counter = 0;
                    endlotTimer.restart()
                    break
                }
                case 2: {
                    // console.log("TIMER_STOP")
                    endlotTimer.stop()
                    break
                }
                case 3: {
                    // console.log("TIMER_RESUME")
                    endlotTimer.start()
                    break
                }
            }
        }
    }

    Column{
        id: checklist
        width: parent.width
        height: currentHeight
        spacing: 0
        Item {
            width: parent.width
            height: 50
            Rectangle{
                anchors.centerIn: parent
                width: 50
                height: width
                radius: width/2
                color:"black"
            }
            MaterialDesignIcon {
                name: "alert-octagon"
                color: "yellow"
                size: 50
                anchors.centerIn: parent
            }

        }
        Item {
            width:parent.width
            height:50
            Text {
                text: endLotState
                anchors.centerIn: parent
                font.pointSize: 16
                color: "yellow"
            }
            Item{
                width: parent.width / 2 - parent.children[0].width / 2
                anchors.verticalCenter: parent.verticalCenter
                anchors.right: parent.right
                visible: false
                Text{
                    id: endlotTimerText
                    anchors.centerIn: parent
                    font.pointSize: 14
                    color: "white"
                    text: currentEndlotTimer
                }
            }
        }
        Item{
            width: parent.width
            height:600
            Grid{
                id: stateGrid
                columns: 1
                rows: Math.ceil(listRunningState.count)
                anchors.fill: parent
                spacing: 0
                flow: GridLayout.LeftToRight
                Repeater{
                    id: listRepeater
                    model:listRunningState
                    delegate: Row{
                        height: stateGrid.height / stateGrid.rows
                        width: stateGrid.width / stateGrid.columns
                        spacing: 20

                        Text{
                            id: endlotList
                            text: _state
                            font.pointSize: 12
                            color: "white"
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 20
                            visible: true
                        }

                        AnimatedImage {
                            id: gifImage
                            width:  20
                            height: 20
                            property bool checkEndElement
                            source: checkEndElement ? "../../Resources/check.gif" : "../../Resources/loading.gif" 
                            onCheckEndElement: {
                            }
                        }
                        Connections{
                            target: endLotPopupRoot
                            ignoreUnknownSignals: true
                            onRunStatusUpdate:{
                                if(typeof runStatus !== "undefined")
                                {
                                    endlotList.color = runStatus[index] === 1 ? "gray" :"white"
                                    gifImage.checkEndElement = runStatus[index] === 1
                                }
                            }
                        }
                    }
                }
            }
        }
        Item{
            width: parent.width
            height: 30
            visible: okButton.visible
            Text {
                text: qsTr("All Station Cleared !")
                anchors.centerIn: parent
                font.pointSize: 16
                color: "yellow"
            }
        }
    }
    //Popup BSTIM
    Popup {
        id: popupBSTIM
        width: 720
        height: buttonRec.height
        x: mainWindow.width  / 2 - width  / 2 - control_bar_layout.width
        y: mainWindow.height / 2 - height / 2 - notifyBarArea.height
		closePolicy: Popup.NoAutoClose
        visible: false
        // background: Rectangle {
        //     color: ffb300
        //     border.color: "black"
        // }
		onVisibleChanged:{
			if(visible && !openOnceBSTIM) bstmRow.visible = true
		}
		Rectangle{
            id: buttonRec
			anchors.centerIn: parent
            width: 720
			height: buttonColumn.height + 30
            color: "#202124"
            border.color: okButton.visible? "green":"#ffb300"
            border.width: 8
			Column{
            	id: buttonColumn
				width: parent.width
				height: implicitHeight
				Item{
					width: parent.width
					height: 175
					Rectangle{
						height: parent.height / 1.5
						width: parent.width / 1.25
						anchors.centerIn: parent
						color: "transparent"
						border.color: "white"
						border.width: 2
						Label{
							text: okButton.visible? "End Lot Completed" : "Choose BSTIM action"
							anchors.fill: parent
							font.pointSize: _fontSubSize
							font.family: _fontFamily
							horizontalAlignment: Text.AlignHCenter
							verticalAlignment: Text.AlignVCenter
						}
					}
				}
				Row{
					id: bstmRow
					width: parent.width
					height: 175
					Item {
						width: parent.width / 2
						height: parent.height * 0.85
						Button{
                            width: parent.width * 0.85
                            height: parent.height * 0.85
                            opacity: enabled? 1:0.5
							anchors.centerIn: parent
                            contentItem: Text{
                                text: "Purge"
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
								listRunningState.append({_state: "S01. Bottom Shipper Tray Input Module"})
								currentListCount = 10
								okButton.visible = false
								bstmRow.visible = false
								openOnceBSTIM = true
								productionVM.actionBSTIMChoosen(0)
								endLotPopupRoot.checkLotEnded()
								endLotPopupRoot.runStatusUpdate()
							}
						}
					}
					Item {
						width: parent.width / 2
						height: parent.height * 0.85
						Button{
                            width: parent.width * 0.85
                            height: parent.height * 0.85
                            opacity: enabled? 1:0.5
							anchors.centerIn: parent
                            contentItem: Text{
                                text: "Keep"
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
								productionVM.actionBSTIMChoosen(1)
								okButton.visible = true
								bstmRow.visible = false
								openOnceBSTIM = true
								endLotPopupRoot.checkLotEnded()
							}
						}
					}
				}
				Item{
					id: okButton
					width:parent.width
					height: 100
					Button{
                        width: parent.width/2
                        height: parent.height * 0.85
                        opacity: enabled? 1:0.5
                        anchors.centerIn: parent
                        contentItem: Text{
                            text: "Ok"
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
							productionVM.endLotConfirmClicked()
							okButton.visible = false
							endLotPopupRoot.visible = false
							isOpened = false
							popupBSTIM.visible = false
						}
						onVisibleChanged:{
							if(visible) currentHeight = 800
							else currentHeight = 700
						}
					}
				}
			}
		}
    }
}
