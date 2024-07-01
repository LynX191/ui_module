import QtQuick 2.9
import QtQuick.Window 2.3
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

import "../../Component"
Item{
    id: ioItem
    property int _fontPixelSize: 25
    property int _fontSubSize: 16
    property int _fontPointSize: 16
    property string _fontFamily:  "Arial"
    property int moduleNumber: 14
    property var outputData: ioControlVM.outputData
    property var inputData: ioControlVM.inputData
    property var outputEnable: ioControlVM.outputEnable
    property bool firstRunning: false

    width: parent.width
    height: parent.height
    enabled: (loginVM.ioControlEnabled && (( systemBarVM.processState === "Pause" || systemBarVM.processState === "Error" ) && ( systemBarVM.lotState === "Started" || systemBarVM.lotState === "Ending" )) && !productionVM.operationValue) || (loginVM.ioControlEnabled && productionVM.enabledForRunLot  && productionVM.fileValid && !mainWindow.isInitializing) || loginVM.superUserActive
    opacity: enabled? 1 : 0.5
    onVisibleChanged: {
        if(systemLogVM.serverReady && firstRunning){
            ioControlVM.changeThreadState(visible)
        }
    }
    Connections{
        target: systemLogVM
        ignoreUnknownSignals: true
        onServerReadyChanged: {
			ioControlVM.changeReceiveSignalMode(serverReady)
			stationVM.changeReceivePosValMode(serverReady)
            if(serverReady){
                ioControlVM.changeThreadState(true)
            }
        }
    }
    Connections{
        target: systemBarVM
        ignoreUnknownSignals: true
        onProcessStateChanged:{
            if(systemBarVM.processState === "Running" && !firstRunning){
                firstRunning = true
                ioControlVM.setCompletedIO(true)
                ioControlVM.changeThreadState(false)
            }
        }
    }
    Connections{
        target: master_app
        ignoreUnknownSignals: true
        onPowerUpStateChanged:{
            if(!state){
                firstRunning = false
                ioControlVM.setCompletedIO(false)
            }
        }
    }
    function getOutputValue(module, pin )
    {
        return outputData[(module - 1) * 16 + (pin - 1)]
    }
    function getInputValue(module, pin )
    {
        return inputData[(module - 1) * 16 + (pin - 1)]
    }
	function getOutputEnable(module, pin)
	{
        return outputEnable[(module - 1) * 16 + (pin - 1)]
	}
    Item{
        anchors.horizontalCenter: parent.horizontalCenter
        width: parent.width
        height: parent.height
        Column{
            id: ioColumn
            width: parent.width
            height: width / 1795 * 1080
            spacing: 0
            TabBar{
                id: tabIO
                width: parent.width
                height: 80
                currentIndex: swipeView.currentIndex
                anchors.horizontalCenter: parent.horizontalCenter
                Repeater{
                    model: moduleNumber
                    width: parent.width
                    height: parent.height
                    TabButton{
                        height: parent.height
                        Image {
                            anchors.fill: parent
                            source: "imageIOControl/Station"+(index)+".png"
                            fillMode: Image.PreserveAspectFit
                        }
                        onPressed: {
                            loginVM.updateIdleTimer()
                            swipeView.currentIndex = index
                        }
                    }
                }
                Connections{
                    target: stationVM
                    ignoreUnknownSignals: true
                    onSetTabIndexIO:{
                        swipeView.currentIndex = ioTabIndex
                    }
                }
            }
            Label {
                id: tilte
                text: typeof ioControlVM.titleMap === "undefined" ? "" : (typeof ioControlVM.titleMap[swipeView.currentIndex] === "undefined" ? "": ioControlVM.titleMap[swipeView.currentIndex])
                font.bold: true
                font.pixelSize: _fontPixelSize
                font.family: _fontFamily
                anchors.horizontalCenter: parent.horizontalCenter
            }
            PageIndicator {
                id: indicator
                count: swipeView.count
                currentIndex: swipeView.currentIndex
                anchors.horizontalCenter: parent.horizontalCenter
                delegate: Rectangle {
                    implicitWidth: 8
                    implicitHeight: 8
                    radius: width / 2
                    color: "#21be2b"
                    opacity: index === swipeView.currentIndex ? 0.95 : pressed ? 0.7 : 0.45
                    Behavior on opacity {
                        OpacityAnimator {
                            duration: 100
                        }
                    }
                }
            }
            Flickable{
                width: parent.width
                height: parent.height
                clip: true
                contentWidth: -1
                contentHeight: swipeView.height
                SwipeView{
                    id: swipeView
                    width: ioColumn.width
                    height: ioColumn.height - tilte.height - indicator.height - tabIO.height// - 50
                    anchors.horizontalCenter: parent.horizontalCenter
                    currentIndex: -1
                    clip: true
                    Repeater{
                        id: swipeRpt
                        model: moduleNumber
                        property var rowGroupIO:    [1,1,0,1,2,1,1,2,1,1,1,0,0,1]
                        property var rowSeparateIO: [4,4,5,4,3,4,4,3,4,4,4,5,5,4]
                        Loader {
                            active:true
                            sourceComponent: Item{
                                width: swipeView.width
                                height: swipeView.height
                                Column{
                                    width: parent.width
                                    height: parent.height
                                    spacing: 0
                                    Item{
                                        id: groupIOItem
                                        width: parent.width
                                        height: parent.height * 0.23 * swipeRpt.rowGroupIO[index]
                                        GridLayout {
                                            id: gridIO
                                            columns: 15
                                            rows: swipeRpt.rowGroupIO[index]
                                            width: parent.width
                                            height: parent.height
                                            rowSpacing: 0
                                            columnSpacing: 0
                                            anchors.top: parent.top
                                            Repeater {
                                                id: ioRepeater
                                                model: ioControlVM.ioModelMap[index.toString()]
                                                Layout.fillHeight: true
                                                Layout.fillWidth: true
                                                delegate: Item{
                                                    Layout.columnSpan: modelData.columnN
                                                    Layout.rowSpan: 1
                                                    Layout.preferredWidth:   gridIO.width   / gridIO.columns * Layout.columnSpan
                                                    Layout.preferredHeight:  gridIO.height  / gridIO.rows    * Layout.rowSpan
                                                    Layout.alignment : Qt.AlignCenter | Qt.AlignTop
                                                    Rectangle{
                                                        width: parent.width * (modelData.columnN * 0.025/3 + 0.925)
                                                        height: parent.height * 0.95
                                                        anchors.centerIn: parent
                                                        color: "transparent"
                                                        border{
                                                            width: 1
                                                            color: "white"
                                                        }
                                                        Rectangle{
                                                            width: parent.width - 2
                                                            height: parent.height / 4
                                                            anchors.horizontalCenter: parent.horizontalCenter
                                                            anchors.top: parent.top
                                                            anchors.topMargin: 2
                                                            color: "black"
                                                            Text{
                                                                text: modelData.name
                                                                anchors.centerIn: parent
                                                                font.pixelSize: _fontSubSize
                                                                font.family: _fontFamily
                                                                font.bold: true
                                                                color: "white"
                                                            }
                                                        }
                                                        Item{
                                                            width: parent.width
                                                            height: parent.height * 3/4
                                                            anchors.bottom: parent.bottom
                                                            anchors.horizontalCenter: parent.horizontalCenter
                                                            Row{
                                                                id: inputGroupRec
                                                                width: parent.width - 2
                                                                height: parent.height * 1/3
                                                                anchors.top: parent.top
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                                spacing: 0
                                                                Repeater{
                                                                    id: inputItemRpt
                                                                    model: ioControlVM.inputItemModelMap[modelData.orderState]
                                                                    delegate: Item{
                                                                        id: inputGroup
                                                                        property int moduleOrder: modelData.module
                                                                        property int pinOrder: modelData.pin
                                                                        property string colorState: getInputValue(moduleOrder,pinOrder) ? "#00ff26" : "#ffffff"
                                                                        width: parent.width / inputItemRpt.count
                                                                        height: parent.height
                                                                            Rectangle{
                                                                            width: parent.width
                                                                            height: parent.height
                                                                            anchors.centerIn: parent
                                                                            color: parent.colorState
                                                                            border{
                                                                                width: 1
                                                                                color: "black"
                                                                            }
                                                                            Text{
                                                                                width: parent.width
                                                                                height: parent.height
                                                                                anchors.centerIn: parent
                                                                                verticalAlignment: Text.AlignVCenter
                                                                                horizontalAlignment: Text.AlignHCenter
                                                                                font.pixelSize: _fontPixelSize
                                                                                font.family: _fontFamily
                                                                                color: "black"
                                                                                text: modelData.id
                                                                            }
                                                                        }

                                                                    }
                                                                }
                                                            }
                                                            Row{
                                                                width: parent.width - 2
                                                                height: parent.height  - inputGroupRec.height
                                                                anchors.bottom: parent.bottom
                                                                anchors.bottomMargin: 2
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                                spacing: 0
                                                                Repeater{
                                                                    id: outputItemRpt
                                                                    model: ioControlVM.outputItemModelMap[modelData.orderState]
                                                                    delegate: Rectangle{
                                                                        id: outputGroup
                                                                        property int moduleOrder: modelData.module
                                                                        property int pinOrder: modelData.pin
                                                                        property string idOutput: modelData.id
                                                                        property string colorState: getOutputValue(moduleOrder,pinOrder) ? "#00ff26" : "#ffffff"
																		property int getEnable: getOutputEnable(moduleOrder,pinOrder)

                                                                        width: parent.width / outputItemRpt.count
                                                                        height: parent.height
                                                                        color: "transparent"
                                                                        border{
                                                                            width: 1
                                                                            color: "black"
                                                                        }
                                                                        ButtonText{
                                                                            id: outputItenBtn
                                                                            anchors.centerIn: parent
                                                                            _width: parent.width * 0.95
                                                                            _height: parent.height * 0.85
                                                                            _textColor: "black"
                                                                            _textFont: _fontFamily
                                                                            _text: modelData.id +"\n" + modelData.name
                                                                            _sizeFont: _fontPointSize
                                                                            _btnColorDefault: parent.colorState
																			enabled: parent.getEnable
																			opacity: enabled? 1:0.5
                                                                            Component.onCompleted:{
                                                                                timerWaitCompleted.restart()
                                                                            }
                                                                            Connections{
                                                                                target: ioControlVM
                                                                                ignoreUnknownSignals: true
                                                                                onOutputDataChanged:{
                                                                                    timerWaitCompleted.restart()
                                                                                }
                                                                            }
                                                                            Timer{
                                                                                id: timerWaitCompleted
                                                                                interval: 50
                                                                                repeat: false
                                                                                onTriggered:{
                                                                                    if(outputItemRpt.count === 2){
                                                                                        if(outputItemRpt.itemAt(0).idOutput === outputItemRpt.itemAt(1).idOutput){
                                                                                            if(index === 1){
                                                                                                if(outputItemRpt.itemAt(0).colorState === "#00ff26")
                                                                                                    outputItemRpt.itemAt(1).colorState = "#ffffff"
                                                                                                else if(outputItemRpt.itemAt(0).colorState === "#ffffff")
                                                                                                    outputItemRpt.itemAt(1).colorState = "#00ff26"
                                                                                            }
                                                                                        }
                                                                                        else
                                                                                            return
                                                                                    }
                                                                                }
                                                                            }
                                                                            onPressed: {
                                                                                loginVM.updateIdleTimer()
                                                                                if(outputGroup.colorState === "#00ff26"){
                                                                                    state = 0
                                                                                    if(outputItemRpt.count === 2)
                                                                                        if(modelData.name === "Vacuum" || modelData.name === "Purge")
                                                                                            {}
                                                                                        else
                                                                                            return
                                                                                }
                                                                                else{
                                                                                    state = 1
                                                                                }
                                                                                if(outputItemRpt.count === 2){
                                                                                    if(outputItemRpt.itemAt(0).idOutput !== outputItemRpt.itemAt(1).idOutput){
                                                                                        if(index === 0){
                                                                                            var colorState1 = outputItemRpt.itemAt(1).colorState
                                                                                            if(colorState1 === "#00ff26"){
                                                                                                var moduleIndex1 = outputItemRpt.itemAt(1).moduleOrder;
                                                                                                var pinIndex1 = outputItemRpt.itemAt(1).pinOrder;
                                                                                                ioControlVM.sendOutput(moduleIndex1, pinIndex1 ,0)
                                                                                            }
                                                                                        }
                                                                                        else if(index === 1){
                                                                                            var colorState0 = outputItemRpt.itemAt(0).colorState
                                                                                            if( colorState0 === "#00ff26"){
                                                                                                var moduleIndex0 = outputItemRpt.itemAt(0).moduleOrder;
                                                                                                var pinIndex0 = outputItemRpt.itemAt(0).pinOrder;
                                                                                                ioControlVM.sendOutput(moduleIndex0, pinIndex0 ,0)
                                                                                            }
                                                                                        }
                                                                                    }
                                                                                    else{
                                                                                        ioControlVM.sendOutput(modelData.module, modelData.pin ,!index)
                                                                                        return
                                                                                    }
                                                                                }
                                                                                ioControlVM.sendOutput(modelData.module, modelData.pin ,state)
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    Row{
                                        width: parent.width
                                        height: parent.height - groupIOItem.height - 60
                                        spacing: 0

                                        Item{
                                            width: parent.width / 2
                                            height: parent.height
                                            GroupBox{
                                                width: parent.width
                                                height: parent.height
                                                anchors.centerIn: parent
                                            }
                                            GridLayout {
                                                id: gridO
                                                columns: 4
                                                rows: swipeRpt.rowSeparateIO[index]
                                                width: parent.width
                                                height: parent.height
                                                rowSpacing: 0
                                                columnSpacing: 0
                                                anchors.top: parent.top
                                                Repeater {
                                                    model: ioControlVM.outputModelMap[index.toString()]
                                                    Layout.fillHeight: true
                                                    Layout.fillWidth: true
                                                    delegate: Item{
                                                        Layout.columnSpan: 1
                                                        Layout.rowSpan: 1
                                                        Layout.preferredWidth:   gridO.width   / gridO.columns * Layout.columnSpan
                                                        Layout.preferredHeight:  gridO.height  / gridO.rows    * Layout.rowSpan
                                                        Layout.alignment : Qt.AlignCenter | Qt.AlignTop
                                                        Rectangle{
                                                            width: parent.width * ( 0.025/3 + 0.925)
                                                            height: parent.height * 0.95
                                                            anchors.centerIn: parent
                                                            color: "transparent"
                                                            border{
                                                                width: 1
                                                                color: "white"
                                                            }
                                                            Rectangle{
                                                                width: parent.width - 2
                                                                height: parent.height / 4
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                                anchors.top: parent.top
                                                                anchors.topMargin: 2
                                                                color: "black"
                                                                Text{
                                                                    text: modelData.id
                                                                    anchors.centerIn: parent
                                                                    font.pixelSize: _fontSubSize
                                                                    font.family: _fontFamily
                                                                    font.bold: true
                                                                    color: "white"
                                                                }
                                                            }
                                                            Item{
                                                                width: parent.width
                                                                height: parent.height * 3/4
                                                                anchors.bottom: parent.bottom
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                                Rectangle{
                                                                    id: outputOnly
                                                                    property int moduleOrder: modelData.module
                                                                    property int pinOrder: modelData.pin
                                                                    property string colorState: getOutputValue(moduleOrder,pinOrder) ? "#00ff26" : "#ffffff"
																	property int getEnable: getOutputEnable(moduleOrder,pinOrder)
                                                                    width: parent.width
                                                                    height: parent.height
                                                                    color: "transparent"
                                                                    ButtonText{
                                                                        anchors.centerIn: parent
                                                                        _width: parent.width * 0.9
                                                                        _height: parent.height * 0.85
                                                                        _textColor: "black"
                                                                        _textFont: _fontFamily
                                                                        _text: modelData.name
                                                                        _sizeFont: _fontSubSize
                                                                        _btnColorDefault: parent.colorState
																		enabled: parent.getEnable
																		opacity: enabled? 1:0.5
                                                                        onPressed: {
                                                                            loginVM.updateIdleTimer()
                                                                            if(parent.colorState === "#00ff26"){
                                                                                state = 0
                                                                            }
                                                                            else{
                                                                                state = 1
                                                                            }
                                                                            ioControlVM.sendOutput(modelData.module, modelData.pin ,state)
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        Item{
                                            width: parent.width / 2
                                            height: parent.height
                                            GroupBox{
                                                width: parent.width
                                                height: parent.height
                                                anchors.centerIn: parent
                                            }
                                            GridLayout {
                                                id: gridI
                                                columns: index === 7 ? 3 : 4
                                                rows: swipeRpt.rowSeparateIO[index]
                                                width: parent.width
                                                height: parent.height
                                                rowSpacing: 0
                                                columnSpacing: 0
                                                anchors.top: parent.top
                                                Repeater {
                                                    model: ioControlVM.inputModelMap[index.toString()]
                                                    Layout.fillHeight: true
                                                    Layout.fillWidth: true
                                                    delegate: Item{
                                                        Layout.columnSpan: 1
                                                        Layout.rowSpan: 1
                                                        Layout.preferredWidth:   gridI.width   / gridI.columns * Layout.columnSpan
                                                        Layout.preferredHeight:  gridI.height  / gridI.rows    * Layout.rowSpan
                                                        Layout.alignment : Qt.AlignCenter | Qt.AlignTop
                                                        Rectangle{
                                                            width: parent.width * ( 0.025/3 + 0.925)
                                                            height: parent.height * 0.95
                                                            anchors.centerIn: parent
                                                            color: "transparent"
                                                            border{
                                                                width: 1
                                                                color: "white"
                                                            }
                                                            Rectangle{
                                                                width: parent.width - 2
                                                                height: parent.height / 4
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                                anchors.top: parent.top
                                                                anchors.topMargin: 2
                                                                color: "black"
                                                                Text{
                                                                    anchors.centerIn: parent
                                                                    font.pixelSize: _fontSubSize
                                                                    font.family: _fontFamily
                                                                    font.bold: true
                                                                    color: "white"
                                                                    text: modelData.id
                                                                }
                                                            }
                                                            Item{
                                                                width: parent.width
                                                                height: parent.height * 3/4
                                                                anchors.bottom: parent.bottom
                                                                anchors.horizontalCenter: parent.horizontalCenter
                                                                ColumnLayout{
                                                                    anchors.fill: parent
                                                                    spacing: 0
                                                                    Rectangle{
                                                                        id: analogRec
                                                                        visible: modelData.id.slice(0, 2)==="AI"
                                                                        Layout.preferredWidth:parent.width-4
                                                                        Layout.preferredHeight:30
                                                                        Layout.alignment: Qt.AlignHCenter
                                                                        color: "#201e1f"

                                                                        Text {
                                                                            id: analogText
                                                                            // text: ioControlVM.inputAnalogMap[((modelData.module+1)*4 + (modelData.pin+1) ) * -1]
                                                                            font.pointSize: 20
                                                                            font.family: font7Seg.name
                                                                            font.bold: true
                                                                            font.letterSpacing: 4
                                                                            color: "#05ff0d"
                                                                            anchors.centerIn: parent
                                                                            FontLoader { id: font7Seg; source: "../../Fonts/Seven Segment.ttf" }
                                                                        }
                                                                        Connections{
                                                                            enabled: analogRec.visible
                                                                            target: ioControlVM
                                                                            ignoreUnknownSignals: true
                                                                            onInputAnalogMapChanged:{
                                                                                analogText.text = ioControlVM.inputAnalogMap[((modelData.module+1)*4 + (modelData.pin+1) ) * -1]
                                                                                if (parseInt(analogText.text) > 0) {
                                                                                    inputRec.color  = "#00ff26"
                                                                                    analogText.color = "#00ff26"
                                                                                }
                                                                                else if (parseInt(analogText.text) === 0) {
                                                                                    inputRec.color  = "#ffffff"
                                                                                    analogText.color = "#ffffff"
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                    Rectangle{
                                                                        property int moduleOrder: modelData.module
                                                                        property int pinOrder: modelData.pin
                                                                        property string colorState: getInputValue(moduleOrder,pinOrder) ? "#00ff26" : "#ffffff"
                                                                        Layout.fillWidth: true
                                                                        Layout.fillHeight: true
                                                                        color:"transparent"
                                                                        Rectangle{
                                                                            id: inputRec
                                                                            width: parent.width - 4
                                                                            height: parent.height - 4
                                                                            color: parent.colorState
                                                                            anchors.centerIn: parent
                                                                            Text{
                                                                                width: parent.width - 20
                                                                                height: parent.height
                                                                                anchors.centerIn: parent
                                                                                // font.pixelSize: _fontSubSize
                                                                                font.pointSize: _fontSubSize
                                                                                font.family: _fontFamily
                                                                                font.bold: true
                                                                                text: modelData.name
                                                                                wrapMode: Text.Wrap
                                                                                horizontalAlignment: Text.AlignHCenter
                                                                                verticalAlignment: Text.AlignVCenter
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}



