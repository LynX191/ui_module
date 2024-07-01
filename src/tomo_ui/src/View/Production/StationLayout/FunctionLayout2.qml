import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.0
import QtQml.Models 2.3
import QtQuick.Dialogs 1.2
import QtQuick.Controls.Styles 1.4

import "../../../Component"
import "../../../Component/MaterialDesign"

Item {
    id: stationLayout
    property string _sourceImage: ""
    property int _fontPixelSize: 20
    property string _fontFamily:  "Arial"
    property var backgroundColor: "#424242"
    property int _borderSize: 1
    property int _stationIndex: -1


    property int minLimitSpeed: stationVM.currentMinLimitSpeed
    property int maxLimitSpeed: stationVM.currentMaxLimitSpeed


    property int minLimitPosition: stationVM.currentMinLimitPosition
    property int maxLimitPosition: stationVM.currentMaxLimitPosition
    
    property var stateVector: stationVM.moveStateVector
    property bool _parmEnabled: stationVM.enabledParmButton
    property bool firstRunning
    Connections{
        target: systemBarVM
        ignoreUnknownSignals: true
        onProcessStateChanged:{
            if(systemBarVM.processState === "Running" && !firstRunning){
                firstRunning = true
            }
        }
    }
    Connections{
        target: master_app
        ignoreUnknownSignals: true
        onPowerUpStateChanged:{
            if(!state){
                firstRunning = false
            }
        }
    }
    function getReadyToUse(button, currentAxis){
        var state = stateVector[currentAxis]
        if(state === 0){
            return true
        }
        else if(state === 1){
            return false
        }
        else if(state === 2){
            if(button.indexOf("Stop") !== -1){
                return true
            }
            else{
                return false
            }
        }
    }


    property var myTableModel:[
        { _info: "TSTIM Z Axis - Current Position",   _value: stationVM.tsCurrentPosition }
    ]
    function commafy(num) {
        var str = num.toString().split('.');
        if (str[0].length >= 5) {
            str[0] = str[0].replace(/(\d)(?=(\d{3})+$)/g, '$1,');
        }
        if (str[1] && str[1].length >= 5) {
            str[1] = str[1].replace(/(\d{3})/g, '$1 ');
        }
        return str.join('.');
    }
    Column{
        width: parent.width
        height: parent.height
        spacing: 0
        visible: !parameterPopup.visible
        Item{ // Button control AFolding
            width: parent.width
            height: parent.height * 0.25
            Row{
                id: rowFBtn
                width: parent.width
                height: parent.height
                anchors.fill: parent
                property int btnWidth: width / 3
                property int btnHeight: height * 0.75
                Repeater{
                    // model: ["Home All","Stop All","Check Tray","Feed Tray","TSTIM I/O"]
                    model: ["TSTIM I/O", "Parameters"]
                    Item{
                        width: rowFBtn.btnWidth
                        height: rowFBtn.btnHeight
                        anchors.verticalCenter: rowFBtn.verticalCenter
                        Button{
                            text: modelData
                            width: parent.width * 0.95
                            height: parent.height * 0.5
                            anchors.centerIn: parent
                            visible: text === "Parameters" ? _parmEnabled : true
                            enabled: text === "Parameters" ? ((!parameterPopup.visible && (loginVM.parametersEnabled || loginVM.superUserActive) && productionVM.firstRunning && systemBarVM.processState !== "Running")) : productionVM.enabledForRunLot
                            opacity: enabled? 1:0.5
                            contentItem: Text{
                                anchors.fill: parent
                                text: parent.text
                                font.pixelSize: _fontPixelSize
                                font.family: _fontFamily
                                font.bold: true
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                                wrapMode: Text.Wrap
                                color: "white"
                            }
                            onClicked: {
                                loginVM.updateIdleTimer()
                                switch(index){
                                case 0: stationVM.goToIoTab(_stationIndex)
                                    break
                                case 1: {
                                        stationVM.requestParmList(_stationIndex)
                                        parameterPopup._currentStation = _stationIndex
                                        if(!parameterPopup.visible)
                                            parameterPopup.visible = true
                                    }
                                default:
                                    break
                                }
                            }
                        }
                    }
                }
            }
        }
        Item{
            width: parent.width
            height: parent.height * 0.75
            enabled: productionVM.enabledForRunLot // || loginVM.superUserActive
            opacity: enabled? 1 : 0.5
            Column{
                width: parent.width
                height: parent.height
                anchors.fill: parent
                spacing: 0
                GroupBox{
                    width: parent.width
                    height: parent.height * 1/8
                    Label {
                        x: parent.leftPadding + 10
                        y: -25
                        width: implicitWidth
                        text: "Axis Name"
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        elide: Text.ElideRight
                        Rectangle{
                            anchors.centerIn: parent
                            color: backgroundColor
                            height: parent.height
                            width: parent.width + 10
                            z:-1
                        }
                    }
                    ComboBox{
                        id: axisCbb
                        width: parent.width
                        height: parent.height
                        model: ["TSTIM Z Axis"]
                        enabled: false
                        contentItem: Text {
                            leftPadding: 20
                            rightPadding: parent.indicator.width + parent.spacing
                            text: parent.displayText
                            font.pixelSize: _fontPixelSize
                            color: "white"
                            verticalAlignment: Text.AlignVCenter
                            elide: Text.ElideMiddle
                        }
                        onActivated: {
                            loginVM.updateIdleTimer()
                        }
                    }
                }
                GroupBox{
                    width: parent.width
                    height: parent.height * 4/8
                    Column{
                        anchors.fill: parent
                        width: parent.width
                        height: parent.height
                        spacing: 0
                        Row{
                            id: rowABtn
                            width: parent.width
                            height: parent.height * 0.2
                            property int btnWidth: width / 2
                            property int btnHeight: height
                            Repeater{
                                model: ["Home","Stop"]
                                Item{
                                    width: rowABtn.btnWidth
                                    height: rowABtn.btnHeight
                                    anchors.verticalCenter: rowABtn.verticalCenter
                                    Button{
                                        text: modelData
                                        width: parent.width * 0.7
                                        height: parent.height
                                        enabled: getReadyToUse(text, 3)
                                        opacity: enabled? 1 : 0.5
                                        anchors.centerIn: parent
                                        contentItem: Text{
                                            anchors.fill: parent
                                            text: parent.text
                                            font.pixelSize: _fontPixelSize
                                            font.family: _fontFamily
                                            font.bold: true
                                            verticalAlignment: Text.AlignVCenter
                                            horizontalAlignment: Text.AlignHCenter
                                            wrapMode: Text.Wrap
                                            color: "white"
                                        }
                                        onClicked: {
                                            loginVM.updateIdleTimer()
                                            switch(index){
                                            case 0: if(systemLogVM.serverReady)  stationVM.tsHomeClicked()
                                                break
                                            case 1: if(systemLogVM.serverReady)  stationVM.tsStopClicked()
                                                break
                                            default:
                                                break
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        Column{
                            width: parent.width
                            height: parent.height * 0.4
                            Text{
                                font.pixelSize: _fontPixelSize
                                font.family: _fontFamily
                                color: "white"
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                                elide: Text.ElideMiddle
                                // anchors.top: parent.top
                                // anchors.topMargin: 15
                                anchors.horizontalCenter: parent.horizontalCenter
                                text: "Min Limit: "+minLimitPosition+"\n"+"Max Limit: "+maxLimitPosition +"\n Set speed: " + Math.round(speedSld.value) + " rpm"
                            }
                            Slider {
                                id: speedSld
                                anchors.horizontalCenter: parent.horizontalCenter
                                enabled: getReadyToUse("Move", 3)
                                opacity: enabled? 1 : 0.5
                                from: minLimitSpeed
                                value: stationVM.tsSpeedSlider
                                to: maxLimitSpeed
                                width: parent.width / 4
                                onMoved: {
                                    loginVM.updateIdleTimer()
                                    stationVM.tsSpeedSlider = value
                                }
                            }
                        }
                        Row{
                            width: parent.width
                            height: parent.height * 0.3
                            GroupBox{
                                width: parent.width / 2
                                height: parent.height
                                Label {
                                    x: parent.leftPadding + 10
                                    y: -25
                                     width: implicitWidth
                                    text: "Absolute Move"
                                    font.pixelSize: _fontPixelSize
                                    font.family: _fontFamily
                                    elide: Text.ElideRight
                                    Rectangle{
                                        anchors.centerIn: parent
                                        color: backgroundColor
                                        height: parent.height
                                        width: parent.width + 10
                                        z:-1
                                    }
                                }
                                Row{
                                    width: parent.width * 0.8
                                    height: parent.height
                                    anchors.centerIn: parent
                                    Item{
                                        width: parent.width / 2
                                        height:parent.height
                                        FieldTextWithKeyboard{
                                            onlyNumber: true
                                            id: absolutePosText
                                            width: parent.width
                                            height:parent.height / 1.5
                                            anchors.verticalCenter: parent.verticalCenter
                                            placeholderText: "Enter Position"
                                            readOnly: false
                                            font.pixelSize: _fontPixelSize
                                            font.family: _fontFamily
                                            wrapMode: Text.WordWrap
                                            color: "white"
                                            horizontalAlignment: Text.AlignHCenter
                                            verticalAlignment: Text.AlignVCenter
                                            validator: IntValidator { bottom: -100000; top: 100000 }
                                            _minimumValue: minLimitPosition
                                            _maximumValue: maxLimitPosition
                                            onTextChanged: {
                                                // if(parseInt(text) >= maxLimitPosition) text = maxLimitPosition
                                                // else if (parseInt(text) <= minLimitPosition) text = minLimitPosition
                                                stationVM.tsPosition = parseInt(text)
                                            }

                                        }
                                    }
                                    Text{
                                        text: "mm"
                                        width: parent.width / 6
                                        height: parent.height
                                        anchors.verticalCenter: parent.verticalCenter
                                        font.pixelSize: _fontPixelSize * 1.125
                                        font.family: _fontFamily
                                        color: "white"
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                    }
                                    Button{
                                        id:moveButton
                                        width: parent.width / 3
                                        height: parent.height * 0.9
                                        text: "Move"
                                        anchors.verticalCenter: parent.verticalCenter
                                        enabled: getReadyToUse("Move", 3)
                                        opacity: enabled? 1 : 0.5
                                        contentItem: Text{
                                            anchors.fill: parent
                                            text: parent.text
                                            font.pixelSize: _fontPixelSize
                                            font.family: _fontFamily
                                            font.bold: true
                                            verticalAlignment: Text.AlignVCenter
                                            horizontalAlignment: Text.AlignHCenter
                                            wrapMode: Text.Wrap
                                            color: "white"
                                        }
                                        onClicked: {
                                            loginVM.updateIdleTimer()
                                            if(systemLogVM.serverReady)  stationVM.tsMoveClicked()

                                        }
                                    }
                                }
                            }
                            GroupBox{
                                width: parent.width / 2
                                height: parent.height
                                Label {
                                    x: parent.leftPadding + 10
                                    y: -25
                                    width: implicitWidth
                                    text: "Relative Move"
                                    font.pixelSize: _fontPixelSize
                                    font.family: _fontFamily
                                    elide: Text.ElideRight
                                    Rectangle{
                                        anchors.centerIn: parent
                                        color: backgroundColor
                                        height: parent.height
                                        width: parent.width + 10
                                        z:-1
                                    }
                                }
                                Row{
                                    anchors.fill: parent
                                    property int btnWidth: width / 4
                                    property int btnHeight: height
                                    ButtonMaterial{
                                        _width: parent.btnWidth
                                        _height:parent.btnHeight
                                        _size: _width
                                        _iconSourceOff: "arrow-down-bold"
                                        _durationAnimation: 100
                                        _colorOverlayLow:"black"
                                        _colorMouseOver:"#696969"
                                        enabled: getReadyToUse("Move", 3)
                                        opacity: enabled? 1 : 0.5
                                        onClicked: {
                                            loginVM.updateIdleTimer()
                                            if(stationVM.tsCurrentPosition - stationVM.tsDistance < minLimitPosition){
	                                            stationVM.tsDistance = stationVM.tsCurrentPosition - minLimitPosition
                                                tsDistance.text = stationVM.tsDistance
                                            }
                                            if(systemLogVM.serverReady)  stationVM.tsBackwardClicked()
                                        }
                                    }
                                    Item{
                                        width: parent.btnWidth * 1.5
                                        height:parent.btnHeight
                                        FieldTextWithKeyboard{
                                            id: tsDistance
                                            onlyNumber: true
                                            width: parent.width
                                            height: 50
                                            anchors.centerIn: parent
                                            readOnly: false
                                            placeholderText: "Enter Distance"
                                            font.pixelSize: _fontPixelSize
                                            font.family: _fontFamily
                                            horizontalAlignment: Text.AlignHCenter
                                            verticalAlignment: Text.AlignVCenter
                                            validator: IntValidator { bottom: -100000; top: 100000 }
                                            _minimumValue: 0
                                            _maximumValue: maxLimitPosition - minLimitPosition
                                            onTextChanged: {
                                                if(parseInt(text) >= maxLimitPosition - minLimitPosition ) text = maxLimitPosition - minLimitPosition
                                                else if (parseInt(text) <= 0) text = 0
                                                stationVM.tsDistance = parseInt(text)
                                            }
                                        }
                                    }
                                    Text{
                                        width: parent.btnWidth * 0.5
                                        height:parent.btnHeight
                                        text: "mm"
                                        font.pixelSize: _fontPixelSize
                                        font.family: _fontFamily
                                        color: "white"
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                        elide: Text.ElideMiddle
                                    }
                                    ButtonMaterial{
                                        _width: parent.btnWidth
                                        _height:parent.btnHeight
                                        _size: _width
                                        _iconSourceOff: "arrow-up-bold"
                                        _durationAnimation: 100
                                        _colorOverlayLow:"black"
                                        _colorMouseOver:"#696969"
                                        enabled: getReadyToUse("Move", 3)
                                        opacity: enabled? 1 : 0.5
                                        onClicked: {
                                            loginVM.updateIdleTimer()
                                            if(stationVM.tsCurrentPosition + stationVM.tsDistance > maxLimitPosition){
	                                            stationVM.tsDistance = maxLimitPosition - stationVM.tsCurrentPosition
                                                tsDistance.text = stationVM.tsDistance
                                            }
                                            if(systemLogVM.serverReady)   stationVM.tsForwardClicked()
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                GroupBox{
                    width: parent.width
                    height: parent.height * 3/8
                    Label {
                        x: parent.leftPadding + 10
                        y: -23
                        width: implicitWidth
                        text: "Axis Information"
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        elide: Text.ElideRight
                        Rectangle{
                            anchors.centerIn: parent
                            color: backgroundColor
                            height: parent.height
                            width: parent.width + 10
                            z:-1
                        }
                    }
                    Item{
                        id: infoFItem
                        anchors.fill: parent
                        property int alignSpace: 20
                        Item{
                            width: parent.width
                            height: parent.height
                            ListView {
                                id: listShortcut
                                anchors.fill: parent
                                model: myTableModel
                                interactive: false
                                property var lineHeight: parent.height / 4
                                delegate: Rectangle {
                                    width: listShortcut.width
                                    height: listShortcut.lineHeight
                                    Row {
                                        anchors.fill: parent
                                        Rectangle{
                                            id: infoCol
                                            width: parent.width*0.6
                                            height: parent.height
                                            color: index % 2 === 0? "#4c5052":"#232526"
                                            border.color: "black"
                                            border.width: 0.5
                                            Text {
                                                anchors.fill: parent
                                                text: modelData._info
                                                color: "white"
                                                font.pointSize: 14
                                                verticalAlignment: Text.AlignVCenter
                                                horizontalAlignment: Text.AlignLeft
                                                leftPadding: 20
                                            }
                                        }
                                        Rectangle{
                                            id: valueCol
                                            width: parent.width-infoCol.width
                                            height: parent.height
                                            color:  index % 2 === 0? "#424547":"#151617"
                                            border.color: "black"
                                            border.width: 0.5
                                            Text {
                                                anchors.fill: parent
                                                text: stationLayout.commafy(modelData._value)+ " mm"
                                                color: "white"
                                                font.pointSize: 20
                                                verticalAlignment: Text.AlignVCenter
                                                horizontalAlignment: Text.AlignLeft
                                                leftPadding: 20
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
    Parameters{
        id: parameterPopup
        x: stationLayout.width  / 2 - width  / 2 
        y: 50
		z:  20
    }
}


