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
    property bool isFoldingPrepareOpen: false
    property bool isFoldOpen: false
    property bool isFoldFinishOpen: false
    property int afCurrentPosition

    property int minLimitSpeed: stationVM.currentMinLimitSpeed
    property int maxLimitSpeed: stationVM.currentMaxLimitSpeed

    property int minLimitPosition: stationVM.currentMinLimitPosition
    property int maxLimitPosition: stationVM.currentMaxLimitPosition
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
    property var stateVector: stationVM.moveStateVector

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
    
    function changeAfPoperty(){
        switch(axisCbb.currentIndex){
            case 0: 
                afCurrentPosition = stationVM.afXAxisPosition
                break
            case 1: 
                afCurrentPosition = stationVM.afYAxisPosition
                break
        }
    }
    Connections{
        target: stationVM
        ignoreUnknownSignals: true
        onAfXAxisPositionChanged:{
            stationLayout.changeAfPoperty()
        }
        onAfYAxisPositionChanged:{
            stationLayout.changeAfPoperty()
        }
    }

    property var myTableModel: [
        { _info: "AFolding X Axis - Current Position",   _value: stationVM.afXAxisPosition },
        { _info: "AFolding Y Axis - Current Position",   _value: stationVM.afYAxisPosition }
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
                property int btnWidth: width / 4
                property int btnHeight: height * 0.75
                Repeater{
                    // model: ["Home ALL","Stop All","Folding Prepare","Fold","Folding Finish"]
                    model: ["Home ALL","Stop All","Afolding I/O", "Parameters"]
                    Item{
                        width: rowFBtn.btnWidth
                        height: rowFBtn.btnHeight
                        anchors.verticalCenter: rowFBtn.verticalCenter
                        Button{
                            text: modelData
                            width: parent.width * 0.95
                            height: parent.height * 0.5
                            anchors.centerIn: parent
                            visible: text === "Parameters" ? _parmEnabled : getReadyToUse(text, axisCbb.currentIndex + 1)
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
                                    case 0: stationVM.afHomeAllClicked()
                                        break
                                    case 1: stationVM.afStopAllClicked()
                                        break
                                    // case 2: isFoldingPrepareOpen = !isFoldingPrepareOpen
                                    //     break
                                    // case 3: isFoldOpen = !isFoldOpen
                                    //     break
                                    // case 4: isFoldFinishOpen = !isFoldFinishOpen
                                    //     break
                                    case 2: stationVM.goToIoTab(_stationIndex)
                                        break
                                    case 3: {
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
                        model: ["AFolding X Axis","AFolding Y Axis"]
                        currentIndex: stationVM.afCurrentAxis - 1
                        enabled: getReadyToUse("Move", axisCbb.currentIndex + 1)
                        opacity: enabled? 1 : 0.5
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
                            switch(currentIndex){
                                case 0: 
                                    afCurrentPosition = stationVM.afXAxisPosition
                                    break
                                case 1: 
                                    afCurrentPosition = stationVM.afYAxisPosition
                                    break
                            }
                            loginVM.updateIdleTimer()
                            if(systemLogVM.serverReady) stationVM.afCurrentAxis = index + 1
                            absolutePosText.text = ""
                            relativeDisText.text = ""
                        }
                        onVisibleChanged: {
                            if(visible && systemLogVM.serverReady) stationVM.afCurrentAxis = currentIndex + 1
                            switch(currentIndex){
                                case 0: 
                                    afCurrentPosition = stationVM.afXAxisPosition
                                    break
                                case 1: 
                                    afCurrentPosition = stationVM.afYAxisPosition
                                    break
                            }
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
                            height: parent.height * 0.3
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
                                        anchors.centerIn: parent
                                        enabled: getReadyToUse(text, axisCbb.currentIndex + 1)
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
                                            switch(index){
                                                case 0: stationVM.afHomeClicked()
                                                    break
                                                case 1: stationVM.afStopClicked()
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
                                enabled: getReadyToUse("Move", axisCbb.currentIndex + 1)
                                opacity: enabled? 1 : 0.5
                                from: minLimitSpeed
                                value:{
                                    switch (axisCbb.currentIndex) {
                                    case 0:
                                        return stationVM.afSpeedSliderX
                                    case 1:
                                        return stationVM.afSpeedSliderY
                                    default:
                                        return;
                                    }
                                }
                                to: maxLimitSpeed
                                width: parent.width / 4
                                onMoved: {
                                    loginVM.updateIdleTimer()
                                    switch (axisCbb.currentIndex) {
                                    case 0:
                                        stationVM.afSpeedSliderX = value
                                        break;
                                    case 1:
                                        stationVM.afSpeedSliderY = value
                                        break;
                                    default:
                                        return;
                                    }
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
                                                stationVM.afPosition = parseInt(text)
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
                                        enabled: getReadyToUse("Move", axisCbb.currentIndex + 1)
                                        opacity: enabled? 1 : 0.5
                                        anchors.verticalCenter: parent.verticalCenter
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
                                        onClicked:  {
                                            loginVM.updateIdleTimer()
                                            stationVM.afMoveClicked()
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
                                        _iconSourceOff: "arrow-left-bold"
                                        _durationAnimation: 100
                                        _colorOverlayLow:"black"
                                        _colorMouseOver:"#696969"
                                        enabled: getReadyToUse("Move", axisCbb.currentIndex + 1)
                                        opacity: enabled? 1 : 0.5
                                        onClicked: {
                                            loginVM.updateIdleTimer()
                                            if(afCurrentPosition - stationVM.afDistance < minLimitPosition){
	                                            stationVM.afDistance = afCurrentPosition - minLimitPosition
                                                relativeDisText.text = stationVM.afDistance
                                            }
                                            stationVM.afBackwardClicked()
                                        }
                                    }
                                    Item{
                                        width: parent.btnWidth * 1.5
                                        height:parent.btnHeight
                                        FieldTextWithKeyboard{
                                            onlyNumber: true
                                            id: relativeDisText
                                            width: parent.width
                                            height:50
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
                                                stationVM.afDistance = parseInt(text)
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
                                        _iconSourceOff: "arrow-right-bold"
                                        _durationAnimation: 100
                                        _colorOverlayLow:"black"
                                        _colorMouseOver:"#696969"
                                        enabled: getReadyToUse("Move", axisCbb.currentIndex + 1)
                                        opacity: enabled? 1 : 0.5
                                        onClicked:  {
                                            loginVM.updateIdleTimer()
                                            if(afCurrentPosition + stationVM.afDistance > maxLimitPosition){
	                                            stationVM.afDistance = maxLimitPosition - afCurrentPosition
                                                relativeDisText.text = stationVM.afDistance
                                            }
                                            stationVM.afForwardClicked()
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
                                property var lineHeight: parent.height/4
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
                                                font.pixelSize: _fontPixelSize
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
                                                text: stationLayout.commafy(modelData._value) + " mm"
                                                color: "white"
                                                font.pixelSize: _fontPixelSize
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
        //Popup Folding Prepare
        Popup{
            width: 600
            height: 150
            x: parent.x - width / 2
            y: parent.y + height / 2
            visible: isFoldingPrepareOpen || isFoldOpen || isFoldFinishOpen? true : false
            onClosed: {
                isFoldingPrepareOpen = false
                isFoldOpen = false
                isFoldFinishOpen = false
            }
            modal: true
            Column{
                spacing: 10
                anchors.fill: parent
                Layout.fillHeight : true
                Row{
                    height: parent.height / 2
                    width: parent.width
                    Layout.alignment: Qt.AlignCenter
                    spacing: 15
                    Text{
                        width: parent.width / 2 - 10
                        height: parent.height
                        leftPadding: 20
                        text: "CHANGE TRAY TYPE"
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        verticalAlignment: Text.AlignVCenter
                    }
                    ComboBox {
                        id: foldingCbb
                        property var trayTypeModel: ["NO TRAY","TRAY TOP","TRAY BASE","TRAY BASE SMALL","TRAY BASE BIG"]
                        width: parent.width / 2 - 10
                        height: parent.height
                        model: trayTypeModel
                        delegate: ItemDelegate {
                            width: foldingCbb.width
                            height: foldingCbb.height
                            contentItem: Text {
                                text: modelData
                                color: "black"
                                font.pixelSize: _fontPixelSize - 2
                                font.family: _fontFamily
                                elide: Text.ElideRight
                                verticalAlignment: Text.AlignVCenter
                            }
                            highlighted: foldingCbb.highlightedIndex === index
                        }
                        contentItem: Text {
                            width: foldingCbb.width
                            height: foldingCbb.height
                            leftPadding: 5
                            rightPadding: foldingCbb.indicator.width + foldingCbb.spacing
                            text: foldingCbb.displayText
                            font.pixelSize: _fontPixelSize - 2
                            font.family: _fontFamily
                            color: "#17a81a"
                            verticalAlignment: Text.AlignVCenter
                            elide: Text.ElideRight
                        }
                        onCurrentTextChanged: {
                            displayText = currentText
                            stationVM.afTrayType = currentIndex
                        }
                        onActivated: {
                            loginVM.updateIdleTimer()
                        }
                    }
                }
                Item{
                    width: parent.width
                    height: parent.height / 2
                    Button{
                        width: parent.width / 2 - 20
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: "Confirm"
                        contentItem: Text {
                            text: parent.text
                            font.pixelSize: _fontPixelSize
                            font.family: _fontFamily
                            opacity: enabled ? 1.0 : 0.3
                            color: parent.down ? "#17a81a" : "#21be2b"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            elide: Text.ElideRight
                        }
                        onClicked: {
                            loginVM.updateIdleTimer()
                            if(isFoldingPrepareOpen)
                                stationVM.afPrepareClicked()
                            else if(isFoldOpen)
                                stationVM.afFoldClicked()
                            else if(isFoldFinishOpen)
                                stationVM.afFoldFinishClicked()
                            isFoldingPrepareOpen = false
                            isFoldOpen = false
                            isFoldFinishOpen = false
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


