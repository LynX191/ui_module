import QtQml.Models 2.3
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.0

import "../../../Component"
import "../../../Component/MaterialDesign"

Popup {
    id: parmRoot
    property string _sourceImage: ""
    property int _fontPixelSize : 20
    property string _fontFamily :  "Arial"
    property int _borderSize: 1
    property string _stationTitle: ""
    property bool serverReady: false
    property int _currentStation
    property var _parameters: stationVM.parametersList
    property string _positionParm
    property bool _haveAxis: false
    property int expandValue: 0
    property bool acceptToClose: false
    property string uiParmList
    closePolicy: Popup.NoAutoClose
    modal: true
    visible: false
    width: 850
    padding: 5
    signal expandAll(bool value)
    signal revertClicked()
    signal resetClicked()
    enter: null
    exit: null
    Connections{
        target: loginVM
        ignoreUnknownSignals: true
        onAutoLogoutCheck:{
            if(parmRoot.visible)
                parmRoot.visible = false
        }
    }
    onVisibleChanged: {
        if(!visible){
            expandValue = 0
        }
        else {
            if(acceptToClose){
                expandModel(_parameters)
                acceptToClose = false
            }
            timer100.restart()
        }
    }
    Connections{
        target: systemBarVM
        ignoreUnknownSignals: true
        onProcessStateChanged:{
            if(systemBarVM.processState === "Running"){
                if(parmRoot.visible)
                    parmRoot.visible = false
            }

        }
    }
    Connections{
        target: master_app
        ignoreUnknownSignals: true
        onParmClose:{
            acceptToClose = true
            columnParm.haveChange = false
            parmRoot.close()
        }
    }
    Timer{
        id: timer100
        interval: 100
        onTriggered:{
            expandAllBtn.clicked()
        }
    }
    Connections{
        ignoreUnknownSignals: true
        target: stationVM
        onParametersListChanged:{
            if(parmRoot.visible)
                parmRoot.expandModel(_parameters)
        }
    }
    Component {
        id: someComponent
        ListModel {
        }
    }
    function checkBeforeClose(){
        loginVM.updateIdleTimer()
        if(columnParm.haveChange){
            modalDialogBoxVM.getModalDialogQML(true,"parm_close")
        }
        else{
            parmRoot.close()
        }
    }
    function createModel(parent) {
        var newModel = someComponent.createObject(parent);
        return newModel;
    }

    function expandModel(input){
        parmList.clear()
        sectionList.clear()
        var currentSection = 0
        var currectSectionList = ""
        for(var i = 0; i < input.length; i++){
            var splitList = input[i].split(";")
            var parmSection = splitList[0]
            var parmName    = splitList[1]
            var dataType    = splitList[2]
            var unit        = splitList[3]
            var defaultVal  = splitList[4]
            var minLimit    = splitList[5]
            var maxLimit    = splitList[6]
            var currentValue    = splitList[7]
			var parmSectionDelimited = "[" + parmSection + "]"
            if(currectSectionList.indexOf(parmSectionDelimited) === -1)
            {
                sectionList.append({nameSection: parmSection})
                currectSectionList = currectSectionList + parmSectionDelimited
            }
            parmList.append({parmSection: parmSection, parmName: parmName, dataType: dataType , unit: unit , defaultVal: defaultVal , minLimit: minLimit , maxLimit: maxLimit , uiValue: defaultVal , currentValue: typeof currentValue !== "undefined" ? currentValue : defaultVal})
        }
        
        if(parmList.get(0).parmName === "Enable Folding Inspection"){
            for(var i = 0; i < 5; i++){
                controlBarVM.inspecVector[i] = parseInt(parmList.get(i).currentValue)
            }
            controlBarVM.inspecVectorChanged()
        }

        var tempValue = ""
        for(var i = 0; i < parmList.count; i++){
            var value = "^" + parmList.get(i).uiValue
            tempValue += value
        }
        uiParmList = tempValue
        checkChange()
        expandAllBtn.clicked()
    }
    function checkChange(){
        var tempValue = ""
        for(var i = 0; i < parmList.count; i++){
            var value = "^" + parmList.get(i).uiValue
            tempValue += value
        }
        columnParm.haveChange = uiParmList !== tempValue
    }
    ListModel{
        id: parmList
    }
    ListModel{
        id: sectionList
    }

    Column{
        anchors.fill: parent
        Rectangle{
            width: parent.width
            height: columnParm.height
            clip: true
            color: "transparent"
            Column{
                id: columnParm
                property bool haveChange: false
                width: parent.width - parent.border.width * 2
                anchors.centerIn: parent
                clip: true
                Rectangle{
                    id: applyArea
                    width: columnParm.width
                    height: 50
                    color: "transparent"
                    Row{
                        width: parent.width
                        Item{
                            width: 200
                            height: 50
                            anchors.verticalCenter: parent.verticalCenter
                            Label {
                                id: tilte
                                text: "Parameters"
                                anchors.fill: parent
                                font.bold: true
                                font.pixelSize: _fontPixelSize
                                font.family: _fontFamily
                                anchors.horizontalCenter: parent.horizontalCenter
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                leftPadding: 20
                            }
                        }
                        Item{
                            width: 200
                            height: 50
                            anchors.verticalCenter: parent.verticalCenter
                            Row{
                                anchors.centerIn: parent
                                Item{
                                    width: 33.3
                                    height: 50
                                }
                                ButtonMaterial{
                                    id: expandAllBtn
                                    _width: 50
                                    _height: 50
                                    _size: 50
                                    _iconSourceOn: "arrow-expand-down"
                                    _iconSourceOff: ""
                                    _enableEffect: false
                                    opacity: enabled? 1:0.5
                                    anchors.verticalCenter: parent.verticalCenter
                                    Popup{
                                        visible: expandAllBtn.hovered && parmRoot.visible
                                        x: (expandAllBtn.width - width) / 2
                                        y: -50
                                        z: 100
                                        width: 90
                                        height: 30
                                        padding: 0
                                        Rectangle{
                                            anchors.fill: parent
                                            color: "gray"
                                            border{
                                                width: 2
                                                color: "black"
                                            }
                                            Text{
                                                anchors.fill: parent
                                                text: "Expand"
                                                color: "white"
                                                font.pointSize: 12
                                                font.family: _fontFamily
                                                verticalAlignment: Text.AlignVCenter
                                                horizontalAlignment: Text.AlignHCenter
                                            }
                                        }
                                    }
                                    onClicked: {
                                        loginVM.updateIdleTimer()
                                        expandValue = sectionList.count
                                        expandAll(true)
                                    }
                                }
                                Item{
                                    width: 33.3
                                    height: 50
                                }
                                ButtonMaterial{
                                    id: collapseAllBtn
                                    _width: 50
                                    _height: 50
                                    _size: 50
                                    _iconSourceOn: "arrow-expand-up"
                                    _iconSourceOff: ""
                                    _enableEffect: false
                                    opacity: enabled? 1:0.5
                                    anchors.verticalCenter: parent.verticalCenter
                                    Popup{
                                        visible: collapseAllBtn.hovered && parmRoot.visible
                                        x: (collapseAllBtn.width - width) / 2
                                        y: -50
                                        z: 100
                                        width: 90
                                        height: 30
                                        padding: 0
                                        Rectangle{
                                            anchors.fill: parent
                                            color: "gray"
                                            border{
                                                width: 2
                                                color: "black"
                                            }
                                            Text{
                                                anchors.fill: parent
                                                text: "Collapse"
                                                color: "white"
                                                font.pointSize: 12
                                                font.family: _fontFamily
                                                verticalAlignment: Text.AlignVCenter
                                                horizontalAlignment: Text.AlignHCenter
                                            }
                                        }
                                    }
                                    onClicked: {
                                        loginVM.updateIdleTimer()
                                        expandValue = 0
                                        expandAll(false)
                                    }
                                }
                                Item{
                                    width: 33.3
                                    height: 50
                                }
                            }
                        }
                        Item{
                            width: 125
                            height: 50
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Item{
                            width: 325
                            height: 50
                            anchors.verticalCenter: parent.verticalCenter
                            Row{
                                anchors.centerIn: parent
                                Item{
                                    width: 25
                                    height: 50
                                }
                                ButtonMaterial{
                                    _width: 50
                                    _height: 50
                                    _size: 50
                                    _iconSourceOn: "history"
                                    _iconSourceOff: ""
                                    _enableEffect: false
                                    opacity: enabled? 1:0.5
                                    anchors.verticalCenter: parent.verticalCenter
                                    onClicked: {
                                        loginVM.updateIdleTimer()
                                        // modalDialogBoxVM.getModalDialogQML(true,"parm_reset")
                                        // console.log(ioControlVM.titleMap[_currentStation])
                                        modalDialogBoxVM.popupModalDialog(true, -2 , "parm_reset" , "Confirm to reset all parameters for [" + ioControlVM.titleMap[_currentStation] + "] station to the default values - ")
                                    }
                                    Popup{
                                        visible: parent.hovered && parmRoot.visible
                                        x: (parent.width - width) / 2
                                        y: -50
                                        z: 100
                                        width: 120
                                        height: 30
                                        padding: 0
                                        Rectangle{
                                            anchors.fill: parent
                                            color: "gray"
                                            border{
                                                width: 2
                                                color: "black"
                                            }
                                            Text{
                                                anchors.fill: parent
                                                text: "Factory Reset"
                                                color: "white"
                                                font.pointSize: 12
                                                font.family: _fontFamily
                                                verticalAlignment: Text.AlignVCenter
                                                horizontalAlignment: Text.AlignHCenter
                                            }
                                        }
                                    }
                                    Connections{
                                        target: master_app
                                        ignoreUnknownSignals: true
                                        onParmReset:{
                                            parmRoot.resetClicked()
                                        }
                                    }
                                }
                                Item{
                                    width: 25
                                    height: 50
                                }
                                ButtonMaterial{
                                    _width: 50
                                    _height: 50
                                    _size: 50
                                    _iconSourceOn: "cached"
                                    _iconSourceOff: ""
                                    _enableEffect: false
                                    opacity: enabled? 1:0.5
                                    anchors.verticalCenter: parent.verticalCenter
                                    onClicked: {
                                        loginVM.updateIdleTimer()
                                        parmRoot.revertClicked()
                                    }
                                    Popup{
                                        visible: parent.hovered && parmRoot.visible
                                        x: (parent.width - width) / 2
                                        y: -50
                                        z: 100
                                        width: 90
                                        height: 30
                                        padding: 0
                                        Rectangle{
                                            anchors.fill: parent
                                            color: "gray"
                                            border{
                                                width: 2
                                                color: "black"
                                            }
                                            Text{
                                                anchors.fill: parent
                                                text: "Revert"
                                                color: "white"
                                                font.pointSize: 12
                                                font.family: _fontFamily
                                                verticalAlignment: Text.AlignVCenter
                                                horizontalAlignment: Text.AlignHCenter
                                            }
                                        }
                                    }
                                }
                                Item{
                                    width: 25
                                    height: 50
                                }
                                ButtonMaterial{
                                    _width: 50
                                    _height: 50
                                    _size: 50
                                    _iconSourceOn: "content-save-settings"
                                    _iconSourceOff: ""
                                    _enableEffect: false
                                    enabled: columnParm.haveChange
                                    opacity: enabled? 1:0.5
                                    anchors.verticalCenter: parent.verticalCenter
                                    onClicked: {
                                        loginVM.updateIdleTimer()
                                        var tempValue = ""
                                        for(var i = 0; i < parmList.count; i++){
                                            var value = "^" + parmList.get(i).uiValue
                                            tempValue += value
                                        }
                                        uiParmList = tempValue
                                        stationVM.setParameters(_currentStation, tempValue)
                                        checkChange()
                                        stationVM.requestParmList(_currentStation)
                                    }
                                    Popup{
                                        visible: parent.hovered && parmRoot.visible
                                        x: (parent.width - width) / 2
                                        y: -50
                                        z: 100
                                        width: 90
                                        height: 30
                                        padding: 0
                                        Rectangle{
                                            anchors.fill: parent
                                            color: "gray"
                                            border{
                                                width: 2
                                                color: "black"
                                            }
                                            Text{
                                                anchors.fill: parent
                                                text: "Save"
                                                color: "white"
                                                font.pointSize: 12
                                                font.family: _fontFamily
                                                verticalAlignment: Text.AlignVCenter
                                                horizontalAlignment: Text.AlignHCenter
                                            }
                                        }
                                    }
                                }
                                Item{
                                    width: 25
                                    height: 50
                                }
                                ButtonMaterial{
                                    id: closeBtn
                                    _width: 50
                                    _height: 50
                                    _size: 50
                                    _iconSourceOn: "close-box-outline"
                                    _iconSourceOff: ""
                                    _enableEffect: false
                                    opacity: enabled? 1:0.5
                                    anchors.verticalCenter: parent.verticalCenter
                                    Popup{
                                        visible: parent.hovered && parmRoot.visible
                                        x: (parent.width - width) / 2
                                        y: -50
                                        z: 100
                                        width: 90
                                        height: 30
                                        padding: 0
                                        Rectangle{
                                            anchors.fill: parent
                                            color: "gray"
                                            border{
                                                width: 2
                                                color: "black"
                                            }
                                            Text{
                                                anchors.fill: parent
                                                text: "Close"
                                                color: "white"
                                                font.pointSize: 12
                                                font.family: _fontFamily
                                                verticalAlignment: Text.AlignVCenter
                                                horizontalAlignment: Text.AlignHCenter
                                            }
                                        }
                                    }
                                    onClicked: {
                                        checkBeforeClose()
                                    }
                                }
                                Item{
                                    width: 25
                                    height: 50
                                }
                            }
                        }
                    }
                }
                Flickable{
                    contentHeight: parmColumn.height
                    width: 850
                    height: parmColumn.height > 780 ? 780 : parmColumn.height
                    anchors.horizontalCenter: parent.horizontalCenter
                    clip: true
                    Column{
                        id: parmColumn
                        width: parent.width
                        anchors.centerIn: parent
                        Repeater{
                            model: sectionList
                            delegate: Column{
                                Rectangle{
                                    width: 850
                                    height: 50
                                    color: "transparent"
                                    border{
                                        width: 2
                                        color: "gray"
                                    }
                                    Text{
                                        id: sectionTitle
                                        property bool isExpand
                                        anchors.fill: parent
                                        text: isExpand ? ("  ▼  " + nameSection) : ("  ►  " + nameSection)
                                        font.pixelSize: _fontPixelSize
                                        font.family: _fontFamily
                                        color: "white"
                                        horizontalAlignment: Text.AlignLeft
                                        verticalAlignment: Text.AlignVCenter
                                        elide: Text.ElideMiddle
                                    }
                                    MouseArea{
                                        anchors.fill: parent
                                        onClicked: {
                                            loginVM.updateIdleTimer()
                                            sectionTitle.isExpand = !sectionTitle.isExpand
                                            if(sectionTitle.isExpand){
                                                expandValue++
                                            }
                                            else{
                                                expandValue--
                                            }
                                        }
                                    }
                                    Connections{
                                        target: parmRoot
                                        ignoreUnknownSignals: true
                                        onExpandAll:{
                                            sectionTitle.isExpand = value
                                        }
                                    }
                                }
                                Repeater{
                                    model: parmList
                                    delegate: Row{
                                        visible: (parmSection === nameSection) && sectionTitle.isExpand
                                        height: 50
                                        Rectangle{
                                            width: 400
                                            height: parent.height
                                            color: "transparent"
                                            border{
                                                width: 2
                                                color: "gray"
                                            }
                                            Text{
                                                anchors.fill: parent
                                                text: parmName
                                                font.pixelSize: _fontPixelSize
                                                font.family: _fontFamily
                                                color: "white"
                                                horizontalAlignment: Text.AlignRight
                                                verticalAlignment: Text.AlignVCenter
                                                rightPadding: 15
                                                elide: Text.ElideMiddle
                                            }
                                        }
                                        Rectangle{
                                            width: 125
                                            height: parent.height
                                            color: "transparent"
                                            border{
                                                width: 2
                                                color: "gray"
                                            }
                                            Text{
                                                id: valueParm
                                                anchors.fill: parent
                                                font.pixelSize: _fontPixelSize
                                                font.family: _fontFamily
                                                color: "white"
                                                horizontalAlignment: Text.AlignHCenter
                                                verticalAlignment: Text.AlignVCenter
                                                elide: Text.ElideMiddle
                                                text: dataType === "bool" ? (checkValueParm.checked ? "True":"False") : parmSld.value + " " + unit
                                            }
                                        }
                                        Rectangle{
                                            width: 325
                                            height: parent.height
                                            color: "transparent"
                                            border{
                                                width: 2
                                                color: "gray"
                                            }
                                            Row{
                                                width: parent.width
                                                height: parent.height
                                                anchors.centerIn: parent
                                                Item{
                                                    height: parent.height
                                                    width: height
                                                    visible: dataType !== "bool"
                                                    Text{
                                                        anchors.fill: parent
                                                        text: parmSld.from
                                                        font.pixelSize: _fontPixelSize - 3
                                                        font.family: _fontFamily
                                                        color: "white"
                                                        horizontalAlignment: Text.AlignRight
                                                        verticalAlignment: Text.AlignVCenter
                                                    }
                                                }
                                                CheckBox{
                                                    id: checkValueParm
                                                    width: parent.width
                                                    height: parent.height
                                                    visible: dataType === "bool"
                                                    checked: currentValue === "1"
                                                    property bool isCompleted: false
                                                    onCheckedChanged: {
                                                        loginVM.updateIdleTimer()
                                                        if(!isCompleted){
                                                            isCompleted = true
                                                            return
                                                        }
                                                        if(checked){
                                                            valueParm.text = "True"
                                                            parmList.setProperty(index, "uiValue", "1")
                                                        }
                                                        else{
                                                            valueParm.text = "False"
                                                            parmList.setProperty(index, "uiValue", "0")
                                                        }
                                                        checkChange()
                                                    }
                                                    Component.onCompleted: {
                                                        if(!isCompleted){
                                                            isCompleted = true
                                                        }
                                                    }
                                                }
                                                Slider {
                                                    id: parmSld
                                                    width: parent.width - parent.height * 2
                                                    height: parent.height
                                                    opacity: enabled? 1 : 0.5
                                                    from: parseInt(minLimit)
                                                    to: parseInt(maxLimit)
                                                    stepSize: dataType === "double" ?  0.1 : 1
                                                    visible: dataType === "int" || dataType === "double"
                                                    value: parseFloat(currentValue)
                                                    property bool ableDoubleClick: true
                                                    property bool isCompleted: true
                                                    onValueChanged: {
                                                        loginVM.updateIdleTimer()
                                                        if(!isCompleted){
                                                            isCompleted = true
                                                            return
                                                        }
                                                        if(dataType === "double"){
                                                            var tempValue = value * 1000
                                                            parmList.setProperty(index, "uiValue", tempValue.toString())
                                                        }
                                                        else{
                                                            var tempValue = value
                                                            parmList.setProperty(index, "uiValue", tempValue.toString())
                                                        }
                                                        checkChange()
                                                    }
                                                    Component.onCompleted: {
                                                        if(!isCompleted){
                                                            isCompleted = true
                                                        }
                                                        if(dataType === "double"){
                                                            var tempValue = value * 1000
                                                            parmList.setProperty(index, "uiValue", tempValue.toString())
                                                        }
                                                        else{
                                                            var tempValue = value
                                                            parmList.setProperty(index, "uiValue", tempValue.toString())
                                                        }
                                                    }
                                                }
                                                Item{
                                                    height: parent.height
                                                    width: height
                                                    visible: dataType !== "bool"
                                                    Text{
                                                        anchors.fill: parent
                                                        text: parmSld.to
                                                        font.pixelSize: _fontPixelSize - 3
                                                        font.family: _fontFamily
                                                        color: "white"
                                                        horizontalAlignment: Text.AlignLeft
                                                        verticalAlignment: Text.AlignVCenter
                                                    }
                                                }
                                                Connections{
                                                    target: parmRoot
                                                    ignoreUnknownSignals: true
                                                    onResetClicked:{
                                                        parmSld.value = defaultVal
                                                        checkValueParm.checked = defaultVal === "1"
                                                        checkChange()
                                                    }
                                                    onRevertClicked:{
                                                        parmSld.value = currentValue
                                                        checkValueParm.checked = currentValue === "1"
                                                        checkChange()
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
