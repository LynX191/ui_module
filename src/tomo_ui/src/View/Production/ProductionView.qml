import QtQml 2.2
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.3


// My import
import "../../Component"
import "../../View"
import "../../View/FeatureView"
import "StationLayout"
//end

Item{
    id: production_item
    width: parent.width
    height: parent.height
    property bool isStartLotOpen: false
    property bool isAuditOpen: false
    property int _fontPixelSize: 28
    property int _fontSubSize: 20
    property string _fontFamily: "Arial"
    property real rate: 0.738916256
    property real rate2: 0.7
    property bool isExpand: false

    property string currentDateTime
    property string totalRuntime
    property string formattedTime
    property var errorListSaving:[]
    enabled: (loginVM.productionEnabled && productionVM.fileValid) || loginVM.superUserActive
    opacity: enabled ? 1 : 0.5
    property bool enableForRunLot: (!mainWindow.isInitializing && checkStationAccess() ) || loginVM.superUserActive
    property bool serverReady: false
    onVisibleChanged: {
        if(!visible) stationFunction.visible = false
    }
    function checkStationAccess(){
        if(systemBarVM.lotState === "Started"){
            if(systemBarVM.processState !== "Running")
                return true
            else{
                return false
            }
        }
        else{
            return true
        }
    }
    property var myTableModel: [
        { _info: "History of Cycle Time",               _value: "Lot Data" },
        { _info: "Pack Size",                           _value: productionVM.recipeValue },
        { _info: "Product Name",                        _value: productionVM.productName },
        { _info: "Lot Start Time",                      _value: productionVM.lotStartTime },
        { _info: "Lot End Time",                        _value: productionVM.lotEndTime },
        { _info: "Total Runtime",                       _value: formattedTime },
        { _info: "Total Downtime",                      _value: productionVM.totalDowntime },
        { _info: "Average Cartons Per Minute",          _value: productionVM.avgCartonPerMin},
        { _info: "Average Cycle Time",                  _value: productionVM.averageCycleTime + " sec"},
        { _info: "No. of Bottom Tray Processed",        _value: productionVM.noBottomShipper },
        { _info: "No. of Top Tray Processed",           _value: productionVM.noTopShipper },
        { _info: "No. of Completed Trays",              _value: productionVM.completedTray },
        { _info: "No. of Loaded Cartons",               _value: productionVM.noLoadedCarton },
        { _info: "Current Date & Time",                 _value: currentDateTime },
        { _info: "Current Software Version",            _value: "ST02-00.00.01" },
    ]
    ListModel {
        id: imageModel
        ListElement { x: 584; y: 150; z: 2; source: "imageStation/01_green.png"; source1: "imageStation/01_orange.png"; source2: "imageStation/01_red.png" }
        ListElement { x: 363; y: 159; z: 2; source: "imageStation/02_green.png"; source1: "imageStation/02_orange.png"; source2: "imageStation/02_red.png" }
        ListElement { x: 402; y: 169; z: 3; source: "imageStation/03_green.png"; source1: "imageStation/03_orange.png"; source2: "imageStation/03_red.png" }
        ListElement { x: 469; y: 255; z: 0; source: "imageStation/04_green.png"; source1: "imageStation/04_orange.png"; source2: "imageStation/04_red.png" }
        ListElement { x: 522; y: 402; z: 2; source: "imageStation/05_green.png"; source1: "imageStation/05_orange.png"; source2: "imageStation/05_red.png" }
        ListElement { x: 512; y: 430; z: 3; source: "imageStation/06_green.png"; source1: "imageStation/06_orange.png"; source2: "imageStation/06_red.png" }
        ListElement { x: 366; y: 529; z: 2; source: "imageStation/07_green.png"; source1: "imageStation/07_orange.png"; source2: "imageStation/07_red.png" }
        ListElement { x: 330; y: 425; z: 3; source: "imageStation/08_green.png"; source1: "imageStation/08_orange.png"; source2: "imageStation/08_red.png" }
        ListElement { x: 103; y: 499; z: 3; source: "imageStation/09_green.png"; source1: "imageStation/09_orange.png"; source2: "imageStation/09_red.png" }
        ListElement { x: 036; y: 649; z: 5; source: "imageStation/10_green.png"; source1: "imageStation/10_orange.png"; source2: "imageStation/10_red.png" }
        ListElement { x: 034; y: 605; z: 4; source: "imageStation/11_green.png"; source1: "imageStation/11_orange.png"; source2: "imageStation/11_red.png" }
        ListElement { x: 116; y: 662; z: 5; source: "imageStation/12_green.png"; source1: "imageStation/12_orange.png"; source2: "imageStation/12_red.png" }
        ListElement { x: 067; y: 312; z: 1; source: "imageStation/13_green.png"; source1: "imageStation/13_orange.png"; source2: "imageStation/13_red.png" }
    }

    ListModel{
        id: emegencyModel
        ListElement { x: 520; y: 695; z: 0; source: "imageStation/ES1.png"; source2: "imageStation/ES1A.png"; name: "estop1"  }
        ListElement { x: 105; y: 575; z: 0; source: "imageStation/ES2.png"; source2: "imageStation/ES2A.png"; name: "estop2"  }
        ListElement { x: 176; y: 280; z: 0; source: "imageStation/ES3.png"; source2: "imageStation/ES3A.png"; name: "estop3"  }
        ListElement { x: 557; y: 100; z: 0; source: "imageStation/ES4.png"; source2: "imageStation/ES4A.png"; name: "estop4"  }
        ListElement { x: 850; y: 180; z: 0; source: "imageStation/ES5.png"; source2: "imageStation/ES5A.png"; name: "estop5"  }
        ListElement { x: 705; y: 630; z: 0; source: "imageStation/ES6.png"; source2: "imageStation/ES6A.png"; name: "estop6"  }
        ListElement { x: 855; y: 320; z: 0; source: "imageStation/GD1.png"; source2: "imageStation/GD1A.png"; name: "guard1"  }
        ListElement { x: 290; y: 150; z: 0; source: "imageStation/GD2.png"; source2: "imageStation/GD2A.png"; name: "guard2"  }
        ListElement { x: 002; y: 500; z: 0; source: "imageStation/GD3.png"; source2: "imageStation/GD3A.png"; name: "guard3"  }
        ListElement { x: 335; y: 695; z: 0; source: "imageStation/GD4.png"; source2: "imageStation/GD4A.png"; name: "cm_door" }
        ListElement { x: 335; y: 173; z: 4; source: "imageStation/CT1.png"; source2: "imageStation/CT1A.png"; name: "im_int"  }
        ListElement { x: 80 ; y: 321; z: 2; source: "imageStation/CT2.png"; source2: "imageStation/CT2A.png"; name: "om_int_t1"  }
        ListElement { x: 214; y: 321; z: 2; source: "imageStation/CT3.png"; source2: "imageStation/CT3A.png"; name: "om_int_t2"  }
    }
    Timer {
        id: counterTimer
        interval: 1000
        running: false
        repeat: true
        triggeredOnStart: true
        onTriggered: {
			var currentTime = productionVM.getCurrentTimeProduct()
			var lotDuration	= currentTime - productionVM.lotStartTimeSec
            var hours = Math.floor(lotDuration / 3600);
            var minutes = Math.floor((lotDuration % 3600) / 60);
            var seconds = Math.round(lotDuration % 60);
			formattedTime = ("00" + hours).slice(-2) + ":" +
					("00" + minutes).slice(-2) + ":" +
					("00" + seconds).slice(-2);
        }
    }

    Timer {
        interval: 1000
        running: true
        repeat: true
        triggeredOnStart: true
        onTriggered: {
            var currentTime = new Date();
            var year = currentTime.getFullYear();
            var month = currentTime.getMonth() + 1;
            var day = currentTime.getDate();
            var hours = currentTime.getHours();
            var minutes = currentTime.getMinutes();
            var seconds = currentTime.getSeconds();
            // Format the date and time as YYYY-MM-DD HH:mm:ss
            var formattedDateTime = pad(year) + "-" + pad(month) + "-" + pad(day) + "\n" +
                                    pad(hours) + ":" + pad(minutes) + ":" + pad(seconds);
            currentDateTime = formattedDateTime;
        }
    }

    Connections{
        target: productionVM
        ignoreUnknownSignals: true
        onLotEndTimeChanged:{
            if(productionVM.lotEndTime === "")
                return
            counterTimer.running = false
        }
        onConfirmStartLotClickedChanged:{
            counterTimer.running = true
        }
        onOpenStartLotConfirm:{
            startlotConfirm.visible  = true
        }
		onOpenEndLotConfirm:{
			endlotConfirm.visible = true
		}
        onOpenAbortLotConfirm:{
            abortLotConfirm.visible = true
        }
        onTotalRuntimeChanged:{
            var currentRunTime = productionVM.totalRuntime

            // Format the time with leading zeros
            formattedTime = currentRunTime
        }
    }

    function pad(number) {
        return (number < 10 ? "0" : "") + number;
    }

    ListModel {
        id: production_TOMOC
    }

    ListModel {
        id: station_name
        Component.onCompleted: {
            for (var i = 1; i <= 13; i++) {
                if (i == 10) {
                    station_name.append({"name": ">  S10. Conveyor"});
                } else if (i == 11|i == 12) {
                    continue; // Skip iteration for i = 11
                } else {
                    station_name.append({"name": ioControlVM.titleMap[i]});
                }
            }
        }
    }
    ListModel {
        id: station_name2
        Component.onCompleted: {
            for (var i = 1; i <= 13; i++) {
                if (i == 10) {
                    station_name2.append({"name": " v  S10. Conveyor"});
                }
                station_name2.append({"name": ioControlVM.titleMap[i]});
            }
        }
    }

    onEnabledChanged:{
        if(!enabled) stationFunction.visible = false
    }
    property bool firstRunning
    Connections{
        target: systemBarVM
        ignoreUnknownSignals: true
        onProcessStateChanged:{
            if(systemBarVM.processState === "Running" && !firstRunning){
                firstRunning = true
                productionVM.firstRunning = true
            }
        }
    }
    Connections{
        target: master_app
        ignoreUnknownSignals: true
        onPowerUpStateChanged:{
            if(!state){
                firstRunning = false
                productionVM.firstRunning = false
            }
        }
    }
    function openStation(stationIndex,stationTitle){
        if(!systemLogVM.serverReady){
            modalDialogBoxVM.getModalDialogQML(false,"wait_server")
            return
        }
        if(!master_app.systemReady){
            modalDialogBoxVM.getModalDialogQML(false,"system_lost")
            return
        }
        if(!firstRunning){
            modalDialogBoxVM.getModalDialogQML(false, "init_move")
            return
        }
        if(!stationFunction.visible){
            var stationName = "Station" + stationIndex
            stationFunction._stationTitle = stationTitle
            if(stationIndex < 10){
            stationFunction._stationIndex = stationIndex
            }
            else if(stationIndex === 11){
                stationFunction._stationIndex = 13
                stationName = "Station13"
            }
            stationFunction._sourceImage = "../imageStation/"+stationName+".png"
            stationFunction.visible = true
        }
    }

    function openStation2(stationIndex,stationTitle){
        if(!systemLogVM.serverReady){
            modalDialogBoxVM.getModalDialogQML(false,"wait_server")
            return
        }
        if(!master_app.systemReady){
            modalDialogBoxVM.getModalDialogQML(false,"system_lost")
            return
        }
        if(!firstRunning){
            modalDialogBoxVM.getModalDialogQML(false, "init_move")
            return
        }
        if(!stationFunction.visible){
            var stationName = "Station" + stationIndex
            stationFunction._stationTitle = stationTitle
            stationFunction._stationIndex = stationIndex
            stationFunction._sourceImage = "../imageStation/"+stationName+".png"
            stationFunction.visible = true
        }
    }

    GridLayout{
        width: parent.width
        height: width * 1080 / 1795
        anchors.top: parent.top
        columns: 18
        rows: 10
        rowSpacing: 0
        columnSpacing: 0
        Layout.fillHeight: true
        Layout.fillWidth: true
        Rectangle{
            id: stationListRec
            enabled: enableForRunLot
            opacity: enabled ? 1 : 0.5
            Layout.columnSpan: 4
            Layout.rowSpan: 10
            Layout.preferredWidth:  parent.width/parent.columns* Layout.columnSpan
            Layout.preferredHeight: parent.height /parent.rows* Layout.rowSpan
            Layout.alignment: Qt.AlignTop
            color: "transparent"
            Label {
                text: qsTr("Station")
                anchors{horizontalCenter: parent.horizontalCenter
                    top: parent.top}
                font.pixelSize: _fontPixelSize
                font.family: _fontFamily
            }
            GridLayout {
                id: grid_name
                columns: 1
                rows: 14
                width: parent.width
                height: parent.height
                columnSpacing: 0
                rowSpacing: 0
                y: 30
                Repeater {
                    id: stationRpt
                    model: production_item.isExpand? station_name2 : station_name
                    Layout.fillHeight: true
                    delegate: Rectangle {
                        Layout.preferredWidth: grid_name.width / grid_name.columns
                        Layout.preferredHeight: grid_name.height / grid_name.rows * 0.9
                        Layout.columnSpan: 1
                        Layout.rowSpan: 1
                        color: "transparent"
                        clip: true
                        opacity: endLotPopup.visible ? 0 : 1
                        enabled: endLotPopup.visible ? false : true
                        Text {
                            anchors.left: parent.left
                            anchors.leftMargin: {
                                if (isExpand && index > 9 && index < 13) {
                                    return 40
                                } else if(index ===9 ){
                                    return -3
                                } else {
                                    return 20
                                }
                            }
                            anchors.verticalCenter: parent.verticalCenter
                            text: typeof model.name === "undefined" ? "" : model.name
                            font.pixelSize: _fontSubSize
                            font.family: _fontFamily
                            color: "white"
                        }
                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: false
                            onEntered: {
                                parent.children[0].font.bold = true
                                if(index < 9 ){
                                    imageModelRepeater.itemAt(index).opacity = 1;
                                }
                                else if (index === 9 ){
                                    imageModelRepeater.itemAt(9).opacity = 1;
                                    imageModelRepeater.itemAt(10).opacity = 1;
                                    imageModelRepeater.itemAt(11).opacity = 1;
                                }
                                else if (index > 9 ){
                                    if(isExpand === true){
                                        imageModelRepeater.itemAt(index - 1).opacity = 1;
                                    }
                                    else if(isExpand === false ){
                                        imageModelRepeater.itemAt(12).opacity = 1;
                                    }
                                }
                            }
                            onExited: {
                                parent.children[0].font.bold = false
                                if(errorListSaving[index + 1] < -1) return
                                if(index < 9 ){
                                if(errorListSaving[index + 1] < -1) return
                                    imageModelRepeater.itemAt(index).opacity = 0.3;
                                }
                                else if (index === 9 ){
                                    imageModelRepeater.itemAt(9).opacity = 0.3;
                                    imageModelRepeater.itemAt(10).opacity = 0.3;
                                    imageModelRepeater.itemAt(11).opacity = 0.3;
                                }
                                else if (index > 9 ){
                                    if(isExpand){
                                        imageModelRepeater.itemAt(index - 1).opacity = 0.3;
                                    }
                                    else if(!isExpand){
                                        imageModelRepeater.itemAt(12).opacity = 0.3;
                                    }
                                }
                            }
                            onClicked: {
                                loginVM.updateIdleTimer()
                                var stationIndex = index + 1;
                                if(stationIndex === 10){
                                    production_item.isExpand = !production_item.isExpand
                                }
                                else{
                                    if(production_item.isExpand){
                                        if(stationIndex < 10){
                                            openStation2(stationIndex,model.name)
                                        }
                                        else{
                                            openStation2(index,model.name)
                                        }
                                    }
                                    else{
                                        if(stationIndex < 10){
                                            openStation(stationIndex,model.name)
                                        }
                                        else{
                                            openStation(11,model.name)
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        Rectangle{
            Layout.columnSpan: 9
            Layout.rowSpan: 1
            Layout.preferredWidth:  parent.width/parent.columns* Layout.columnSpan
            Layout.preferredHeight: parent.height /parent.rows* Layout.rowSpan
            color: "black"
            enabled: controlBarVM.playIconStatus || loginVM.superUserActive
            RowLayout{
                width: parent.width-spacing*2
                height: parent.height * 2/3
                anchors.centerIn: parent
                spacing: 20
                property int buttonCount: 4
                property int cellWidth: parent.width / parent.buttonCount
                Item{
                    Layout.preferredWidth: parent.width / parent.buttonCount-parent.spacing
                    Layout.preferredHeight: parent.height
                    Layout.alignment: Qt.AlignVCenter
                    Button{
                        id: startLotButton
                        enabled: (productionVM.startLotEnabled && productionVM.bypassValue) || loginVM.superUserActive
                        anchors.fill: parent
                        contentItem: Text {
                            text: "Start Lot"
                            font.pixelSize: _fontSubSize
                            font.family: _fontFamily
                            opacity: enabled ? 1.0 : 0.3
                            color: "white"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            elide: Text.ElideRight
                        }
                        onClicked: {
                            loginVM.updateIdleTimer()
                            isStartLotOpen = true
                        }
                    }
                }
                Item{
                    Layout.preferredWidth: parent.width / parent.buttonCount-parent.spacing
                    Layout.preferredHeight: parent.height
                    Layout.alignment: Qt.AlignVCenter
                    Button{
                        id: endLotButton
                        enabled: (productionVM.endLotEnabled && productionVM.bypassValue) || loginVM.superUserActive
                        anchors.fill: parent
                        contentItem: Text {
                            text: "End Lot"
                            font.pixelSize: _fontSubSize
                            font.family: _fontFamily
                            opacity: enabled ? 1.0 : 0.3
                            color: "white"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            elide: Text.ElideRight
                        }
                        onClicked: {
                            loginVM.updateIdleTimer()
							endlotConfirm.visible = true
                        }
                        Connections{
                            target: productionVM
                            ignoreUnknownSignals: true
                            onOpenEndLotPopup: endLotPopup.visible = true
                        }
                    }
                }
                ComboBoxCustom{
                    id: statusCbb
                    Layout.preferredWidth: parent.width / parent.buttonCount-parent.spacing
                    Layout.preferredHeight: parent.height
                    implicitWidth: parent.width / parent.buttonCount
					enabled: productionVM.enabledForRunLot || loginVM.superUserActive
                    opacity: enabled ? 1.0 : 0.3
                    property var operatingModel: ["Remote","Local"]
                    property bool isCompleted: false
                    _model: operatingModel
                    // _bgCombobox: "#303030"
                    // _bgPopup: "#141414"
                    _opacityPopup: 0.6
                    _viewText:"Status"
                    _fontSize:_fontSubSize
                    _fontFamily: _fontFamily
                    modelDataColor:"white"
                    onActivated: {
                        loginVM.updateIdleTimer()
                        productionVM.setBypassValue(currentIndex)
                        master_app.setBypassMode(currentIndex)
                    }
                    delegate: ItemDelegate {
                        width: statusCbb.width
                        height: statusCbb.height
                        contentItem: Text {
                            text: modelData
                            color: "white"
                            leftPadding: 10
                            font.pixelSize: _fontPixelSize - 4
                            font.family: _fontFamily
                            elide: Text.ElideRight
                            verticalAlignment: Text.AlignVCenter
                        }
                        highlighted: statusCbb.highlightedIndex === index
                    }
                    Component.onCompleted: {
                        if(!isCompleted) {
                            currentIndex = master_app.getBypassMode()
                            isCompleted = true
                        }
                    }
                }
                Item{
                    Layout.preferredWidth: parent.width / parent.buttonCount-parent.spacing
                    Layout.preferredHeight: parent.height
                    Layout.alignment: Qt.AlignVCenter
                    Button{
                        id: auditConveyorButton
                        enabled: productionVM.auditConveyorEnabled
                        anchors.fill: parent
                        contentItem: Text {
                            text: "Audit Conveyor"
                            font.pixelSize: _fontSubSize
                            font.family: _fontFamily
                            opacity: enabled ? 1.0 : 0.3
                            color: "white"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            elide: Text.ElideRight
                        }
                        onClicked: {
                            loginVM.updateIdleTimer()
                            isAuditOpen = true
                        }

                    }
                }
            }
        }
        Rectangle{
            Layout.columnSpan: 5
            Layout.rowSpan: 10
            Layout.preferredWidth:  parent.width/parent.columns * Layout.columnSpan
            Layout.preferredHeight: parent.height /parent.rows * Layout.rowSpan
            Layout.alignment: Qt.AlignTop
            color: "transparent"
            Item{
                id: mainCycleTime
                anchors.fill: parent
                Item{
                    width: parent.width
                    height: parent.height
                    Item {
                        id:tableArea
                        anchors.fill: parent
                        clip: true
                        ListView {
                            id: listShortcut
                            anchors.fill: parent
                            model: myTableModel
                            interactive: false
                            clip: true
                            property var lineHeight: mainCycleTime.height / (count + 5.8)
                            delegate: Rectangle {
                                width: listShortcut.width
                                height: listShortcut.lineHeight
                                Row {
                                    anchors.fill: parent
                                    spacing: 0
                                    Rectangle{
                                        id: infoCol
                                        width: parent.width*0.65
                                        height: parent.height
                                        color: index % 2 == 0 ? "#4c5052":"#232526"
                                        border.color: modelData._info.indexOf("History") != -1?  (historyPopup.visible? "#7db8b0":"#24304f"):  "black"
                                        border.width: modelData._info.indexOf("History") != -1?  4 : 0.5
                                        Text {
                                            anchors.fill: parent
                                            text: modelData._info
                                            color: "white"
                                            font.pointSize: 14
                                            font.bold: modelData._info.indexOf("History") != -1?  true:  false
                                            verticalAlignment: Text.AlignVCenter
                                            horizontalAlignment: Text.AlignLeft
                                            leftPadding: 10
                                        }
                                        MouseArea{
                                            anchors.fill: parent
                                            enabled: (modelData._info.indexOf("History") != -1) ||  (modelData._info.indexOf("Software") != -1) 
                                            onClicked: {
                                                if(modelData._info.indexOf("History") != -1){
                                                    loginVM.updateIdleTimer()
                                                    infoCol.border.color = "#24304f"
                                                    historyPopup.open()
                                                }
                                            }
                                            onDoubleClicked: {                                                
                                                if(modelData._info.indexOf("Software") != -1){
                                                    loginVM.updateIdleTimer()
                                                    uiDialog.open()
                                                }
                                            }
                                        }
                                    }
                                    Rectangle{
                                        id: valueCol
                                        width: parent.width-infoCol.width
                                        height: parent.height
                                        color: index % 2 == 0 ? "#4c5052":"#232526"
                                        border.color: modelData._info.indexOf("History") != -1?  (lotDataPopup.visible? "#7db8b0":"#24304f"):  "black"
                                        border.width: modelData._info.indexOf("History") != -1?  4 : 0.5
                                        Text {
                                            anchors.fill: parent
                                            text: modelData._value
                                            color: "white"
                                            font.pointSize: 14
                                            font.bold: modelData._info.indexOf("History") != -1?  true:  false
                                            verticalAlignment: Text.AlignVCenter
                                            horizontalAlignment: Text.AlignLeft
                                            leftPadding: 10
                                        }
                                        MouseArea{
                                            anchors.fill: parent
                                            enabled: (modelData._info.indexOf("History") != -1) ||  (modelData._info.indexOf("Software") != -1) 
                                            onClicked: {
                                                if(modelData._info.indexOf("History") != -1){
                                                    loginVM.updateIdleTimer()
                                                    valueCol.border.color = "#24304f"
                                                    lotDataPopup.open()
                                                }
                                            }
                                            onDoubleClicked: {                                                
                                                if(modelData._info.indexOf("Software") != -1){
                                                    loginVM.updateIdleTimer()
                                                    uiDialog.open()
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
        Rectangle{
            id: mainStationRec
            enabled: enableForRunLot
            opacity: enabled ? 1 : 0.5
            Layout.columnSpan: 9
            Layout.rowSpan: 9
            Layout.preferredWidth:  parent.width/parent.columns* Layout.columnSpan
            Layout.preferredHeight: parent.height /parent.rows* Layout.rowSpan
            color: "transparent"
            border{
                width: 0.5
                color: "black"
            }
            Item{
                id: mainStationTop
                width: parent.width
                height: 75
                z:5
                function getBypassValue(){
                    if(productionVM.bypassValue)
                        return "Local"
                    else
                        return "Remote"
                }
                function getDryRunValue(){
                    if(productionVM.dryRunValue)
                        return "On"
                    else
                        return "Off"
                }
                Rectangle{
                    anchors.fill: parent
                    color: "black"
                    border{
                        width: productionVM.enabledForRunLot ? 0 : 5
                        color: "#7db8b0"
                    }
					Row{
						anchors.centerIn: parent
						spacing: 21
						Rectangle {
							color: "#1e1e1e"; width: mainStationTop.width/4 - 2*parent.spacing/3; height: mainStationTop.height - 10
							Text {
							    text: "Status: " + mainStationTop.getBypassValue()
							    color: "yellow"
								font.pixelSize: _fontSubSize
								font.family: _fontFamily
							    horizontalAlignment: Text.AlignHCenter
							    verticalAlignment: Text.AlignVCenter
								anchors.centerIn: parent
								font.bold: true
							}
						}
						Rectangle {
							color: "#1e1e1e"; width: 2*mainStationTop.width/4 - 2*parent.spacing/3; height: mainStationTop.height - 10
                            property bool operationState: ( systemBarVM.processState === "Pause" || systemBarVM.processState === "Error" ) && ( systemBarVM.lotState === "Started" || systemBarVM.lotState === "Ending" )
							Text {
                                visible: true // !parent.operationState
							    property string operationValue: productionVM.enabledForRunLot ? "Manual     " : "Automatic"
							    text: "Operation Mode: "+ operationValue
							    color: "yellow"
								font.pixelSize: _fontSubSize
								font.family: _fontFamily
							    horizontalAlignment: Text.AlignHCenter
							    verticalAlignment: Text.AlignVCenter
								anchors.centerIn: parent
								font.bold: true
							}
                            Switch{
                                id: operationSwitch
                                visible: false //parent.operationState
                                anchors.fill: parent
                                text: checked ? "Operation Mode: Automatic" : "Operation Mode: Manual     "
                                checked: productionVM.operationValue
                                indicator: Rectangle {
                                    implicitWidth: 48
                                    implicitHeight: 26
                                    x: parent.width / 2 - width / 2
                                    y: 34
                                    radius: 13
                                    color: operationSwitch.checked ? "#17a81a" : "#ffffff"
                                    border.color: operationSwitch.checked ? "#17a81a" : "#cccccc"

                                    Rectangle {
                                        x: operationSwitch.checked ? parent.width - width : 0
                                        width: 26
                                        height: 26
                                        radius: 13
                                        color: operationSwitch.down ? "#cccccc" : "#ffffff"
                                        border.color: operationSwitch.checked ? (operationSwitch.down ? "#17a81a" : "#21be2b") : "#999999"
                                    }
                                }

                                contentItem: Text {
                                    text: operationSwitch.text
                                    color: "yellow"
                                    font.pixelSize: _fontSubSize
                                    font.family: _fontFamily
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignTop
                                    anchors.centerIn: parent
                                    font.bold: true
                                }
                                onCheckedChanged: {
                                    loginVM.updateIdleTimer()
                                    if(checked)
                                    productionVM.setOperationValue(checked)
                                }
                                onVisibleChanged: {
                                    if(visible){
                                        loginVM.updateIdleTimer()
                                        productionVM.setOperationValue(checked)
                                    }
                                }
                                // onPressed: {
                                //     if(checked){
                                //         modalDialogBoxVM.getModalDialogQML(true, "auto_manual");
                                //     }
                                // }
                                onReleased: {
                                    if(!checked){
                                        checked = true
                                        modalDialogBoxVM.getModalDialogQML(true, "auto_manual");
                                    }
                                }
                                Connections{
                                    target: productionVM
                                    ignoreUnknownSignals: true
                                    onOperationValueChanged:{
                                        enabled = true
                                        operationSwitch.checked = productionVM.operationValue
                                    }
                                }
                            }
						}
						Rectangle {
							color: "#1e1e1e"; width: mainStationTop.width/4- 2*parent.spacing/3; height: mainStationTop.height - 10
							Text {
                                id: dryText
							    text: productionVM.dryRunValue ? "Dry Run Mode: On" : "Dry Run Mode: Off" //mainStationTop.getDryRunValue()
							    color: "yellow"
								font.pixelSize: _fontSubSize
								font.family: _fontFamily
							    horizontalAlignment: Text.AlignHCenter
							    verticalAlignment: Text.AlignVCenter
								anchors.centerIn: parent
								font.bold: true
							}
						}
					}
                }
            }

            Item{
                anchors.fill: parent
                id: strayItem
                Image {
                    id: stray_image
                    anchors.centerIn: parent
                    width: parent.width
                    height: parent.height
                    fillMode: Image.PreserveAspectFit
                    source: "imageStation/ShipperTray.png"
                }
                Repeater {
                    id: imageModelRepeater
                    model: imageModel
                    delegate: Image {
                        id: image
                        width: sourceSize.width * rate
                        height: sourceSize.height * rate
						enabled: !startlotConfirm.visible 
                        x: model.x + stray_image.x
                        y: model.y + stray_image.y
                        z: model.z
                        source: model.source
                        opacity: 0.3
                        MouseArea {
                            anchors.fill: parent
                            hoverEnabled: false
                            onEntered: {
                                if(index < 9){
                                stationRpt.itemAt(index).children[0].font.bold = true
                                }
                                else {
                                    if(isExpand === true){
                                        stationRpt.itemAt(index + 1).children[0].font.bold = true
                                    }
                                    else if(isExpand === false){
                                            if(index === 9 |index === 10 |index === 11){
                                            stationRpt.itemAt(9).children[0].font.bold = true
                                        }
                                        else if(index === 12){
                                            stationRpt.itemAt(10).children[0].font.bold = true
                                        }
                                    }
                                }
                                image.opacity = 1;
                            }
                            onExited: {
                                if(index < 9){
                                stationRpt.itemAt(index).children[0].font.bold = false
                                }
                                else {
                                    if(isExpand === true){
                                        stationRpt.itemAt(index + 1).children[0].font.bold = false
                                    }
                                    else if(isExpand === false){
                                            if(index === 9 |index === 10 |index === 11){
                                            stationRpt.itemAt(9).children[0].font.bold = false
                                        }
                                        else if(index === 12){
                                            stationRpt.itemAt(10).children[0].font.bold = false
                                        }
                                    }
                                }
                                if(errorListSaving[index + 1] < -1) return
                                image.opacity = 0.3;
                            }
                            onClicked: {
                                loginVM.updateIdleTimer()
                                var stationIndex = index + 1
                                var stationTitle
                                if(stationIndex < 10){
                                    stationTitle= station_name2.get(stationIndex - 1).name
                                }
                                else if (stationIndex >= 10){
                                    stationTitle= station_name2.get(stationIndex).name
                                }
                                openStation2(stationIndex ,stationTitle)
                            }
                        }
                    }
                }
                Connections{
                    target: modalDialogBoxVM
                    ignoreUnknownSignals: true
                    onModulesStateConverted:{
                        errorListSaving = modulesState
                        for(var i = 1; i < modulesState.length; i++){
                            if(modulesState[i] === -3 ){
                                imageModelRepeater.itemAt(i - 1).source = imageModel.get(i - 1).source2
                                imageModelRepeater.itemAt(i - 1).opacity = 1
                            }
                            else if(modulesState[i] === -2 ){
                                imageModelRepeater.itemAt(i - 1).source = imageModel.get(i - 1).source1
                                imageModelRepeater.itemAt(i - 1).opacity = 1
                            }
                            else{
                                imageModelRepeater.itemAt(i - 1).source = imageModel.get(i - 1).source
                                imageModelRepeater.itemAt(i - 1).opacity = 0.3
                            }
                        }
                    }
                }
                Repeater{
                    id: emegencyRepeater
                    model: emegencyModel
                    delegate: Image {
                        id: emergencyImage
                        width: sourceSize.width * rate2
                        height: sourceSize.height * rate2
                        x: model.x + stray_image.x
                        y: model.y + stray_image.y
                        z: model.z
                        source: model.source
                        Connections{
                            target: productionVM
                            ignoreUnknownSignals: true
                            onEmergencyEvent: {
                                if(model.name === nameEvent){
                                    if(state) emergencyImage.source = model.source2
                                    else emergencyImage.source = model.source
                                }
                            }
                        }
                    }
                }
                property var inspecVector: controlBarVM.inspecVector
                property var inspecModel: [
                    { name: "Folding Inspection",               value: inspecVector[0], x: 556, y: 305 },
                    { name: "Shipper Inspection",               value: inspecVector[1], x: 713, y: 220 },
                    { name: "Carton Stack Inspection (Main)",   value: inspecVector[2], x: 487, y: 550 },
                    { name: "Carton Stack Inspection (Buffer)", value: inspecVector[3], x: 530, y: 600 },
                    { name: "Filling Inspection",               value: inspecVector[4], x: 554, y: 467 },
                ]
                Repeater{
                    model: parent.inspecModel
                    delegate: Rectangle{
                        width: 30
                        height: width
                        color: "black"
                        x: modelData.x
                        y: modelData.y
                        z: 10
                        border{
                            width: 1
                            color: "gray"
                        }
                        radius: 15
                        ButtonMaterial{
                            _width: parent.width
                            _height: _width
                            _size: _width
                            _iconSourceOn: "camera-iris"
                            _iconSourceOff: ""
                            anchors.centerIn: parent
                            _colorOverlayLow: modelData.value ? "#52ff80":"gray"
                            _enableEffect: false
                            enabled: false
                        }
                    }
                }

                Connections{
                    ignoreUnknownSignals: true
                    target: stationVM
                    onParametersListChanged:{
                        strayItem.expandModel(stationVM.parametersList)
                    }
                }
                ListModel{
                    id: parmList
                }
                function expandModel(input){
                    parmList.clear()
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
                        parmList.append({parmSection: parmSection, parmName: parmName, dataType: dataType , unit: unit , defaultVal: defaultVal , minLimit: minLimit , maxLimit: maxLimit , uiValue: defaultVal , currentValue: typeof currentValue !== "undefined" ? currentValue : defaultVal})
                    }
                    if(parmList.get(0).parmName === "Enable Folding Inspection"){
                        for(var i = 0; i < 5; i++){
                            controlBarVM.inspecVector[i] = parseInt(parmList.get(i).currentValue)
                        }
                        controlBarVM.inspecVectorChanged()
                    }
                    else{
                        parmList.clear()
                    }
                }
            }
        }
    }

    //Popup Start Lot
    Popup{
        id: popUpStartLot
        width: 600
        height: 450
        x: mainWindow.width / 2 - width / 2 - control_bar_layout.width
        y: mainWindow.height / 2 - height / 2 - notifyBarArea.height
        visible: isStartLotOpen
        onClosed: isStartLotOpen = false
        modal: true
        closePolicy: Popup.CloseOnEscape
        Rectangle{
            anchors.fill: parent
            color: "transparent"
            border{
                width: 3
                color: "black"
            }
            ColumnLayout{
                spacing: 10
                anchors.fill: parent
                Layout.fillHeight : true
                Row{
                    height: parent.height / 5 //productionVM.bypassValue ? parent.height / 5 : parent.height / 4
                    width: parent.width
                    Layout.alignment: Qt.AlignCenter
                    Text{
                        width: parent.width / 2
                        height: parent.height
                        leftPadding: 10
                        text: "Recipe (Carton Type)"
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        font.bold: true
                        verticalAlignment: Text.AlignVCenter
                    }
                    Item{
                        width: parent.width / 2
                        height: parent.height
                        ComboBox{
                            id: recipeCbb
                            property var recipeModel: [5,30,90]
                            model: recipeModel
                            width: parent.width * 0.95
                            height: parent.height * 0.9
                            anchors.centerIn: parent
                            currentIndex: recipeModel.indexOf(productionVM.recipeValue)
                            delegate: ItemDelegate {
                                width: recipeCbb.width
                                height: recipeCbb.height
                                contentItem: Text {
                                    text: modelData
                                    color: "black"
                                    leftPadding: 10
                                    font.pixelSize: _fontPixelSize - 4
                                    font.family: _fontFamily
                                    elide: Text.ElideRight
                                    verticalAlignment: Text.AlignVCenter
                                }
                                highlighted: recipeCbb.highlightedIndex === index
                            }
                            contentItem: Text {
                                width: recipeCbb.width
                                height: recipeCbb.height
                                leftPadding: 10
                                rightPadding: recipeCbb.indicator.width + recipeCbb.spacing
                                text: recipeCbb.displayText
                                font.pixelSize: _fontPixelSize
                                font.family: _fontFamily
                                color: "black"
                                elide: Text.ElideRight
                                verticalAlignment: Text.AlignVCenter
                            }
                            onActivated: {
                                loginVM.updateIdleTimer()
                                displayText = currentText
                                productionVM.setRecipeValue(currentText)
                            }
                        }
                    }
                }
                Row{
                    height: parent.height / 5
                    width: parent.width
                    Layout.alignment: Qt.AlignCenter
                    // visible: productionVM.bypassValue 
                    Text{
                        width: parent.width / 2
                        height: parent.height
                        leftPadding: 10
                        text: "Product Name"
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        font.bold: true
                        verticalAlignment: Text.AlignVCenter
                    }
                    Item{
                        width: parent.width / 2
                        height: parent.height
                        ComboBox{
                            id: productIdCbb
                            property var productIdModel: ["Precision", "FreshLook", "Dailies"]
                            model: productIdModel
                            width: parent.width * 0.95
                            height: parent.height * 0.9
                            anchors.centerIn: parent
                            currentIndex: productionVM.productId
                            delegate: ItemDelegate {
                                width: productIdCbb.width
                                height: productIdCbb.height
                                contentItem: Text {
                                    text: modelData
                                    color: "black"
                                    leftPadding: 10
                                    font.pixelSize: _fontPixelSize - 4
                                    font.family: _fontFamily
                                    elide: Text.ElideRight
                                    verticalAlignment: Text.AlignVCenter
                                }
                                highlighted: productIdCbb.highlightedIndex === index
                            }
                            contentItem: Text {
                                width: productIdCbb.width
                                height: productIdCbb.height
                                leftPadding: 10
                                rightPadding: productIdCbb.indicator.width + productIdCbb.spacing
                                text: productIdCbb.displayText
                                font.pixelSize: _fontPixelSize
                                font.family: _fontFamily
                                color: "black"
                                elide: Text.ElideRight
                                verticalAlignment: Text.AlignVCenter
                            }
                            onActivated: {
                                loginVM.updateIdleTimer()
                                displayText = currentText
                                productionVM.setProductId(currentIndex)
                            }
                        }
                    }
                }
                Row{
                    height: parent.height / 5 //productionVM.bypassValue ? parent.height / 5 : parent.height / 4
                    width: parent.width
                    Layout.alignment: Qt.AlignCenter
                    Text{
                        width: parent.width / 2
                        height: parent.height
                        leftPadding: 10
                        text: "Dry Run Mode"
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        font.bold: true
                        verticalAlignment: Text.AlignVCenter
                    }
                    Item{
                        width: parent.width / 2
                        height: parent.height
                        ComboBox {
                            id: dryRunCbb
                            property var operatingModel: ["Off","On"]
                            model: operatingModel
                            width: parent.width * 0.95
                            height: parent.height * 0.9
                            anchors.centerIn: parent
                            property bool isCompleted: false
                            delegate: ItemDelegate {
                                width: dryRunCbb.width
                                height: dryRunCbb.height
                                contentItem: Text {
                                    text: modelData
                                    color: "black"
                                    leftPadding: 10
                                    font.pixelSize: _fontPixelSize - 4
                                    font.family: _fontFamily
                                    elide: Text.ElideRight
                                    verticalAlignment: Text.AlignVCenter
                                }
                                highlighted: dryRunCbb.highlightedIndex === index
                            }
                            contentItem: Text {
                                width: dryRunCbb.width
                                height: dryRunCbb.height
                                leftPadding: 10
                                rightPadding: dryRunCbb.indicator.width + dryRunCbb.spacing
                                text: dryRunCbb.displayText
                                font.pixelSize: _fontPixelSize
                                font.family: _fontFamily
                                color: "black"
                                verticalAlignment: Text.AlignVCenter
                                elide: Text.ElideRight
                            }
                            onActivated: {
                                loginVM.updateIdleTimer()
                                displayText = currentText
                                productionVM.setDryRunValue(currentIndex)
                                master_app.setDryrunMode(currentIndex)
                            }
                            Component.onCompleted: {
                                if(!isCompleted) {
                                    currentIndex = master_app.getDryrunMode()
                                    isCompleted = true
                                }
                            }
                            Connections{
                                target: productionVM
                                ignoreUnknownSignals: true
                                onBypassValueChanged:{
                                    if(!productionVM.bypassValue){  
                                        dryRunCbb.currentIndex = 0
                                        dryRunCbb.enabled = false
                                        dryRunCbb.displayText = dryRunCbb.currentText
                                        productionVM.setDryRunValue(dryRunCbb.currentIndex)
                                        master_app.setDryrunMode(dryRunCbb.currentIndex)
                                    }
                                    else{
                                        dryRunCbb.enabled = true
                                        dryRunCbb.displayText = dryRunCbb.currentText
                                        productionVM.setDryRunValue(dryRunCbb.currentIndex)
                                        master_app.setDryrunMode(dryRunCbb.currentIndex)
                                    }
                                }
                            }
                        }
                    }
                }
                Item{
                    width: parent.width
                    height: parent.height / 5 //productionVM.bypassValue ? parent.height / 5 : parent.height / 4
                    Row{
                        height: parent.height
                        width: parent.width
                        Item{
                            width: parent.width / 2
                            height: parent.height
                            anchors.verticalCenter: parent.verticalCenter
                            Button{
                                id: runLotButton
                                enabled: productionVM.startLotEnabled
                                width: parent.width * 0.95
                                height: parent.height
                                anchors.centerIn: parent
                                text: "Start Lot"
                                contentItem: Text {
                                    text: parent.text
                                    font.pixelSize: _fontPixelSize
                                    font.family: _fontFamily
                                    opacity: enabled ? 1.0 : 0.3
                                    color: parent.down ? "black" : "#21be2b"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    elide: Text.ElideRight
                                }
                                onClicked: {
                                    loginVM.updateIdleTimer()
                                    productionVM.startLotClicked()
                                    isStartLotOpen = false
                                    headVM.clearResult()
                                    pimVM.clearResult()
                                    shipperVM.clearResult()
                                }
                            }
                        }
                        Item{
                            width: parent.width / 2
                            height: parent.height
                            anchors.verticalCenter: parent.verticalCenter
                            Button{
                                id: cancelLotButton
                                // enabled: productionVM.startLotEnabled
                                width: parent.width * 0.95
                                height: parent.height
                                anchors.centerIn: parent
                                text: "Cancel"
                                contentItem: Text {
                                    text: parent.text
                                    font.pixelSize: _fontPixelSize
                                    font.family: _fontFamily
                                    opacity: enabled ? 1.0 : 0.3
                                    color: parent.down ? "black" : "#21be2b"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    elide: Text.ElideRight
                                }
                                onClicked: {
                                    loginVM.updateIdleTimer()
                                    isStartLotOpen = false
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    //Popup Audit Conveyor
    Popup{
        width: 600
        height: 550
        x: mainWindow.width / 2 - width / 2 - control_bar_layout.width
        y: mainWindow.height / 2 - height / 2 - notifyBarArea.height
        visible: isAuditOpen
        onClosed: isAuditOpen = false
        modal: true
        Rectangle{
            id: popupAuditFrame
            anchors.fill: parent
            color: "transparent"
            border{
                width: 3
                color: "black"
            }
            Column{
                spacing: 0
                anchors.fill: parent
                Row{
                    height: parent.height / 6
                    width: parent.width
                    anchors.horizontalCenter: parent.horizontalCenter
                    Text{
                        width: parent.width / 2
                        height: parent.height
                        leftPadding: 10
                        text: "Quantity Setup:"
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        verticalAlignment: Text.AlignVCenter
                    }
                    Item{
                        width: parent.width / 2
                        height: parent.height
                        Rectangle{
                            width: parent.width * 0.9
                            height: parent.height * 0.75
                            color: "#000000"
                            anchors.centerIn: parent
                            border{
                                width: 3
                                color: "gray"
                            }
                            FieldTextWithKeyboard{
                                id: qtySetField
                                onlyNumber: true
                                anchors.fill: parent
                                anchors.centerIn: parent
                                background: null
                                font.pixelSize: _fontPixelSize
                                font.family: _fontFamily
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                                _minimumValue: 1
                                _maximumValue: 10
                                onTextChanged: {
                                    if(text != "")
                                        productionVM.setQuantityValue(parseInt(text))
                                }
                            }
                            Rectangle{
                                visible: qtySetField.visible
                                x: qtySetField.x + qtySetField.width / 2
                                y: qtySetField.y - 70
								width: 400
								height: 40
								color: "black"
								border{
									color: "gray"
									width: 2
								}
								Text{
									anchors.fill: parent
                                	text: "Please insert value between 1-10"
									color: "white"
                                	font.pixelSize: _fontPixelSize - 3
									font.family: _fontFamily
									verticalAlignment: Text.AlignVCenter
									horizontalAlignment: Text.AlignHCenter
								}
                            }
                        }
                    }
                }
                Row{
                    height: parent.height / 6
                    width: parent.width
                    anchors.horizontalCenter: parent.horizontalCenter
                    Text{
                        width: parent.width / 2
                        height: parent.height
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: 10
                        text: "Eject Counter: "
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                    }
                    Text{
                        id: blowNumber
                        width: parent.width / 2
                        height: parent.height
                        text: productionVM.actualQuantity
                        leftPadding: 10
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    Timer{
                        id: makeTextDash
                        interval: 5000
                        onTriggered: {
                            blowNumber.text = " - "
                        }
                    }
                    Connections{
                        target: productionVM
                        ignoreUnknownSignals: true
                        onActualQuantityChanged:{
                            blowNumber.text = productionVM.actualQuantity
                            if(productionVM.actualQuantity === productionVM.quantityValue)
                                makeTextDash.restart()
                        }
                    }
                }
                Row{
                    height: parent.height / 6
                    width: parent.width
                    anchors.horizontalCenter: parent.horizontalCenter
                    Text{
                        width: parent.width / 2
                        height: parent.height
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: 10
                        text: "Total Ejected: "
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                    }
                    Text{
                        width: parent.width / 2
                        height: parent.height
                        text: productionVM.totalBlowouts
                        leftPadding: 10
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignHCenter
                    }
                }
                Row{
                    height: parent.height / 6
                    width: parent.width
                    anchors.horizontalCenter: parent.horizontalCenter
                    Text{
                        width: parent.width / 2
                        height: parent.height
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: 10
                        text: "No. of Audit Performed: "
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                    }
                    Text{
                        width: parent.width / 2
                        height: parent.height
                        text: productionVM.totalAudit
                        leftPadding: 10
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignHCenter
                    }
                }
                Button{
                    id: resetCountButton
                    enabled: productionVM.resetCountEnabled
                    height: parent.height / 6
                    width: 200
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: "Reset Count"
                    contentItem: Text {
                        text: parent.text
                        font.pixelSize: _fontPixelSize
                        font.family: _fontFamily
                        opacity: enabled ? 1.0 : 0.3
                        color: parent.down ? "black" : "#21be2b"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight
                    }
                    onClicked: {
                        loginVM.updateIdleTimer()
                        productionVM.resetCountClicked()
                    }
                }
                Row{
                    height: parent.height / 6
                    width: parent.width
                    anchors.horizontalCenter: parent.horizontalCenter
                    Button{
                        id: startAuditButton
                        enabled: productionVM.startAuditEnabled & qtySetField.text !== "" & qtySetField.text !== "0"
                        height: parent.height
                        width: (parent.width-parent.spacing)/2
                        text: "Start Audit"
                        contentItem: Text {
                            text: parent.text
                            font.pixelSize: _fontPixelSize
                            font.family: _fontFamily
                            opacity: enabled ? 1.0 : 0.3
                            color: parent.down ? "black" : "#21be2b"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            elide: Text.ElideRight
                        }
                        onClicked: {
                            loginVM.updateIdleTimer()
                            productionVM.startAuditClicked()
                            qtySetField.text = ""
                        }
                    }
                    Button{
                        id: endAuditButton
                        enabled: productionVM.endAuditEnabled
                        height: parent.height
                        width: (parent.width-parent.spacing)/2
                        text: "End Audit"
                        contentItem: Text {
                            text: parent.text
                            font.pixelSize: _fontPixelSize
                            font.family: _fontFamily
                            opacity: enabled ? 1.0 : 0.3
                            color: parent.down ? "black" : "#21be2b"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            elide: Text.ElideRight
                        }
                        onClicked: {
                            loginVM.updateIdleTimer()
                            productionVM.endAuditClicked()
                        }
                    }
                }
            }
        }
    }

    
    FileDialogView{
        id: uiDialog
        _extension: [ "Zip files (*.zip)" ]
        onAccepted: {
            var selectedUrl = uiDialog.fileUrl.toString()
            var convert1 = selectedUrl.replace("file://","");
            var convert2 = convert1.replace(" ", "\\ ");
            var uiZipPath = convert2
            var uiZipName = selectedUrl.replace(folder.toString() + "/","")
            var uiName = uiZipName.replace(".zip","")
            settingVM.updateUi(uiZipPath, uiZipName, uiName)
            modalDialogBoxVM.getLoadingPopup("User interface updating");
        }
    }
    Connections{
        target: settingVM
        ignoreUnknownSignals: true
        onEnabledUiBtn:{
            modalDialogBoxVM.getClosePopupLoading();
        }
    }

	//Popup start lot
    StartLotShowup{
        id: startlotConfirm
        x: mainWindow.width  / 2 - width  / 2 - control_bar_layout.width
        y: mainWindow.height / 2 - height / 2 - notifyBarArea.height
		z: 19
    }
    //Popup End Lot Confirm
    EndLotShowup{
        id: endlotConfirm
        x: mainWindow.width  / 2 - width  / 2 - control_bar_layout.width
        y: mainWindow.height / 2 - height / 2 - notifyBarArea.height
		z: 19
    }
	//Popup End Lot process
    EndLotPopup{
        id: endLotPopup
        x: stationListRec.x
        y: stationListRec.y
		z: 19
    }
    //Popup Abort Lot Confirm
    AbortLot{
        id: abortLotConfirm
        x: mainWindow.width  / 2 - width  / 2 - control_bar_layout.width
        y: mainWindow.height / 2 - height / 2 - notifyBarArea.height
		z: 19
    }
    //Popup History Table
    HistoryTable{
        id: historyPopup
        x: mainWindow.width  / 2 - width  / 2 - control_bar_layout.width
        y: mainWindow.height / 2 - height / 2 - notifyBarArea.height
		z: 19
    }
	//Popup for Lot Table
    LotDataTable{
        id: lotDataPopup
        x: mainWindow.width  / 2 - width  / 2 - control_bar_layout.width
        y: mainWindow.height / 2 - height / 2 - notifyBarArea.height
		z: 19
    }
	//Popup station module
    StationLayout {
        id: stationFunction
        visible: false
        width: parent.width
        height: width / 1795 * 1020
    }
	// Waitting initializing
    Popup {
		id: initPopup
		width: 350
		height: 350
		padding: 0
        visible: mainWindow.isInitializing
        x: mainWindow.width / 2 - width / 2 - control_bar_layout.width
        y: mainWindow.height / 2 - height / 2 - notifyBarArea.height
		closePolicy: Popup.NoAutoClose
		// background: Rectangle {
		// 	width: initPopup.width
		// 	height: initPopup.height
		// 	color: "transparent"
		// }
		Rectangle{
			// anchors.centerIn: parent
			// width: 350
			// height: 350
			anchors.fill: parent
			color: "black"
			border{
				width: 5
				color: "gray"
			}

			Text{
				text: "Initializing..."
				font.bold: true
				font.pixelSize: 24
				color: "cyan"
				anchors.horizontalCenter: parent.horizontalCenter
				anchors.top: parent.top
				anchors.topMargin: 20
			}

			AnimatedImage {
				id: gifImage
				anchors.horizontalCenter: parent.horizontalCenter
				anchors.bottom: parent.bottom
				anchors.bottomMargin: 10
				height: 256
				width: 256
				source: ":/../../../Resources/loading.gif"
				onStatusChanged: playing = (status == AnimatedImage.Ready)
			}
		}
	}
}
