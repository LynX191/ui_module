import QtQml 2.3
import QtQuick 2.9
import QtQuick.Window 2.3
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtQuick.Dialogs 1.2

import "../../View"
import "../../Component"
import "../../Component/MaterialDesign"
import "../../Dialog"
import "../FeatureView"


Rectangle {
	id: settingRoot
    width: parent.width
    height: parent.height
    clip: true
    enabled: loginVM.settingEnabled || loginVM.superUserActive
    opacity: enabled? 1 : 0.5
    color: "black"
    property QtObject _settingModel: QtObject{}
    property int _fontPointSize: 10
    property int _fontPixelSize: 14
    property string _fontFamily:  "Adobe Gothic Std B"
    property var camsListID: settingVM.camsListID
	property var xavierResources: master_app.xavierResources
	function getXavierValue(xavier, data) {
		var split
		if(typeof xavierResources[xavier] !== "undefined"){
			split = xavierResources[xavier].split("-")
			return parseInt(split[data])
		}
		else{
			return 1
		}
	}

    property var permissionList: [
        { name: "Control Master"},
        { name: "Production & Control"},
        { name: "IO Control"},
        { name: "Settings"},
        { name: "System Log"},
        { name: "Parameters"}
    ]
	property var camList: [
        { name: "Cam 1", currentId: camsListID[0]},
        { name: "Cam 2", currentId: camsListID[1]},
        { name: "Cam 3", currentId: camsListID[2]},
    ]
    ListModel{
        id:settingList
        ListElement{tabName:"General"               ;tabIcon:"cogs"                }
        ListElement{tabName:"Access Rights"         ;tabIcon:"account-edit"        }
        ListElement{tabName:"Camera ID"       		;tabIcon:"camera-iris"         }
        ListElement{tabName:"Backup and Recovery"   ;tabIcon:"backup-restore"      }
        ListElement{tabName:"About"                 ;tabIcon:"information-outline" }
        ListElement{tabName:"Disk Info"             ;tabIcon:"harddisk" }
    }
    Rectangle{
        id:settingLeftView
        anchors.left: parent.left
        anchors.top: parent.top
        height: parent.height
        width: parent.width*0.25
        color: "#201E1F"
        clip: true
        ListView{
            id:settingListView
            anchors.fill: parent
            anchors.rightMargin: 16
            anchors.leftMargin: 25
            model:settingList
            property var currentTab: 0
            header: Item{
                width: parent.width
                height: 80
                Text {
                    text: qsTr("Settings")
                    anchors.bottom: parent.bottom
                    anchors.bottomMargin: 20
                    anchors.left: parent.left
                    anchors.leftMargin: 20
                    font.pointSize: 18
                    font.bold: true
                    color: "white"
                }
            }
            delegate: Button {
                id:settingbtn
                width: parent.width
                height: index===0 && !loginVM.superUserActive?0:implicitHeight+20
                visible: index===0 && !loginVM.superUserActive?false:true
                Row{
                    anchors.verticalCenter: parent.verticalCenter
                    leftPadding: 20
                    spacing: 15
                    MaterialDesignIcon {
                        id: icon
                        name: tabIcon
                        color: settingbtn.down ? "gray" : "white"
                        size: 24
                        scale: settingbtn.pressed ? 0.8 : 1.0
                        anchors.verticalCenter: parent.verticalCenter
                        Behavior on scale {
                           NumberAnimation {
                               duration: 200
                           }
                        }
                    }
                Text{
                    text: tabName
                    font.pointSize: 12
                    opacity: enabled ? 1.0 : 0.3
                    color: settingbtn.down ? "gray" : "white"
                    anchors.verticalCenter: parent.verticalCenter
                    elide: Text.ElideRight
                }
                }
                background: Rectangle {
                    id:btnBackground
                    anchors.fill:parent
                    opacity: enabled ? 1 : 0.3
                    color: settingListView.currentTab===index?"#3b3b3b": settingbtn.hovered ? settingbtn.down ? "#26282e" : "#3b3b3b" : "#202020"
                    border.color: "#202020"
                    border.width: 1.5
                    radius: 7
                }
                onClicked: {
                    loginVM.updateIdleTimer()
                    switch(index) {
                      case 0:   settingView.contentY=generalHeader.y
                        break;
                      case 1:   settingView.contentY=accessRightsHeader.y
                        break;
					  case 2:   settingView.contentY=cameraHeader.y
                        break;
                      case 3:   settingView.contentY=backupHeader.y
                        break;
                      case 4:   settingView.contentY=aboutHeader.y
                        break;
                      case 5:   settingView.contentY=diskHeader.y
                        break;
                      default:
                        break;
                    }
                }
            }
        }
    }

    Flickable{
        id: settingView
        anchors.left: settingLeftView.right
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.leftMargin: 50
        anchors.rightMargin: 20
        contentWidth: itemscontent.width
        contentHeight: itemscontent.height
        boundsBehavior: Flickable.StopAtBounds
        ScrollBar.vertical: ScrollBar{}
        Component.onCompleted: {
            updateSettingListView();
        }
        onContentYChanged: {
            updateSettingListView();
        }
        onContentHeightChanged: {
            updateSettingListView();
        }
        function updateSettingListView(){
            if (contentY < accessRightsHeader.y){
                settingListView.currentTab=0
            }else if(contentY < cameraHeader.y){
                settingListView.currentTab=1
            }else if(contentY < backupHeader.y){
                settingListView.currentTab=2
            }else if(contentY < aboutHeader.y){
                settingListView.currentTab=3
            }else if(contentY < diskHeader.y){
                settingListView.currentTab=4
            }else{
                settingListView.currentTab=5
            }
        }
        Column{
            id: itemscontent
            width: settingView.width - 300
            spacing: 10

            ////---------- General -----------//
            Item{
                id: generalHeader
                width: parent.width
                height: 20
                visible: loginVM.superUserActive
            }
            Item{
                Layout.alignment:Qt.AlignLeft
                width:parent.width
                height:40
                visible: loginVM.superUserActive
                Text {
                    text: settingList.get(0).tabName
                    font.family: "Adobe Gothic Std B"
                    font.pointSize: 15
                    font.bold: true
                    anchors.verticalCenter: parent.verticalCenter
                    style: Text.Sunken
                    color: "white"
                }
            }
            GroupBox{
                id: generalGrb
                width: parent.width
                visible: loginVM.superUserActive
                background:Rectangle{
                    color:"#201E1F"
                    radius: 10
                }
                Column{
                    Row{
                        width: implicitWidth
                        height: implicitHeight
                        spacing: 20
                        Item{
                            width: generalGrb.width / 3
                            height: 50
                            Text{
                                text: "System Log Font"
                                color: "white"
                                width: contentWidth
                                height: contentHeight
                                anchors.fill: parent
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                font.pointSize: 12
                            }
                        }
                        ComboBox{
                            id: fontComboBox
                            property bool isCompleted: false
                            width: 350
                            height: 50
                            enabled: true
                            opacity: enabled? 1 :0.4
                            wheelEnabled: true
                            textRole: "fontName"
                            model: fontModel
                            onCurrentIndexChanged: {
                                loginVM.updateIdleTimer()
                                if (isCompleted) {
                                    var selectedFont = fontModel.get(currentIndex).fontName;
                                    settingVM.updateFontCmd(selectedFont)
                                    systemLogVM.getConfigTerminalChanged()
                                    displayText = selectedFont;
                                }
                            }
                            Component.onCompleted: {
                                var fontFamilies = Qt.fontFamilies();
                                for (var i = 0; i < fontFamilies.length; ++i) {
                                    fontModel.append({ fontName: fontFamilies[i] });
                                    if(settingVM.systemLogFont === fontFamilies[i]){
                                        currentIndex = i
                                        displayText = fontFamilies[i]
                                    }
                                }
                                isCompleted = true;
                            }
                        }
                        ListModel {
                            id: fontModel
                        }
                    }
                    Row{
                        width: implicitWidth
                        height: implicitHeight
                        spacing: 20
                        Item{
                            width: generalGrb.width / 3
                            height: 50
                            Text{
                                text: "System Log Text Size"
                                color: "white"
                                width: contentWidth
                                height: contentHeight
                                anchors.fill: parent
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                font.pointSize: 12
                            }
                        }
                        ComboBox{
                            id: textSizeComboBox
                            property bool isCompleted: false
                            width: 350
                            height: 50
                            enabled: true
                            opacity: enabled? 1 :0.4
                            wheelEnabled: true
                            textRole: "fontSize"
                            model: textSizeModel
                            onCurrentIndexChanged: {
                                loginVM.updateIdleTimer()
                                if (isCompleted) {
                                    var selectedSize = textSizeModel.get(currentIndex).fontSize;
                                    settingVM.updateSizeCmd(parseInt(selectedSize))
                                    systemLogVM.getConfigTerminalChanged()
                                    displayText = selectedSize;
                                }
                            }
                            Component.onCompleted: {
                                var textSizes = [10,12,14,16,18,20,22,24,28,32,48]
                                for (var i = 0; i < textSizes.length; ++i) {
                                    textSizeModel.append({ fontSize: textSizes[i] });
                                    if(settingVM.systemLogSize === textSizes[i]){
                                        currentIndex = i
                                        displayText = textSizes[i]
                                    }
                                }
                                isCompleted = true;
                            }
                        }
                        ListModel {
                            id: textSizeModel
                        }
                    }
                    Row{
                        width: implicitWidth
                        height: implicitHeight
                        spacing: 20
                        Item{
                            width: generalGrb.width / 3
                            height: 50
                            Text{
                                text: "System Log Line Spacing"
                                color: "white"
                                width: contentWidth
                                height: contentHeight
                                anchors.fill: parent
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                font.pointSize: 12
                            }
                        }
                        ComboBox{
                            id: lineSpacingComboBox
                            property bool isCompleted: false
                            width: 350
                            height: 50
                            enabled: true
                            opacity: enabled? 1 :0.4
                            wheelEnabled: true
                            textRole: "spacing"
                            model: lineSpacingModel
                            onCurrentIndexChanged: {
                                loginVM.updateIdleTimer()
                                if (isCompleted) {
                                    var selectedSpacing = lineSpacingModel.get(currentIndex).spacing;
                                    settingVM.updateSizeSpacing(parseInt(selectedSpacing))
                                    systemLogVM.getConfigTerminalChanged()
                                    displayText = selectedSpacing;
                                }
                            }
                            Component.onCompleted: {
                                var lineSpacing = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
                                for (var i = 0; i < lineSpacing.length; ++i) {
                                    lineSpacingModel.append({ spacing: lineSpacing[i] });
                                    if(settingVM.systemLogSpacing === lineSpacing[i]){
                                        currentIndex = i
                                        displayText = lineSpacing[i]
                                    }
                                }
                                isCompleted = true;
                            }
                        }
                        ListModel {
                            id: lineSpacingModel
                        }
                    }
                    Row{
                        width: implicitWidth
                        height: implicitHeight
                        spacing: 20
                        Item{
                            width: generalGrb.width / 3
                            height: 50
                            Text{
                                text: "System Log Limit Line"
                                color: "white"
                                width: contentWidth
                                height: contentHeight
                                anchors.fill: parent
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                font.pointSize: 12
                            }
                        }
                        ComboBox{
                            id: limitLineComboBox
                            property bool isCompleted: false
                            width: 350
                            height: 50
                            enabled: true
                            opacity: enabled? 1 :0.4
                            wheelEnabled: true
                            textRole: "limitLine"
                            model: limitLineModel
                            onCurrentIndexChanged: {
                                loginVM.updateIdleTimer()
                                if (isCompleted) {
                                    var limitLine = limitLineModel.get(currentIndex).limitLine;
                                    settingVM.updateLimitLine(parseInt(limitLine))
                                    systemLogVM.getConfigTerminalChanged()
                                    displayText = limitLine;
                                }
                            }
                            Component.onCompleted: {
                                var limitLine = [500,1000,1500,2000,2500,3000,3500,4000,4500,5000,5500,6000,6500,7000,7500,8000,8500,9000,9500,10000]
                                for (var i = 0; i < limitLine.length; ++i) {
                                    limitLineModel.append({ limitLine: limitLine[i] });
                                    if(settingVM.systemLogLimit === limitLine[i]){
                                        currentIndex = i
                                        displayText = limitLine[i]
                                    }
                                }
                                isCompleted = true;
                            }
                        }
                        ListModel {
                            id: limitLineModel
                        }
                    }
                    Row{
                        width: implicitWidth
                        height: implicitHeight
                        spacing: 20
                        Item{
                            width: generalGrb.width / 3
                            height: 50
                            Text{
                                text: "System Log Line In One Scroll"
                                color: "white"
                                width: contentWidth
                                height: contentHeight
                                anchors.fill: parent
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                font.pointSize: 12
                            }
                        }
                        ComboBox{
                            id: scrollLineComboBox
                            property bool isCompleted: false
                            width: 350
                            height: 50
                            enabled: true
                            opacity: enabled? 1 :0.4
                            wheelEnabled: true
                            textRole: "scrollLine"
                            model: scrollLineModel
                            onCurrentIndexChanged: {
                                loginVM.updateIdleTimer()
                                if (isCompleted) {
                                    var scrollLine = scrollLineModel.get(currentIndex).scrollLine;
                                    settingVM.updateSystemLogScroll(parseInt(scrollLine))
                                    systemLogVM.getConfigTerminalChanged()
                                    displayText = scrollLine;
                                }
                            }
                            Component.onCompleted: {
                                var scrollLine = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
                                for (var i = 0 ; i < scrollLine.length; ++i) {
                                    scrollLineModel.append({ scrollLine: scrollLine[i] });
                                    if(settingVM.systemLogScroll === scrollLine[i]){
                                        currentIndex = i
                                        displayText = scrollLine[i]
                                    }
                                }
                                isCompleted = true;
                            }
                        }
                        ListModel {
                            id: scrollLineModel
                        }
                    }
					Row{
                        width: implicitWidth
                        height: implicitHeight
                        spacing: 20
                        Item{
							id: freeSpaceItem
                            width: generalGrb.width / 3
                            height: 50
                            Text{
                                text: "Free space: " + settingVM.freeSpace + " (GB)"
                                color: "white"
                                width: contentWidth
                                height: contentHeight
                                anchors.fill: parent
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                font.pointSize: 12
                            }
                        }
						FieldTextWithKeyboard{
							id: freeSpaceTextField
							onlyNumber: true
							width: 350
							height: 50
							readOnly: false
							font.pixelSize: _fontPixelSize
							font.family: _fontFamily
							horizontalAlignment: Text.AlignHCenter
							verticalAlignment: Text.AlignVCenter
							selectByMouse: false
							maximumLength: 18
							placeholderText: "Enter space want to available (GB)"
							opacity: enabled? 1 : 0.5
							validator: RegExpValidator { regExp: /^[0-9]*$/ }
							_minimumValue: 1
							_maximumValue: 256
							onEnterKeyPress:{
								freeSpaceTextField.focus = false
								settingVM.updateFreeSpace(parseInt(text));
								text = ""
							}
						}
                    }
                }
            }

            ////-------- Set Permission ------------//
            Item{
                id: accessRightsHeader
                width: parent.width
                height: 10
            }
            Item{
                Layout.alignment:Qt.AlignLeft
                width:parent.width
                height:40
                Column{
                    Text {
                        text: settingList.get(1).tabName
                        font.family: "Adobe Gothic Std B"
                        font.pointSize: 15
                        font.bold: true
                        style: Text.Sunken
                        color: "white"
                    }
                }
            }
            GroupBox{
                id: accessRightGrb
                width: parent.width
                height: permisstionCln.height + 25
                background: Rectangle{
                    color: "#201E1F"
                    radius: 10
                }
                Column{
                    id: permisstionCln
                    width: parent.width - applyPermission.width
                    height: implicitHeight
                    Repeater{
                        id: permissionRpt
                        model:permissionList
                        delegate:Row{
                            width: implicitWidth
                            height: implicitHeight
                            spacing: 20
                            Item{
                            width: accessRightGrb.width / 3
                                height: 50
                                Text {
                                    text: modelData.name
                                    color: "white"
                                    width: contentWidth
                                    height: contentHeight
                                    anchors.fill: parent
                                    horizontalAlignment: Text.AlignLeft
                                    verticalAlignment: Text.AlignVCenter
                                    font.pointSize: 12
                                }
                            }
                            ComboBox{
                                id: userAccessCombobox
                                property bool isCompleted: false
                                property var permissionModel:["Administrator","Technician","Operator"]
                                width: 350
                                height: 50
                                enabled: true
                                opacity: enabled? 1 :0.4
                                onCurrentTextChanged: {
                                    loginVM.updateIdleTimer()
                                    if (isCompleted) {
                                        displayText = currentText;
                                        applyPermission.visible = true
                                    }
                                }
                                Component.onCompleted: {
                                    model = permissionModel
                                    currentIndex = eval("settingVM.enabledIndex" + index) - 1
                                    displayText = permissionModel[eval("settingVM.enabledIndex" + index) - 1]
                                    isCompleted = true;
                                }
                            }
                        }
                    }
                }
                Button{
                    id: applyPermission
                    width: 96
                    height: 48
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    text: "Apply"
                    visible: false
                    contentItem: Text {
                        text: parent.text
                        font.pixelSize: _fontPixelSize + 2
                        font.family: _fontFamily
                        color: "white"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight
                    }
                    onClicked: {
                        for(var i = 0; i < permissionRpt.count; i++){
                            settingVM.updatePermissionData(i, permissionRpt.itemAt(i).children[1].currentIndex);
                        }
						loginVM.setPermission()
                        visible = false
                    }
                }
            }

			////------- Camera --------------------//
			Item{
                id: cameraHeader
                width: parent.width
                height: 10
            }
			Item{
                Layout.alignment:Qt.AlignLeft
                width:parent.width
                height:40
                Column{
                    Text {
                        text: settingList.get(2).tabName
                        font.family: "Adobe Gothic Std B"
                        font.pointSize: 15
                        font.bold: true
                        style: Text.Sunken
                        color: "white"
                    }
                }
            }
			GroupBox{
                id: camGrb
                width: parent.width
                height: cameraCln.height + 25
                background: Rectangle{
                    color: "#201E1F"
                    radius: 10
                }
                Column{
                    id: cameraCln
                    width: parent.width - resetIdCam.width
                    height: implicitHeight
                    Repeater{
                        id: camRpt
                        model:camList
                        delegate:Row{
                            width: implicitWidth
                            height: implicitHeight
                            spacing: 10
                            Item{
                            	width: camGrb.width / 3 * 0.33
                                height: 50
                                Text {
                                    text: modelData.name
                                    color: "white"
                                    width: parent.width
                                    height: contentHeight
                                    anchors.fill: parent
                                    horizontalAlignment: Text.AlignLeft
                                    verticalAlignment: Text.AlignVCenter
                                    font.pointSize: 12
                                }
                            }
                            Item{
                            	width: camGrb.width / 3 * 0.67
                                height: 50
                                Text {
                                    id: textCam
                                    text: modelData.currentId
                                    color: "white"
                                    width: parent.width
                                    height: contentHeight
                                    anchors.fill: parent
                                    horizontalAlignment: Text.AlignLeft
                                    verticalAlignment: Text.AlignVCenter
                                    font.pointSize: 12
                                    function resetGetData(){
                                        text = modelData.currentId
                                    }
                                }
                            }
							FieldTextWithKeyboard{
                                id: camTextField
								onlyNumber: false
                                width: 350
                                height: 50
								readOnly: false
								font.pixelSize: _fontPixelSize
								font.family: _fontFamily
								horizontalAlignment: Text.AlignHCenter
								verticalAlignment: Text.AlignVCenter
								selectByMouse: false
								maximumLength: 18
								placeholderText: "Enter Mx identification code (18 characters)"
								opacity: enabled? 1 : 0.5
    							validator: RegExpValidator { regExp: /^[a-zA-Z0-9]*$/ }
                                onTextChanged:{
                                    if(textCam.text !== text )
                                        if(text != ""){
                                            applyIdCam.visible = true
                                            cancelIdCam.visible = true
                                        }
                                }
								onEnterKeyPress:{
                                    applyIdCam.visible = false
                                    cancelIdCam.visible = false
                                    if(text.length < 18){
                                        modalDialogBoxVM.getModalDialogQML(false, "id_invalid");
                                            camTextField.text = ""
                                        return
                                    }
									settingVM.updateCamMxId(modelData.currentId, text);
                                    camTextField.text = ""
								}
							}
                            ButtonMaterial{
                                id: applyIdCam
                                _width: 36
                                _height: _width
                                _size: _width
                                _iconSourceOn: "check-outline"
                                _isClicked: master_app.powerUpState
                                _colorOverlayLow: "#73ff66"
                                _enableEffect: false
                                opacity: enabled? 1:0.5
                                visible: false
                                anchors.verticalCenter: parent.verticalCenter
                                onClicked: {
                                    loginVM.updateIdleTimer()
                                    visible = false
                                    cancelIdCam.visible = false
                                    if(camTextField.text.length < 18){
                                        modalDialogBoxVM.getModalDialogQML(false, "id_invalid");
                                        camTextField.text = ""
                                        return
                                    }
									settingVM.updateCamMxId(modelData.currentId, camTextField.text);
                                    camTextField.text = ""
                                }
                            }
                            ButtonMaterial{
                                id: cancelIdCam
                                _width: 36
                                _height: _width
                                _size: _width
                                _iconSourceOn: "close-outline"
                                _isClicked: master_app.powerUpState
                                _colorOverlayLow: "red"
                                _enableEffect: false
                                opacity: enabled? 1:0.5
                                visible: false
                                anchors.verticalCenter: parent.verticalCenter
                                onClicked: {
                                    loginVM.updateIdleTimer()
                                    visible = false
                                    applyIdCam.visible = false
                                    camTextField.text = ""
                                }
                            }
						}
					}
                    Item{
                        width: parent.width
                        height: 50
                        Button{
                            id: imagingUpdateBtn
                            width: 200
                            height: 50
                            anchors.centerIn: parent
                            text: "Imaging Update"
                            opacity: enabled? 1:0.5
                            enabled: !master_app.powerUpState
                            contentItem: Text {
                                text: parent.text
                                font.pixelSize: _fontPixelSize + 2
                                font.family: _fontFamily
                                color: "white"
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                                elide: Text.ElideRight
                            }
                            onClicked: {
                                fileDialog.open()
                            }
                            FileDialog {
                                id: fileDialog
                                width: 1000
                                height: 600
                                folder: "/home/"
                                selectMultiple: false
                                selectFolder: false
                                nameFilters: [ "Zip files (*.zip)" ]
                                onAccepted: {
                                    var selectedUrl = fileDialog.fileUrl.toString()
                                    var convert1 = selectedUrl.replace("file://","");
                                    var convert2 = convert1.replace(" ", "\\ ");
                                    var imagingZipPath = convert2
                                    var imagingZipName = selectedUrl.replace(folder.toString() + "/","")
                                    var imagingName = imagingZipName.replace(".zip","")
                                    if(imagingName.indexOf("imaging") != -1){
                                        settingVM.updateImaging(imagingZipPath, imagingZipName, imagingName)
                                        imagingUpdateBtn.enabled = false && !master_app.powerUpState
                                        modalDialogBoxVM.getLoadingPopup("Imaging updating");
                                    }
                                    else{
                                        modalDialogBoxVM.getModalDialogQML(false, "imaging_wrong");
                                    }
                                }
                            }
                            Connections{
                                target: settingVM
                                ignoreUnknownSignals: true
                                onEnabledImagingBtn:{
                                    imagingUpdateBtn.enabled = true && !master_app.powerUpState
			                        modalDialogBoxVM.getClosePopupLoading();
                                }
                            }
                        }
                        // AnimatedImage {
                        //     id: loadImage
                        //     anchors.verticalCenter: parent.verticalCenter
                        //     anchors.left: imagingUpdateBtn.right
                        //     anchors.leftMargin: 20
                        //     visible: false
                        //     height: 40
                        //     width: height
                        //     source: ":/../../../Resources/loading.gif"
                        //     onStatusChanged: playing = (status == AnimatedImage.Ready)
                        // }
                    }
				}
                ButtonMaterial{
                    id: changeDefaultId
                    _width: 48
                    _height: _width
                    _size: _width
                    _iconSourceOn: "content-save-all"
                    _colorOverlayLow: "#ffffff"
                    _enableEffect: false
                    opacity: enabled? 1:0.5
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.right: resetIdCam.left
                    anchors.rightMargin: 20
                    onClicked: {
                        loginVM.updateIdleTimer()
                        settingVM.updateDefaultCamID()
                    }
                }
                ButtonMaterial{
                    id: resetIdCam
                    _width: 48
                    _height: _width
                    _size: _width
                    _iconSourceOn: "restore"
                    _colorOverlayLow: "#ffffff"
                    _enableEffect: false
                    opacity: enabled? 1:0.5
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.right: parent.right
                    anchors.rightMargin: 10
                    onClicked: {
                        loginVM.updateIdleTimer()
                        settingVM.resetToDefaultCamID()
                    }
                }
			}
            ////--------- Backup and restore -----------//
            Item{
                id: backupHeader
                width: parent.width
                height: 10
            }
            Item{
                Layout.alignment:Qt.AlignLeft
                width:parent.width
                height:40
                Text {
                    text: settingList.get(3).tabName
                    font.family: "Adobe Gothic Std B"
                    font.pointSize: 15
                    font.bold: true
                    anchors.verticalCenter: parent.verticalCenter
                    style: Text.Sunken
                    color: "white"
                }
            }
            GroupBox{
                id: backupGroupBox
                width: parent.width
                background: Rectangle{
                    color: "#201E1F"
                    radius: 10
                }
                Row{
                    width: parent.width
                    height: 120
                    anchors.centerIn: parent
                    opacity: enabled ? 1 : 0.5
                    enabled: !master_app.powerUpState
                    Item{
                        width: parent.width / 2
                        height: 50
                        anchors.verticalCenter: parent.verticalCenter
                        Button{
                            text: enabled ? "Backup" : "Backup (Not avaiable)"
                            width: parent.width / 1.5
                            height: 50
                            anchors.centerIn: parent
                            contentItem: Text{
                                text: parent.text
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                                font.pixelSize: _fontPixelSize
                                font.family: _fontFamily
                                color: enabled ? "white" : "red"
								font.bold: enabled
                            }
                            onClicked: {
                                loginVM.updateIdleTimer()
                                saveAsPopup.visible = true
                            }
                        }
                    }
                    Item{
                        width: parent.width / 2
                        height: 50
                        anchors.verticalCenter: parent.verticalCenter
                        Button{
                            text: enabled ? "Recovery" : "Recovery (Not avaiable)"
                            width: parent.width / 1.5
                            height: 50
                            anchors.centerIn: parent
                            contentItem: Text{
                                text: parent.text
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                                font.pixelSize: _fontPixelSize
                                font.family: _fontFamily
                                color: enabled ? "white" : "red"
								font.bold: enabled
                            }
                            onClicked: {
                                loginVM.updateIdleTimer()
                                savedListDialog.open()
                            }
                        }
                    }
                }
            }
            ////--------- About Us  -----------//
            Item{
                id: aboutHeader
                width: parent.width
                height: 10
            }
            Item{
                Layout.alignment:Qt.AlignLeft
                width:parent.width
                height:40
                Text {
                    text: settingList.get(4).tabName
                    font.family: "Adobe Gothic Std B"
                    font.pointSize: 15
                    font.bold: true
                    anchors.verticalCenter: parent.verticalCenter
                    style: Text.Sunken
                    color: "white"
                }
            }
            GroupBox{
                id: aboutGrb
                width: parent.width
                height: aboutImage.height + 30
                background: Rectangle{
                    color: "#201E1F"
                    radius: 10
                }
                Image{
                    id: aboutImage
                    width: parent.width
                    height: width / 860*871
                    source: "qrc:/Resources/About.png"
                    fillMode: Image.PreserveAspectFit
                }
            }
            ////--------- Disk Info  -----------//
            Item{
                id: diskHeader
                width: parent.width
                height: 10
            }
            Item{
                Layout.alignment:Qt.AlignLeft
                width:parent.width
                height:40
                Text {
                    text: settingList.get(5).tabName
                    font.family: "Adobe Gothic Std B"
                    font.pointSize: 15
                    font.bold: true
                    anchors.verticalCenter: parent.verticalCenter
                    style: Text.Sunken
                    color: "white"
                }
            }
            GroupBox{
                id: diskGrb
                width: parent.width
                height: diskContent.height + 30 + syncArea.height
                background: Rectangle{
                    color: "#201E1F"
                    radius: 10
                }
				Grid{
					id: diskContent
					width: parent.width
					height: 800
					columns: 2
					rows: 2
					Repeater{
						model: 4
						delegate: Column{
							id: diskColumn
							width: diskContent.width / 2
							height: 400
							// Define disk usage properties
							property int totalDiskSpace: getXavierValue(index, 0) // Total disk space in GB
							property int usedDiskSpace: getXavierValue(index, 1)  // Used disk space in GB
							property int freeDiskSpace: getXavierValue(index, 2)  // Total disk space in GB
							property int usedNoneCal: getXavierValue(index, 3)  // Total disk space in GB
							property real usedPercentage: totalDiskSpace !== 0 ? usedDiskSpace / totalDiskSpace : 0
							function state(){
								if(usedPercentage < 0.5){
									return "green"
								}
								else if(usedPercentage >= 0.5 && usedPercentage < 0.85)
									return "orange"
								else if(usedPercentage >= 0.85)
									return "red"
							}
							onUsedDiskSpaceChanged:{
								diskCanvas.requestPaint()
							}
							Canvas {
								id: diskCanvas
								width: 300
								height: 300
								anchors.horizontalCenter: parent.horizontalCenter
								onPaint: {
									var ctx = getContext("2d");
									ctx.clearRect(0, 0, width, height); // Clear canvas

									// Calculate circle parameters
									var radius = Math.min(width, height) / 4;
									var centerX = width / 2;
									var centerY = height / 2;

									// Draw total disk space circle
									ctx.beginPath();
									ctx.arc(centerX, centerY, radius, -Math.PI/2, 2 * Math.PI - Math.PI/2);
									ctx.strokeStyle = "lightgray";
									ctx.lineWidth = 150;
									ctx.stroke();

									// Draw used disk space circle
									var endAngle = 2 * Math.PI * usedPercentage - Math.PI/2;
									ctx.beginPath();
									ctx.arc(centerX, centerY, radius, -Math.PI/2, endAngle);
									ctx.strokeStyle = diskColumn.state()
									ctx.lineWidth = 150;
									ctx.stroke();

									// Draw space in center
									var centerRadius = radius * 0.4; // Adjust as needed for the size of the center space
									ctx.beginPath();
									ctx.arc(centerX, centerY, centerRadius, 0, 2 * Math.PI);
									ctx.strokeStyle = "white";
									ctx.lineWidth = 60;
									ctx.stroke();
								}
								Text{
									anchors.centerIn: parent
									text: totalDiskSpace > 1 ? "Xavier " + (index + 1) : "Xavier " + (index + 1) +"\nDisconnected"
									color: totalDiskSpace > 1 ? "black":"red"
									horizontalAlignment: Text.AlignHCenter
									verticalAlignment: Text.AlignVCenter
									font.pointSize: 12
									font.family: "Adobe Gothic Std B"
								}
							}

							Column{
								width: 300
								height: 100
								anchors.horizontalCenter: parent.horizontalCenter
								Row{
									width: 300
									height: 50
									Item{
										width: 100
										height: 50
										anchors.verticalCenter: parent.verticalCenter
										Rectangle{
											width: 30
											height: 30
											color: "lightgray"
											anchors.centerIn: parent
										}
									}
									Item{
										width: 200
										height: 50
										anchors.verticalCenter: parent.verticalCenter
										Text{
											anchors.left: parent.left
											anchors.verticalCenter: parent.verticalCenter
											text: typeof diskColumn.freeDiskSpace !== "undefined"? "Free Space: " + diskColumn.freeDiskSpace + "GB" : "???"
											horizontalAlignment: Text.AlignHCenter
											verticalAlignment: Text.AlignVCenter
											color: "white"
											font.pointSize: 12
                    						font.family: "Adobe Gothic Std B"
										}
									}
								}
								Row{
									width: 300
									height: 50
									Item{
										width: 100
										height: 50
										anchors.verticalCenter: parent.verticalCenter
										Rectangle{
											width: 30
											height: 30
											color: typeof diskColumn.state() !== "undefined"? diskColumn.state() : "white"
											anchors.centerIn: parent
										}
									}
									Item{
										width: 200
										height: 50
										anchors.verticalCenter: parent.verticalCenter
										Text{
											anchors.left: parent.left
											anchors.verticalCenter: parent.verticalCenter
											text: typeof diskColumn.usedDiskSpace !== "undefined"? "Used: " + diskColumn.usedDiskSpace + "GB (" + diskColumn.usedNoneCal +"%)" : "???"
											horizontalAlignment: Text.AlignHCenter
											verticalAlignment: Text.AlignVCenter
											color: "white"
											font.pointSize: 12
                    						font.family: "Adobe Gothic Std B"
										}
									}
								}
							}
						}
					}
				}
                Item{
                    id: syncArea
                    width: parent.width
                    height: 75
                    opacity: enabled? 1:0.5
                    anchors.top: diskContent.bottom
                    Row{
                        height: 50
                        anchors.centerIn: parent
                        spacing: 10
                        Button{
                            id: syncButton
                            width: 200
                            height: 50
                            contentItem: Text{
                                id: folderText
                                text: "Synchronize Time"
                                color: "white"
                                font.pointSize: 14
                                font.family: "Arial"
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                            }
                            onClicked: {
                                settingVM.startSyncTime()
                                // syncTimePopup.visible = true
                            }
                        }
                        Button{
                            id: resetButton
                            width: 200
                            height: 50
                            opacity: enabled? 1:0.5
                            property string xavierVector
                            contentItem: Text{
                                id: resetText
                                text: "Restart Xavier"
                                color: "white"
                                font.pointSize: 14
                                font.family: "Arial"
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                            }
                            onClicked: {
                                if(xavierVector !== ""){
                                    modalDialogBoxVM.popupModalDialog(true, -2, "xavier_restart", "Reset Xavier "+ resetButton.xavierVector + " to fix synchronization error - ")        
                                }
                                else{
                                    modalDialogBoxVM.popupModalDialog(false, -1, "xavier_restart", "No synchronization errors detected on any Xavier devices - No restart required at this time")
                                }
                            }
                            Connections{
                                target: master_app
                                ignoreUnknownSignals: true
                                onXavierWrongTime:{
                                    var currentVector = value
                                    resetButton.xavierVector = resetButton.xavierVector + "["
                                    for(var i =0; i <=2; i++){
                                        if(value[i] === "1"){
                                            var text = i + 1
                                            resetButton.xavierVector = resetButton.xavierVector + text
                                            if(i < 2)
                                            resetButton.xavierVector = resetButton.xavierVector + "-"
                                        }
                                    }
                                    resetButton.xavierVector = resetButton.xavierVector + "]"
                                }
                                onAcceptRestart:{
                                    resetButton.xavierVector = ""
                                    controlBarVM.powerEnabled = false
                                }
                                onSendPowerEnabled:{
                                    controlBarVM.powerEnabled = value
                                }
                            }
                        }
                        Button{
                            id: installButton
                            width: 200
                            height: 50
                            enabled: !master_app.powerUpState
                            contentItem: Text{
                                id: installText
                                text: "Installation"
                                color: "white"
                                font.pointSize: 14
                                font.family: "Arial"
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                            }
                            onClicked: {
                                controlDialog.visible = true
                            }
                            FileDialog {
                                id: controlDialog
                                width: 1000
                                height: 600
                                folder: "/home/"
                                selectMultiple: false
                                selectFolder: false
                                nameFilters: [ "Zip files (*.zip)" ]
                                onAccepted: {
                                    var selectedUrl = controlDialog.fileUrl.toString()
                                    var convert1 = selectedUrl.replace("file://","");
                                    var convert2 = convert1.replace(" ", "\\ ");
                                    var controlZipPath = convert2
                                    var controlZipName = selectedUrl.replace(folder.toString() + "/","")
                                    var controlName = controlZipName.replace(".zip","")
                                    settingVM.updateControl(controlZipPath, controlZipName, controlName)
                                    modalDialogBoxVM.getLoadingPopup("Control updating");
                                }
                            }
                            Connections{
                                target: settingVM
                                ignoreUnknownSignals: true
                                onEnabledControlBtn:{
			                        modalDialogBoxVM.getClosePopupLoading();
                                }
                            }
                        }
                    }
                    Connections{
                        target: master_app
                        ignoreUnknownSignals: true
                        onPowerUpStateChanged:{
                            syncArea.enabled = !state
                        }
                    }
                }
            }
			// Bonus
            Item{
                width: parent.width
                height: 50
            }
        }
    }
    SavedListDialog{
        id: savedListDialog
        visible: false
    }

    // Dialog {
    //     id: resetDefaultDialog
    //     width: 500
    //     height: 150
    //     // x: parent.x + parent.width / 2 - width / 2
    //     // y: parent.y + parent.height / 2 - height / 2
    //     title: "Are you sure you want to reset to defaults setting?"
    //     standardButtons: Dialog.Cancel | Dialog.Ok
    //     // modal: true
    //     onAccepted: {
    //         loginVM.updateIdleTimer()
    //         settingVM.resetToDefault()
    //         resetDefaultDialog.close()
    //     }
    //     onRejected: {
    //         loginVM.updateIdleTimer()
    //         resetDefaultDialog.close()
    //     }
    // }
    InputFileNameDialog{
        id: saveAsPopup
        x: parent.width  / 2 - width  / 2
        y: parent.height / 2 - height / 2
    }
    SyncTime{
        id: syncTimePopup
        x: parent.width  / 2 - width  / 2
        y: parent.height / 2 - height / 2
    }
    Installation{
        id: installPopup
        x: parent.width  / 2 - width  / 2
        y: parent.height / 2 - height / 2
    }
}
