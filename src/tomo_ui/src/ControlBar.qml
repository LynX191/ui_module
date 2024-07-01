import QtQml 2.2
import QtQuick 2.9
import QtQuick.Window 2.3
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

//My import
import "View"
import "Component"
import "Component/MaterialDesign"
//end

Column{
    id: column_control
    property string _tool_tip_text: ""
    property int _widthButton: parent.width
    property int numberButton: loginVM.superUserActive ? 6 : 4

    height: parent.height
    width: parent.width
    spacing: 0
    function callExitPass(){
        exitPassRequire.visible = true
    }
    property bool _enabledCmd: master_app.powerUpState
    property bool _enabledBtn: (loginVM.controlEnabled && _enabledCmd) || loginVM.superUserActive
    property real _opacityBtn: 0.5
    // icon app
    Rectangle{
		id: icon_app_area
        width: column_control._widthButton
        height: width
        color: "#455F61"

        MouseArea{
            id: icon_app_mouse_area
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton
            onDoubleClicked: {
                if(loginVM.superUserActive)
                    master_app.exitDoubleClicked()
                else 
                    exitPassRequire.visible = true
            }
            onPressed: {
                loginVM.updateIdleTimer()
            }
        }

        Image{
            id: closeImage
            width: parent.width*0.8
            height: parent.height
            source: "../Resources/title_bar_left.png"
            fillMode: Image.PreserveAspectFit
            anchors.centerIn: parent
            antialiasing: false
            asynchronous: true
            autoTransform: true
            mipmap: true
        }
		// TooltipWindow{
		// 	_textTooltip: column_control._tool_tip_text
		// 	_visible: icon_app_mouse_area.containsMouse
		// 	x: mainWindow.x + column_control.width
		// 	y: mainWindow.y + column_control.y + icon_app_area.y
		// }
    }
    // start control
    Rectangle{
        id: powerOnBtn
        width: column_control._widthButton
        height: width
        color: "transparent"
        ButtonMaterial{
            id:powerOnTomoBtn
            _width: column_control._widthButton * 0.8
            _height: _width
            _size: _width
            _iconSourceOn: "power-standby"
            _iconSourceOff: ""
            _isClicked: master_app.powerUpState
            anchors.centerIn: parent
            _colorOverlayLow: "red"
            _colorOverlayHigh: "green"
            _enableEffect: false
            opacity: enabled? 1:0.5
            enabled: ( controlBarVM.powerEnabled && loginVM.controlEnabled ) || loginVM.superUserActive
            onClicked: {
                loginVM.updateIdleTimer()
                controlBarVM.powerOnBtnClick(!master_app.powerUpState)
                readyAnimation.running = false
                _width = column_control._widthButton * 0.8
            }
            NumberAnimation on _width {
                id: readyAnimation
                loops: Animation.Infinite
                running: false
                duration: 1500
                from: column_control._widthButton * 0.7; to: column_control._widthButton * 1
            }
            Connections{
                target: master_app
                ignoreUnknownSignals: true
                onReadyToUse:{
                    if(!master_app.powerUpState)
                        readyAnimation.running = true
                }
            }
        }
    }

    // play btn
    Rectangle{
        id: playBtn
        width: column_control._widthButton
        height: width
        visible: loginVM.superUserActive
        color: "transparent"
        ButtonMaterial{
            id:playTomoBtn
            _width: column_control._widthButton * 0.8
            _height: _width
            _size: _width
            _iconSourceOn: "pause"
            _iconSourceOff: "play"
            anchors.centerIn: parent
            _isToggleButton: true
            enabled: _enabledBtn
            opacity: enabled? 1:0.5
            onClicked: {
                loginVM.updateIdleTimer()
                _isClicked = !_isClicked
                controlBarVM.playBtnClick(_isClicked)
            }
            Connections{
                target: controlBarVM
                ignoreUnknownSignals: true
                onSetPlayIconIsClicked:{
                   playTomoBtn._isClicked=value
                }
				onSetEnableButton:{
					if(nameButton == "play") playBtn.enabled = isEnable
				}
            }
        }
    }

    // // Step btn - Next State
    // Rectangle{
    //     id: stateBtn
    //     width: column_control._widthButton
    //     height: width
    //     color: "transparent"
    //     ButtonMaterial{
    //         id: stateTomoBtn
    //         _width: column_control._widthButton * 0.8
    //         _height: _width
    //         _size: _width
    //         _iconSourceOn: "openid"
    //         _iconSourceOff: ""
    //         anchors.centerIn: parent
    //         _enableEffect: true
    //         _durationAnimation:500
    //         _colorOverlayHigh: "aqua"
    //         enabled: _enabledBtn && stateBtn.enabled
    //         opacity: enabled? 1:0.5
    //         onClicked: {
    //             controlBarVM.state_btn_click()
    //         }
	// 		Connections{
    //             target: controlBarVM
    //             ignoreUnknownSignals: true
	// 			onSetEnableButton:{
	// 				if(nameButton == "state")
	// 					stateBtn.enabled = isEnable
	// 			}
	// 		}
    //     }
    // }

    // Step btn - Next Step
    Rectangle{
        id: stepBtn
        width: column_control._widthButton
        height: width
        visible: loginVM.superUserActive
        color: "transparent"
        ButtonMaterial{
            id: stepTomoBtn
            _width: column_control._widthButton * 0.8
            _height: _width
            _size: _width
            _iconSourceOn: "debug-step-over"
            _iconSourceOff: ""
            anchors.centerIn: parent
            _enableEffect: true
            _durationAnimation:500
            _colorOverlayHigh: "aqua"
            enabled: _enabledBtn && stepBtn.enabled
            opacity: enabled? 1:0.5
            onClicked: {
                controlBarVM.step_btn_click()
            }
			// TooltipWindow{
			// 	_textTooltip: qsTr("TomO Step")
			// 	_visible: stepTomoBtn.hovered
			// 	x: mainWindow.x + column_control.width
			// 	y: mainWindow.y + column_control.y + stepBtn.y
			// }

			Connections{
                target: controlBarVM
                ignoreUnknownSignals: true
				onSetEnableButton:{
					if(nameButton == "step")
						stepBtn.enabled = isEnable
				}
			}
        }
    }

    //tab view
    Rectangle{
        width: parent.width
        height: parent.height - column_control._widthButton * numberButton
        color: "#4F5052"
        Item{
            width: parent.width
            height: parent.height
            Loader{
                id: tabControlBar
                anchors.fill: parent
                visible: status == Loader.Ready
                source: "TabControlBar.qml"
            }
        }
    }

    Column{
        width: column_control._widthButton
        height: width * 2
        spacing: 0
        Component.onCompleted: {
            dashboardVM.isCovered = true
            popupLogin.open()
        }
        // login btn
        Rectangle{
            width: parent.width
            height: width
            color: "transparent"
            ButtonMaterial{
                id: login_btn
                _width: column_control._widthButton * 0.8
                _height: _width
                _size: _width
                _iconSourceOn: "account-off"
                anchors.centerIn: parent
                onClicked: {
                    loginVM.updateIdleTimer()
                    // controlBarVM.login_btn_click()
                    dashboardVM.isCovered = true
                    popupLogin.open()
                }
            }
            Connections{
                target: loginVM
                ignoreUnknownSignals: true
                onLevelChanged:{
                    if(isLogin){
                        login_btn._iconSourceOn = "account"
                    }
                    else{
                        login_btn._iconSourceOn = "account-off"
                    }
                }
            }
        }

        // setting btn
        Rectangle{
            id: settingBtn
            width: parent.width
            height: width
            color: "transparent"
            enabled: productionVM.enabledForRunLot || loginVM.superUserActive
            opacity: enabled? 1:0.5
            ButtonMaterial{
                id: setting_btn
                _width: column_control._widthButton * 0.8
                _height: _width
                _size: _width
                _iconSourceOn: "cogs"
                _iconSourceOff: ""
                anchors.centerIn: parent
                onClicked: {
                    loginVM.updateIdleTimer()
                    // controlBarVM.setting_btn_click()
                    var status = tabControlBar.item.onSetting
                    tabControlBar.item.onSetting = !status
                }
            }
        }
    }

    //Popup Bring Up Progress Bar
    Popup {
        id: bringUpProgress
        property int timeOut: 60000
        property bool isBringUp: master_app.powerUpState
        property int widthFullProgress: 775
        property bool isPlayed: false
		property bool breakTimer: false

        modal: true
        closePolicy: Popup.NoAutoClose
        width: 800
        height: 150
        x: mainWindow.width / 2 - width / 2
        y: mainWindow.width / 4 - height / 4
        visible: false
        onIsBringUpChanged: {
            visible = true
            if(bringUpProgress.isBringUp) bringUpBar.width = 0
            else  bringUpBar.width = widthFullProgress
            randomTimer.restart()
            timeOutProgress.restart()
        }
        Text{
            text: {
                if(bringUpProgress.isBringUp)
                    return "System Bring Up In Progress "+ Math.round(bringUpBar.width / (bringUpProgress.width - 25) * 100) + "%"
                else
                    return "System Bring Down In Progress "+ Math.round(bringUpBar.width / (bringUpProgress.width - 25) * 100) + "%"
            }
            width: parent.width
            height: parent.height / 2
            anchors.top: parent.top
            anchors.horizontalCenter: parent.horizontalCenter
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 28
            font.family: "Arial"
            font.bold: true
            color: "white"
        }
        Rectangle {
            id: bringUpBar
            height: parent.height / 2
            radius: 5
            anchors.bottom: parent.bottom
            anchors.left: parent.left
            gradient: Gradient {
                GradientStop { position: 0.1; color: "#74c5e7" }
                GradientStop { position: 0.5; color: "#ffffff" }
                GradientStop { position: 0.9; color: "#080808" }
            }
            Timer{
                id: randomTimer
                interval: 200
                running: false
                repeat: true
                onTriggered: {
                    if(bringUpProgress.isBringUp){
                        parent.width += Math.random()*5
						if(!systemLogVM.sequenceReady) return
						if(bringUpProgress.isPlayed) return
                		dashboardVM.isCovered = false
						bringUpBar.width = bringUpProgress.widthFullProgress
						endProgress.restart()
						timeOutProgress.stop()
						playTomoBtn._isClicked = true
						bringUpProgress.isPlayed = true

					}
                    else{
                        parent.width -= Math.random()*15
						if(systemLogVM.serverReady) return
						if(!bringUpProgress.isPlayed) return
						var timeDelay = 5000
						column_control.delay(timeDelay, function(){
							dashboardVM.isCovered = false
							bringUpBar.width = 0
							endProgress.restart()
							timeOutProgress.stop()
							playTomoBtn._isClicked = false
							bringUpProgress.isPlayed = false
						})
					}
                }
            }
            Timer{
                id: endProgress
                interval: 1000
                running: false
                onTriggered: bringUpProgress.close()
            }
            Timer{
                id: timeOutProgress
                interval: bringUpProgress.timeOut
                running: false
                onTriggered: {
                    bringUpProgress.close()
                    bringUpBar.width = 0
                }
            }
            onWidthChanged: {
                if(bringUpProgress.isBringUp){
                    if(bringUpBar.width >= 750) randomTimer.stop()
                }
                else {
                    if(bringUpBar.width <= 50) randomTimer.stop()
                }
            }
			Connections{
				target: systemLogVM
				ignoreUnknownSignals: true
				onSequenceReadyChanged:{
					controlBarVM.playBtnClick(sequenceReady)
				}
			}
        }
        Connections{
            target: master_app
            ignoreUnknownSignals: true
            onStopProgressBar:{
				if(bringUpProgress.isPlayed) return
                dashboardVM.isCovered = false
				bringUpProgress.isPlayed = true
				randomTimer.stop()
                timeOutProgress.stop()
                endProgress.restart()
			}
        }
    }
    Timer {
        id: timerCB
    }

    function delay(delayTime, cb) {
        timerCB.interval = delayTime;
        timerCB.repeat = false;
        timerCB.triggered.connect(cb);
        timerCB.start();
    }

    //Popup Exit Password Require
    Popup {
        id: exitPassRequire
        x: 460
        y: Screen.desktopAvailableHeight / 2 - height / 2
        width: 1000
        height: columnExitPass.height
		padding: 0
        onXChanged:{
            x = 460
        }
        onVisibleChanged: {
            if(!visible)
                changeExitPassArea.visible = false
        }
        Column{
            id: columnExitPass
            width: parent.width
            Rectangle{
                width: parent.width
                height: 30
                color: "black"
                Text{
                    anchors.fill: parent
                    text: "Exit password is required to close UI"
                    width: parent.width
                    height: parent.height / 2
                    anchors.top: parent.top
                    anchors.horizontalCenter: parent.horizontalCenter
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    font.pixelSize: 24
                    font.family: "Arial"
                    font.bold: true
                    color: "white"
                }
            }
            Rectangle{                
                id: fieldTextItem
                width: parent.width
                property int defaultHeight: 60
                property int expandHeight: 400
                height: defaultHeight
                color: "white"
                border{
                    width: 1
                    color: "black"
                }
                FieldTextWithKeyboard{
                    id: enterSuperPassword
                    _width: parent.width
                    _height: fieldTextItem.defaultHeight
                    anchors.horizontalCenter: parent.horizontalCenter
                    background: null
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    leftPadding: 20
                    font.pointSize: 20
                    color: "black"
                    echoMode: TextField.Password
                    onEnterKeyPress:{
                        loginVM.updateIdleTimer()
                        exitPassRequire.visible = false
                        controlBarVM.checkExitPass(enterSuperPassword.text)
                        enterSuperPassword.text = ""
                    }
                    onFocusChanged:{
                        loginVM.updateIdleTimer()
                        fieldTextItem.height = focus ? fieldTextItem.expandHeight : fieldTextItem.defaultHeight
                    }
                }
            }
            Item{
                width: parent.width
                height: 50
                Row{
                    height: parent.height
                    width: parent.width
                    Item{
                        width: parent.width / 4
                        height: parent.height
                        Text{
                            anchors.fill: parent
                            text: "Permission"
                            width: parent.width
                            height: parent.height / 2
                            anchors.top: parent.top
                            anchors.horizontalCenter: parent.horizontalCenter
                            verticalAlignment: Text.AlignVCenter
                            horizontalAlignment: Text.AlignHCenter
                            font.pixelSize: 18
                            font.family: "Arial"
                            color: "white"
                        }
                    }
                    Item{
                        width: parent.width / 4
                        height: parent.height
                        enabled: loginVM.exitPassEnabled
                        opacity: enabled ? 1:0.5
                        ComboBoxCustom{
                            id: exitCbb
                            anchors.fill: parent
                            property var permissionModel:["Administrator","Technician","Operator"]
                            model: permissionModel 
                            _fontSize: 18
                            property bool isCompleted: false
                            onActivated: {
                                settingVM.updatePermissionData(6, currentIndex);
                                displayText = currentText
                                loginVM.callSetExitPassEnabled()
                            }
                            Connections{
                                target: exitPassRequire
                                ignoreUnknownSignals: true
                                onVisibleChanged:{
                                    if(visible){
                                        exitCbb.model = exitCbb.permissionModel
                                        exitCbb.currentIndex = settingVM.enabledIndex6 - 1
                                        exitCbb.displayText = exitCbb.permissionModel[settingVM.enabledIndex6 - 1]
                                    }
                                }
                            }
                        }
                    }
                    Item{
                        width: parent.width / 4
                        height: parent.height
                        enabled: loginVM.exitPassEnabled
                        opacity: enabled ? 1:0.5
                        Button{
                            anchors.fill: parent
                            // color: "transparent"
                            text: "Change password"
                            contentItem: Text{
                                anchors.fill: parent
                                text: parent.text
                                width: parent.width
                                height: parent.height / 2
                                anchors.top: parent.top
                                anchors.horizontalCenter: parent.horizontalCenter
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                                font.pixelSize: 18
                                font.family: "Arial"
                                color: "white"
                            }
                            onClicked: {
                                changeExitPassArea.visible = !changeExitPassArea.visible
                            }
                        }
                    }
                    Item{
                        width: parent.width / 4
                        height: parent.height
                        enabled: loginVM.exitPassEnabled
                        opacity: enabled ? 1:0.5
                        Button{
                            anchors.fill: parent
                            text: "Reset Password"
                            contentItem: Text{
                                anchors.fill: parent
                                text: parent.text
                                width: parent.width
                                height: parent.height / 2
                                anchors.top: parent.top
                                anchors.horizontalCenter: parent.horizontalCenter
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                                font.pixelSize: 18
                                font.family: "Arial"
                                color: "white"
                            }
                            onClicked: {
                                modalDialogBoxVM.getModalDialogQML(true, "exit_reset")
                            }
                        }
                    }
                }
            }
            Item{
                id: changeExitPassArea
                width: parent.width
                height: 300
                visible: false
                Column{
                    width: parent.width
                    Item{
                        width: parent.width
                        height: 75
                        Row{
                            width: parent.width
                            height: 50
                            anchors.centerIn: parent
                            Text{
                                text: "Current Password"
                                width: parent.width / 2
                                height: parent.height
                                anchors.verticalCenter: parent.verticalCenter
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignRight
                                rightPadding: 20
                                font.pixelSize: 18
                                font.family: "Arial"
                                color: "white"
                            }
                            Rectangle{
                                width: parent.width / 2
                                height: parent.height
                                color: "white"
                                anchors.verticalCenter: parent.verticalCenter
                                FieldTextWithKeyboard{
                                    id: currentExitPassField
                                    anchors.fill: parent
                                    background: null
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    leftPadding: 20
                                    font.pointSize: 20
                                    color: "black"
                                    echoMode: TextField.Password
                                    onEnterKeyPress:{
                                        loginVM.updateIdleTimer()
                                    }
                                }
                            }
                        }
                    }     
                    Item{
                        width: parent.width
                        height: 75
                        Row{
                            width: parent.width
                            height: 50
                            anchors.centerIn: parent
                            Text{
                                text: "New Password"
                                width: parent.width / 2
                                height: parent.height
                                anchors.verticalCenter: parent.verticalCenter
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignRight
                                rightPadding: 20
                                font.pixelSize: 18
                                font.family: "Arial"
                                color: "white"
                            }
                            Rectangle{
                                width: parent.width / 2
                                height: parent.height
                                color: "white"
                                anchors.verticalCenter: parent.verticalCenter
                                FieldTextWithKeyboard{
                                    id: newExitPassField
                                    anchors.fill: parent
                                    background: null
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    leftPadding: 20
                                    font.pointSize: 20
                                    color: "black"
                                    echoMode: TextField.Password
                                    onEnterKeyPress:{
                                        loginVM.updateIdleTimer()
                                    }
                                }
                            }
                        }
                    }    
                    Item{
                        width: parent.width
                        height: 75
                        Row{
                            width: parent.width
                            height: 50
                            anchors.centerIn: parent
                            Text{
                                text: "Confirm New Password"
                                width: parent.width / 2
                                height: parent.height
                                anchors.verticalCenter: parent.verticalCenter
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignRight
                                rightPadding: 20
                                font.pixelSize: 18
                                font.family: "Arial"
                                color: "white"
                            }
                            Rectangle{
                                width: parent.width / 2
                                height: parent.height
                                color: "white"
                                anchors.verticalCenter: parent.verticalCenter
                                FieldTextWithKeyboard{
                                    id: confirmNewExitPassField
                                    anchors.fill: parent
                                    background: null
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    leftPadding: 20
                                    font.pointSize: 20
                                    color: "black"
                                    echoMode: TextField.Password
                                    onEnterKeyPress:{
                                        loginVM.updateIdleTimer()
                                    }
                                }
                            }
                        }
                    }   
                    Item{
                        width: parent.width
                        height: 75
                        Button{
                            width: 150
                            height: 50
                            anchors.centerIn: parent
                            text: "Change"
                            contentItem: Text{
                                anchors.fill: parent
                                text: parent.text
                                width: parent.width
                                height: parent.height / 2
                                anchors.verticalCenter: parent.verticalCenter
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                                font.pixelSize: 18
                                font.family: "Arial"
                                color: "white"
                            }
                            onClicked: {
                                if(currentExitPassField.text === ""){
                                    modalDialogBoxVM.getModalDialogQML(false, "miss_current")
                                    return
                                }
                                if(newExitPassField.text === ""){
                                    modalDialogBoxVM.getModalDialogQML(false, "miss_new")
                                    return
                                }
                                if(confirmNewExitPassField.text === ""){
                                    modalDialogBoxVM.getModalDialogQML(false, "miss_confirm")
                                    return
                                }
                                var result = controlBarVM.checkChangePass(currentExitPassField.text, newExitPassField.text, confirmNewExitPassField.text)
                                switch(result) {
                                    case 0:{
                                        newExitPassField.text = ""
                                        currentExitPassField.text = ""
                                        confirmNewExitPassField.text = ""
                                        changeExitPassArea.visible = false
                                        break
                                    }
                                    case 1:{
                                        currentExitPassField.text = ""
                                        break
                                    }
                                    case 2:{
                                        newExitPassField.text = ""
                                        break
                                    }
                                    case 3:{
                                        confirmNewExitPassField.text = ""
                                        break
                                    }
                                    case 4:{
                                        newExitPassField.text = ""
                                        confirmNewExitPassField.text = ""
                                        break
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
