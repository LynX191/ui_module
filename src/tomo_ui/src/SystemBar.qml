import QtQml 2.2
import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

//My import
import "Component"
import "Component/MaterialDesign"
import "View/SystemLogView"

//end

Column{
    property double _scale_width: 1.0
    property int _fontPointSize: 16
    property string _fontFamily:  "Arial"
    property color _borderColorBtn: "#a9a9a9"
    property color _bgColorBtn: "#4A4A4A"
    property var dict_button_control_overlay:{ "OVERLAY_BTN": 0, "DEFECT_BTN": 1,
                                               "OVERKILL_BTN": 2, "TRACKING_BTN":3
    }

    width: parent.width
    height: parent.height
    anchors.centerIn: parent
    spacing: 5

    //StateMachine
    Item{
        width: parent.width * 0.9
        height: parent.height * 0.457 - 80
		visible:loginVM.superUserActive
        Rectangle{
            anchors.fill: parent
            radius: 5
			color: "transparent"
            Text{
                anchors.fill: parent
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                color: "yellow"
                text: "Process: " + systemBarVM.processState + "  -  " + "Lot: " + systemBarVM.lotState
                font.pointSize:_fontPointSize
                font.weight: Font.DemiBold
                font.family: _fontFamily
                rotation: 90
            }
        }
    }
    // system log
    Rectangle{
        id: system_log_area
        property var wnd: undefined
        property string windowSource: "qrc:/View/SystemLogView/SystemLogView.qml"
        property int marginW: 6
        property int marginH: 1
        property int orginal_main_view_layout_height: 0

        height: parent.height*0.2
        width: parent.width*0.9
        color: "#505557"
        radius: 5
        anchors.left: parent.left
	    enabled: loginVM.systemLogEnabled || loginVM.superUserActive
        opacity: enabled ? 0.75 : 0.5
        Text {
            anchors.centerIn: parent
            text: qsTr("System Log")
            color: parent.wnd === undefined ? "#B2C8DF" : "yellow"
            font.pointSize:_fontPointSize
            font.weight: Font.DemiBold
            font.family: _fontFamily
            rotation: 90
        }
        MouseArea{
            anchors.fill: parent
            hoverEnabled: true
            onEntered: parent.opacity = 1
            onExited: parent.opacity = 0.75
            onPressed: parent.opacity = 0.75
            onReleased: parent.opacity = 1
            property var camsListID: []
            onClicked: {
                loginVM.updateIdleTimer()
                if(system_log_area.wnd === undefined) return;
                systemLogVM.systemLogClicked(system_log_area.wnd.visible)
                //                if(!system_log_area.wnd.visible){
                //                    system_log_area.wnd.show();
                //                    console.log("-----------------------show window system log success")
                //                }
                //                else{
                //                     system_log_area.wnd.hide();
                //                     console.log("-----------------------hidden window system log success")
                //                }
            }
        }

        Timer {
            id: timer
        }

        function delay(delayTime, cb) {
            timer.interval = delayTime;
            timer.repeat = false;
            timer.triggered.connect(cb);
            timer.start();
        }
        Connections{
            target:mainVM
            ignoreUnknownSignals: true
            onSignalUICompleted:{
                system_log_area.delay(100, function() {
                    if(system_log_area.wnd === undefined){
                        var component = Qt.createComponent(system_log_area.windowSource)
                        if( component.status !== Component.Ready ){
                            if( component.status === Component.Error ) {
                                console.debug("Error created window:"+ component.errorString());
                                return;
                            }
                            else return;
                        }
                            // keep org height of main view layout
                            system_log_area.orginal_main_view_layout_height = main_view_layout.height

                            var x_init = control_bar_layout.width
                            var width_init = main_view_layout.width + 12

                            system_log_area.wnd = component.createObject(mainWindow,{
                                                x: x_init,
                                                width: width_init,
                                                _enabled: main_view_layout.enabled,
                                                main_view_layout_object: mainViewItem,
                                            });

                            system_log_area.wnd.hide();
                            systemLogVM.uiCreatedComplete();
                            system_log_area.wnd.closing.connect(function() {
                            system_log_area.wnd = undefined;
                        });
                    }
                })

            }
        }
    }

	Item{
        width: parent.width * 0.9
        height: loginVM.superUserActive ? parent.height * 0 : parent.height * 0.382
	}
	Item{
        width: parent.width * 0.9
        height: parent.height * 0.075
		ButtonMaterial{
			id: muteButton
			anchors.fill: parent
            _size: width
            _iconSourceOn: "volume-high"
            _iconSourceOff: "volume-off"
            _isClicked: productionVM.soundState
            _enableEffect: false
			enabled: loginVM.productionEnabled || loginVM.superUserActive
            property int counter: 0
            onClicked: {
                loginVM.updateIdleTimer()
                productionVM.muteBtnClicked(!_isClicked)
                switch(counter){
                    case 0:
	                    productionVM.endLotTimerAction(0)
		            break
                    case 1:
	                    productionVM.endLotTimerAction(1)
		            break
                    case 2:
	                    productionVM.endLotTimerAction(2)
		            break
                    case 3:
	                    productionVM.endLotTimerAction(3)
		            break
                }
                counter++
                if(counter > 3)  counter = 0
            }
		}
	}
    Rectangle{
        id: guardDoorArea
        height: parent.height*0.25
        width: parent.width*0.9
        property bool isClicked
        color: isClicked ? "green" : "red"
        radius: 5
        anchors.left: parent.left
        enabled: systemLogVM.serverReady && productionVM.enabledForOpenDoor
        opacity: enabled ? 1 : 0.5
        Text {
            anchors.centerIn: parent
            text: parent.isClicked ? "Guard Door Locked" : "Guard Door Unlocked"
            color: "black"
            font.pointSize:_fontPointSize
            font.weight: Font.DemiBold
            font.family: _fontFamily
            rotation: 90
        }
        MouseArea{
            anchors.fill: parent
            onClicked: {
                loginVM.updateIdleTimer()
                ioControlVM.sendOutput(1, 5 , !parent.isClicked)
            }
        }
        Connections{
            target: productionVM
            ignoreUnknownSignals: true
            onEmergencyEvent:{
                if(nameEvent === "guard"){
                    // console.log(nameEvent, state)
                    guardDoorArea.isClicked = !state
                }
            }
        }
    }
    //clock button
    Button{
        id:clockBtn
        width: parent.width
        height: 80
        background: Rectangle {
            color: "transparent"
        }
        onClicked: {
            loginVM.updateIdleTimer()
            if(loginVM.accepted && !loginVM.superUserActive)
                superUserPassword.visible = true
        }
        onPressAndHold: {
            if(loginVM.superUserActive)
                loginVM.checkSuperUser("Log out")
        }
        Item{
            width: parent.width-5
            height: parent.height/2
            anchors.top: parent.top
            anchors.topMargin: 5
            Text {
                id: timeClockText
                color: "white"
                font.pointSize: 11
                font.bold: true
                anchors.centerIn: parent
                text: Qt.formatTime(new Date(),"hh:mm")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
        Item{
            width: parent.width-5
            height: parent.height/2
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 5
            Text {
                id: dateText
                text: Qt.formatDateTime(new Date(), "dd/MM/yy")
                color: "white"
                anchors.centerIn: parent
                font.pointSize: 8
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
        Timer {
            interval: 1000
            repeat: true
            running: true
            onTriggered:
            {
                var _time = new Date()
                timeClockText.text =  Qt.formatTime(_time,"hh:mm")
                dateText.text =  Qt.formatDateTime(_time, "dd/MM/yy")
            }
        }
    }
    Popup {
        id: superUserPassword
        x: - 1410
        y: Screen.desktopAvailableHeight / 2 - height / 2
        width: 1000
        height: 60
		padding: 0
		// modal: true
		// closePolicy: Popup.NoAutoClose
        // modality: Qt.ApplicationModal
        // flags: Qt.FramelessWindowHint | Qt.BypassWindowManagerHint
        onXChanged:{
            x = - 1410
        }
		Rectangle{
			anchors.fill: parent
			color: "#cfd1cf"
			FieldTextWithKeyboard{
				id: enterSuperPassword
				_width: parent.width
				_height: 60
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
					loginVM.checkSuperUser(enterSuperPassword.text)
					superUserPassword.visible = false
					enterSuperPassword.text = ""
				}
				onFocusChanged:{
					loginVM.updateIdleTimer()
					superUserPassword.height = focus ? 400 : 60
				}
			}
		}
    }
}
