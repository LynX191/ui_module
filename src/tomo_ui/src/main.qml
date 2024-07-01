import QtQml 2.2
import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.2
import QtGraphicalEffects 1.0
import QtQuick.Layouts 1.3

//My import
import "Dialog"
import "View"
import "Component/MaterialDesign"
import "Component"
//end

ApplicationWindow {
    id: mainWindow
    flags: getFlags()
    visible: false
    title: qsTr("TomO Cartoner v1.0.1")
   	opacity: 1.0
    width: 1920
    height: 1080
    minimumHeight: 1080
    minimumWidth: 1920
    x: 0
    y: -30
    signal appIsActive()
    signal appIsInactive()
    onVisibleChanged: {
        if(visible && popupLogin.isFirstTimeOpen){
            delayKeyboard.start()
            popupLogin.isFirstTimeOpen = false
        }
        if(visible) mainWindow.showFullScreen()
    }
    function getFlags(){
        if(loginVM.superUserActive)
            return Qt.FramelessWindowHint
        else{
            if(!master_app.powerUpState){
                return Qt.FramelessWindowHint
            }
            else{
                return Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint
            }
        }
    }
    // onYChanged:{
    //     y = 0
    //     mainWindow.showFullScreen()
    // }
	// onXChanged:{
    //     x = 0
    //     mainWindow.showFullScreen()
	// }
    // onActiveChanged:{
    //     // if(!dashboardVM.isCovered)
    //     // tomoVM.updateRvizVisible(active)
    // }

    // Main Layout
    /*---------------------*/

    Row{
		id: row_main_window
        width: parent.width
        height:parent.height
        spacing:6

        // ControlBar
        Rectangle{
            id: control_bar_layout
            enabled: !isInitializing
		    opacity: enabled? 1:0.5
            width: 0.032*parent.width - parent.spacing/3
            height:parent.height
            color: "#201E1F"
            ControlBar{
                id: controlBar
                _tool_tip_text: mainWindow.title
            }
        }

        // Main view
        Column{
            id: mainViewItem
            width: 0.936*parent.width - parent.spacing/3
            height: parent.height
            padding: 0
            spacing: 0
            NotifyBar{
                id: notifyBarArea
                width: parent.width
                height: 60
                visible: true
                property string newNotiColor: "gray"
            }

            Item{
                id: main_view_layout
                enabled: true
                width: parent.width
                height: notifyBarArea.visible? parent.height - notifyBarArea.height : parent.height
            }
        }

        //System status
        Rectangle{
            id: system_status_layout
            enabled: true
            width: 0.032*parent.width - parent.spacing/3
            height:parent.height
            color: "#1e1e1e"
            SystemBar{
                _scale_width: 0.9
                anchors.horizontalCenter: parent.horizontalCenter
            }
        }
    }
    // Global function in main
    QtObject{
        id: internalMain

        function closeApp() {
            mainWindow.close()
        }
    }
    Connections{
        target: master_app
        ignoreUnknownSignals: true
        onConfirmExit: {
            mainVM.closeAppFormUI()
            mainWindow.closeConfirmed = true
            mainWindow.close()
//            Qt.quit();
        }
    }
    Timer{
        id: timeOutExit
        interval: 1000
        onTriggered: mainWindow.close()
    }

    // free component
    Loader{
        id:loadingScreen
        source: "View/Splash/SplashScreen.qml"
        visible: status == Loader.Ready
    }

    // for all dialog
    NotifyView{
        id:notifyView
        width: 900
        height: parent.height
        x:mainViewItem.x+mainViewItem.width-width
    }

    Popup {
        id: popupLogin
        width: 960
        height: 640
        x: parent.width  / 2 - width  / 2
        y: parent.height / 2 - height / 2
        modal: true
        margins: 0
        onClosed: dashboardVM.isCovered = false
        property bool isFirstTimeOpen: true
        Login{
            id: loginView
            anchors.fill: parent
            _width: popupLogin.width
            _height: popupLogin.height
			onQuitSignal: popupLogin.close()
        }
        onVisibleChanged: {

        }
        Timer{
            id: delayKeyboard
            interval: 400
            onTriggered: {
                loginView.viewKeyboardOnInit()
            }
        }
    }
	LoadingPopup{
		id:loadingPopup
	}
    // connection area
    /*-------------------*/

    Connections{
        target: modalDialogBoxVM
        ignoreUnknownSignals: true
        onAddToNotifyBar:{
            var time = new Date()
            notifyBarArea.addNotify(type, messageID, content,time)
        }
        onPopupModalDialog:{
            var splitResult = content.split(" - ")
            modalDialogPopup._typeDialog = type
            modalDialogPopup._content = splitResult[0],
            modalDialogPopup._detailContent = splitResult[1],
            modalDialogPopup._messageID = messageID,
            modalDialogPopup._isFunctional = isFunctional
            modalDialogPopup.visible = true
        }
		onPopupLoading:{
			loadingPopup._text = text
            loadingPopup.open()
		}
		onClosePopupLoading:{
			loadingPopup.close()
		}
    }

    ModalDialogBox{
        id: modalDialogPopup
        z: 999
    }

    Connections{
        target: loadingScreen.item
        ignoreUnknownSignals: true
        onTimeOut:{
            mainWindow.requestActivate()
            mainWindow.visibility =  Window.AutomaticVisibility
            mainWindow.visible = true
			mainVM.uiCompleted()

            controlBarVM.setAfterUiCreated()
            headVM.setAfterUiCreated()
            shipperVM.setAfterUiCreated()
            pimVM.setAfterUiCreated()
            settingVM.setAfterUiCreated()
            tomoVM.setAfterUiCreated()
            dashboardVM.setAfterUiCreated()
            ioControlVM.setAfterUiCreated()
        }
    }

    Connections{
        target: ioControlVM
        Component.onCompleted: ioControlVM.getYamlIOput()
    }

    Connections{
        target: mainVM
        ignoreUnknownSignals: true
		onSendEnableAllsLayout:{
            control_bar_layout.enabled = isEnable
            main_view_layout.enabled = isEnable
            system_status_layout.enabled = isEnable
		}
    }

    // signals area
    /*-------------------*/
    property bool closeConfirmed: false
    onClosing: {
        if(!closeConfirmed){
            close.accepted = false
            if(loginVM.superUserActive)
                master_app.exitDoubleClicked()
            else 
                controlBar.callExitPass()
        }else{
            close.accepted = true
        }
    }
    property bool isInitializing: false
    Connections{
        target: productionVM
        ignoreUnknownSignals: true
        onInitializing:{
            isInitializing = isRunning
        }
    }
}

