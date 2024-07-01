import QtQml 2.2
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Window 2.3
import QtQuick.VirtualKeyboard 2.2
import QtQuick.VirtualKeyboard.Settings 2.2

//my import
import "../Component"

TextField {
    id:inputField

	signal enterKeyPress()
    property bool onlyNumber: false
    property  int _width: 350
    property int _height: 200
    property int _minimumValue: 0
    property int _maximumValue: 100000
    property int _yTextBoard: 50
    text: ""
    readOnly: false
    selectByMouse: false
    width: _width
    height: _height
    bottomPadding: height / 5
    validator: RegExpValidator { regExp: onlyNumber ? /^[0-9]*$/ : /^[a-zA-Z0-9]*$/ } 
    function closeKeyboard(){
        numberKeyboard.close()
        keyboard.visible = false
    }
    inputMethodHints: Qt.ImhNoPredictiveText | Qt.ImhNoAutoUppercase
    Keyboard {
        id:numberKeyboard
        x: (parent.width - width) / 2
        y: 50
        z: 100
        width: 400
        height: 230
        visible:false
        onVisibleChanged:{
            loginVM.updateIdleTimer()
            if(!visible) inputField.focus = visible
        }
        onEnterKeyNumPress:{
            inputField.enterKeyPress()
        }
    }
    Popup {
		id:keyboard
        x: (parent.width - width) / 2
        y: _yTextBoard
        z: 100
        width: 1000
        height: 340
        visible:false
        padding: 0
        onVisibleChanged:{
            loginVM.updateIdleTimer()
            if(!visible) inputField.focus = false
        }
		InputPanel {
            visible: parent.visible
            anchors.fill: parent
        }
    }

    onFocusChanged:{
        loginVM.updateIdleTimer()
        if(!focus) {
            if(onlyNumber) numberKeyboard.close()
            else keyboard.visible = false
        }
        else {
            if(onlyNumber) numberKeyboard.open()
            else keyboard.visible = true
        }
    }
    onTextChanged: {
        loginVM.updateIdleTimer()
		if(onlyNumber){
			if(parseInt(text) >= _maximumValue) text = _maximumValue
			else if (parseInt(text) <= _minimumValue) text = _minimumValue
		}
    }
	Keys.onPressed:{
        loginVM.updateIdleTimer()
		if((event.key == Qt.Key_Enter) || (event.key == Qt.Key_Return)) {
			inputField.enterKeyPress()
			if(!onlyNumber) keyboard.visible = false
		}
	}
}
