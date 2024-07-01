import QtQml 2.0
import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2

// My import
import "../../Component"
import "../../Component/MaterialDesign"

//end

Rectangle{
    id:frame


    signal sendCurrentState(int index, string state)
    property string color_for_span_tag: "#000000"
    // property var _plain_text:[]
    property int _width_layout_status: 80
    property int _height_layout_status: 32
    property string _bgColor: "#000000"
    property string _textFont: "Arial"
    property int _textSize: 16
    property int _lineSpacing: 0
    property int _limit_line: 10000
    property int _scroll_line: 5
    property int _index: -1
	property bool scroll_bar_at_end: true
	property bool _enabled: true
    property var positon_move: 0.01
	property int _max_character_one_line: 0
	property bool checkFirstTime: true

	enabled: _enabled
	opacity: _enabled ? 1.0 : 0.5
    color: "transparent"
	focus: true

    // function appendPlainText(text){
    //     _plain_text.push(text)
    // }
    // function removePlainText(index){
    //     _plain_text.remove(0, Math.round(index))
    // }
    function setConfigCmd(index, bgColor, textFont, textSize, lineSpacing, limitLine,scrollLine)
    {
        frame._index = index
        frame._bgColor = bgColor
        frame._textFont = textFont
        frame._textSize = textSize
        frame._lineSpacing = lineSpacing
        frame._limit_line = limitLine
        frame._scroll_line = scrollLine
    }

    function checkNumOfLine()
    {
        if(textModelALL.count >= frame._limit_line)
        {
            var index =
            frame.removeLine(parseInt(frame._limit_line)*0.1)
            // frame.removePlainText(index)
        }
    }

	function getScrollBarState()
	{
        if(!scrollingBtn._isClicked){
		    frame.scroll_bar_at_end = (list_cmd_scrollbar.size < 0) || ((list_cmd_scrollbar.size + list_cmd_scrollbar.position) === 1 || checkFirstTime)
        }
	}

	function scrollToEnd()
	{
		if(frame.scroll_bar_at_end) {
			list_cmd.positionViewAtEnd()
			list_cmd.currentIndex = list_cmd.count - 1
		}
	}

    function appendLine(logText)
    {
        textModelALL.append({text: logText})
        checkNumOfLine()
		scrollToEnd()
	}

    function separateSignal(logText){
        if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            warnModelALL.append(({text: logText + "</p></pre>"}))
        }
        if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1 ) {
            errorModelALL.append(({text: logText + "</p></pre>"}))
        }
        else{
            infoModelALL.append(({text: logText + "</p></pre>"}))
        }
        if (logText.indexOf("[WM]") !== -1 ) {
            textModelWM.append(({text: logText + "</p></pre>"}))
            // if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            //     warnModelWM.append(({text: logText + "</p></pre>"}))
            // }
            // else if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1) {
            //     errorModelWM.append(({text: logText + "</p></pre>"}))
            // }
            // else{
            //     infoModelWM.append(({text: logText + "</p></pre>"}))
            // }
        }
        else if (logText.indexOf("[DM]") !== -1 ) {
            textModelDM.append(({text: logText + "</p></pre>"}))
            // if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            //     warnModelDM.append(({text: logText + "</p></pre>"}))
            // }
            // else if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1) {
            //     errorModelDM.append(({text: logText + "</p></pre>"}))
            // }
            // else{
            //     infoModelDM.append(({text: logText + "</p></pre>"}))
            // }
        }
        else if (logText.indexOf("[FM]") !== -1 ) {
            textModelFM.append(({text: logText + "</p></pre>"}))
            // if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            //     warnModelFM.append(({text: logText + "</p></pre>"}))
            // }
            // else if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1) {
            //     errorModelFM.append(({text: logText + "</p></pre>"}))
            // }
            // else{
            //     infoModelFM.append(({text: logText + "</p></pre>"}))
            // }
        }
        else if (logText.indexOf("[CM]") !== -1 ) {
            textModelCM.append(({text: logText + "</p></pre>"}))
            // if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            //     warnModelCM.append(({text: logText + "</p></pre>"}))
            // }
            // else if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1) {
            //     errorModelCM.append(({text: logText + "</p></pre>"}))
            // }
            // else{
            //     infoModelCM.append(({text: logText + "</p></pre>"}))
            // }
        }
        else if (logText.indexOf("[IM]") !== -1 ) {
            textModelIM.append(({text: logText + "</p></pre>"}))
            // if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            //     warnModelIM.append(({text: logText + "</p></pre>"}))
            // }
            // else if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1) {
            //     errorModelIM.append(({text: logText + "</p></pre>"}))
            // }
            // else{
            //     infoModelIM.append(({text: logText + "</p></pre>"}))
            // }
        }
        else if (logText.indexOf("[OM]") !== -1 ) {
            textModelOM.append(({text: logText + "</p></pre>"}))
            // if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            //     warnModelOM.append(({text: logText + "</p></pre>"}))
            // }
            // else if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1) {
            //     errorModelOM.append(({text: logText + "</p></pre>"}))
            // }
            // else{
            //     infoModelOM.append(({text: logText + "</p></pre>"}))
            // }
        }
        else if (logText.indexOf("[TM]") !== -1 ) {
            textModelTM.append(({text: logText + "</p></pre>"}))
            // if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            //     warnModelTM.append(({text: logText + "</p></pre>"}))
            // }
            // else if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1) {
            //     errorModelTM.append(({text: logText + "</p></pre>"}))
            // }
            // else{
            //     infoModelTM.append(({text: logText + "</p></pre>"}))
            // }
        }
        else if (logText.indexOf("[GM]") !== -1 ) {
            textModelGM.append(({text: logText + "</p></pre>"}))
            // if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            //     warnModelGM.append(({text: logText + "</p></pre>"}))
            // }
            // else if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1) {
            //     errorModelGM.append(({text: logText + "</p></pre>"}))
            // }
            // else{
            //     infoModelGM.append(({text: logText + "</p></pre>"}))
            // }
        }
        else if (logText.indexOf("[LM]") !== -1 ) {
            textModelLM.append(({text: logText + "</p></pre>"}))
            // if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            //     warnModelLM.append(({text: logText + "</p></pre>"}))
            // }
            // else if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1) {
            //     errorModelLM.append(({text: logText + "</p></pre>"}))
            // }
            // else{
            //     infoModelLM.append(({text: logText + "</p></pre>"}))
            // }
        }
        else if (logText.indexOf("[SM]") !== -1 ) {
            textModelSM.append(({text: logText + "</p></pre>"}))
            // if (logText.indexOf("#FFC706>") !== -1 && logText.indexOf("#FFC706><") === -1) {
            //     warnModelSM.append(({text: logText + "</p></pre>"}))
            // }
            // else if (logText.indexOf("#EF190F>") !== -1 && logText.indexOf("#EF190F><") === -1) {
            //     errorModelSM.append(({text: logText + "</p></pre>"}))
            // }
            // else{
            //     infoModelSM.append(({text: logText + "</p></pre>"}))
            // }
        }
    }

    function changeTypeLogText(moduleName,typeSignal)
    {
        if(typeSignal === "INFO")
            list_cmd.model = eval("infoModel" + moduleName)
        else if(typeSignal === "WARN")
            list_cmd.model = eval("warnModel" + moduleName)
        else if(typeSignal === "ERROR")
            list_cmd.model = eval("errorModel" + moduleName)
        else if(typeSignal === "ALL")
            list_cmd.model = eval("textModel" + moduleName)
		scrollToEnd()
    }

    function removeLine(index){
        textModelALL.remove(0, index)
    }

    function removeAll(){
        textModelALL.clear()
    }

    function setStateButton(text){
        if(text === "Run"){
           status_cmd._text = "Run"
           status_cmd._color = "green"
           icon_cmd.color = "green"
        }
        else{
            status_cmd._text = "Stop"
            status_cmd._color = "red"
            icon_cmd.color = "red"
        }
    }

    onHeightChanged: {
		scrollToEnd()
    }

	TextMetrics {
		id: textMetrics
		font.pointSize: frame._textSize
		font.family:frame._textFont
		text: "A"
	}

    ListModel {
        id: textModelALL
    }

    ListView {
        id:list_cmd
        property int height_one_line: Math.round(frame._textSize * 1.75)
        property int max_line_in_view: Math.round(frame.height / list_cmd.height_one_line)
        property bool lock_view_area: false

        Component {
            id: highlight
            Rectangle {
                color: "#272727";
            }
        }
		Component.onCompleted: positionViewAtEnd()
        highlight: highlight
        highlightFollowsCurrentItem: true
        highlightMoveDuration: -1
        highlightMoveVelocity: -1
        cacheBuffer: 30
        spacing:frame._lineSpacing
        clip: true
        focus: true
        model: textModelALL
        width: parent.width
        height: frame.height - anchors.topMargin //+ frame._lineSpacing// list_cmd.height_one_line*list_cmd.max_line_in_view - anchors.topMargin  //
        anchors.top: parent.top
        anchors.topMargin: 2
        ScrollBar.vertical: ScrollBar {
            id: list_cmd_scrollbar
            width: 15
            height: list_cmd.height
            anchors.right: list_cmd.right
            anchors.top: list_cmd.top
            interactive: true
            active: true
            visible: true
            policy: ScrollBar.AlwaysOn
            contentItem: Rectangle {
                height: list_cmd.height / 10
                width: 15
                radius: width / 2
                opacity: list_cmd_scrollbar.pressed ? 1 : 0.5
                color: "#81e889"
                anchors.horizontalCenter: list_cmd_scrollbar.horizontalCenter
            }
            background: Rectangle {
                anchors.right: list_cmd_scrollbar.right
                anchors.top: list_cmd_scrollbar.top
                width: 15
                height: list_cmd.height
                color: "grey"
            }
        }

        delegate: TextArea{
            id:txtarea
            readOnly: true
            text: model.text
            persistentSelection: true
            width: list_cmd.width - list_cmd_scrollbar.width
            height: contentHeight
            anchors.left: parent.left
            anchors.leftMargin: 20
            topPadding: 0
            bottomPadding: 0
            selectByMouse: true
            selectByKeyboard: true
            selectionColor: "#708090"
            selectedTextColor: "#f0f8ff"
            font.pointSize: _textSize
            font.family: _textFont
            textFormat: TextEdit.RichText
            wrapMode: TextEdit.Wrap
			background: Rectangle{
				implicitWidth: parent.width
				implicitHeight: parent.height
				color: "transparent"
			}
            MouseArea {
                anchors.fill: parent
                onClicked: {
                    loginVM.updateIdleTimer()
                    list_cmd.currentIndex = model.index
                }
            }
        }
    }

    MouseArea{
        id: mouseArea
        x: list_cmd.x
        y: list_cmd.y
        property int currentPosition
        width: list_cmd.width - list_cmd_scrollbar.width
        height: list_cmd.height
        drag.target: list_cmd
        drag.axis: Drag.YAxis
        hoverEnabled: true
        // propagateComposedEvents: true
        onEntered: cursorShape = Qt.IBeamCursor
        onPositionChanged: {
            if(drag.active){
                var positon_move = (mouseY - currentPosition) / list_cmd.contentHeight
                list_cmd_scrollbar.position -= positon_move

                if(list_cmd_scrollbar.position < 0) list_cmd_scrollbar.position = 0.0
                else if(list_cmd_scrollbar.position > 1) list_cmd_scrollbar.position = 1.0
                
                if(list_cmd_scrollbar.position + list_cmd_scrollbar.size > 1){
                    list_cmd_scrollbar.position = 1 -  list_cmd_scrollbar.size
                    list_cmd.lock_view_area = false
                }
                else {
                    list_cmd.lock_view_area = true
                }
            }
            currentPosition = mouseY
        }
        onClicked: {
            loginVM.updateIdleTimer()
            mouse.accepted = false
        }
        onWheel: {
            var positon_move = (list_cmd.height_one_line * _scroll_line) / list_cmd.contentHeight
            if(wheel.angleDelta.y > 0) list_cmd_scrollbar.position -= positon_move
            else list_cmd_scrollbar.position += positon_move

            if(list_cmd_scrollbar.position < 0) list_cmd_scrollbar.position = 0.0
            else if(list_cmd_scrollbar.position > 1) list_cmd_scrollbar.position = 1.0

            if(list_cmd_scrollbar.position + list_cmd_scrollbar.size > 1){
                list_cmd_scrollbar.position = 1 -  list_cmd_scrollbar.size
                list_cmd.lock_view_area = false
            }
            else {
                list_cmd.lock_view_area = true
            }
        }
    }

    TextEdit{
        id: textEdit
        visible: false
    }

    RowLayout{
        anchors.top: parent.top
        anchors.topMargin: 5
        anchors.right: parent.right
        anchors.rightMargin: 15
        width: _width_layout_status
        height: _height_layout_status
        spacing: 6
        visible: loginVM.superUserActive

        Rectangle {
            color: 'transparent'
            Layout.preferredWidth:  _height_layout_status
            Layout.preferredHeight: _height_layout_status
            width: _height_layout_status
            height: width
            MaterialDesignIcon{
                id: icon_cmd
                name:"console"
                size: _height_layout_status
                anchors.centerIn: parent
            }
            MouseArea{
                anchors.fill: parent
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    loginVM.updateIdleTimer()
                    if(status_cmd._text === "") return
					if(status_cmd._text === "---") return
                    frame.sendCurrentState(frame._index, status_cmd._text)
                }
            }
        }
        Rectangle {
            color: "transparent"
            Layout.preferredWidth: parent.width - _height_layout_status - parent.spacing
            Layout.preferredHeight: parent.height
            width: parent.width - _height_layout_status - parent.spacing
            height: parent.height
            Text {
                id: status_cmd
                property string _text: "---"
                property color _color: "white"
                anchors.left: parent.left
                anchors.verticalCenter: parent.verticalCenter
                text: _text
                color: _color
                font.bold: true
                font.pixelSize: 16
            }
        }
    }

    ButtonMaterial{
        id: scrollingBtn
        _width: 48
        _height: _width
        _size: _width
        _iconSourceOn: "mouse-off"
        _iconSourceOff: "mouse"
        anchors.right: parent.right
        anchors.rightMargin: 25
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 10
        _enableEffect: false
        _isToggleButton: true
        enabled: true
        opacity: enabled? 1:0.5
        onClicked: {
            loginVM.updateIdleTimer()
            frame.scroll_bar_at_end = _isClicked
            _isClicked = !_isClicked
        }
    }


    ListModel {
        id: infoModelALL
    }

    ListModel {
        id: warnModelALL
    }

    ListModel {
        id: errorModelALL
    }

    ListModel {
        id: textModelWM
    }

    ListModel {
        id: infoModelWM
    }

    ListModel {
        id: warnModelWM
    }

    ListModel {
        id: errorModelWM
    }

    ListModel {
        id: textModelDM
    }

    ListModel {
        id: infoModelDM
    }

    ListModel {
        id: warnModelDM
    }

    ListModel {
        id: errorModelDM
    }

    ListModel {
        id: textModelFM
    }

    ListModel {
        id: infoModelFM
    }

    ListModel {
        id: warnModelFM
    }

    ListModel {
        id: errorModelFM
    }

    ListModel {
        id: textModelCM
    }

    ListModel {
        id: infoModelCM
    }

    ListModel {
        id: warnModelCM
    }

    ListModel {
        id: errorModelCM
    }

    ListModel {
        id: textModelIM
    }

    ListModel {
        id: infoModelIM
    }

    ListModel {
        id: warnModelIM
    }

    ListModel {
        id: errorModelIM
    }

    ListModel {
        id: textModelOM
    }

    ListModel {
        id: infoModelOM
    }

    ListModel {
        id: warnModelOM
    }

    ListModel {
        id: errorModelOM
    }

    ListModel {
        id: textModelTM
    }

    ListModel {
        id: infoModelTM
    }

    ListModel {
        id: warnModelTM
    }

    ListModel {
        id: errorModelTM
    }

    ListModel {
        id: textModelGM
    }

    ListModel {
        id: infoModelGM
    }

    ListModel {
        id: warnModelGM
    }

    ListModel {
        id: errorModelGM
    }

    ListModel {
        id: textModelLM
    }

    ListModel {
        id: infoModelLM
    }

    ListModel {
        id: warnModelLM
    }

    ListModel {
        id: errorModelLM
    }

    ListModel {
        id: textModelSM
    }

    ListModel {
        id: infoModelSM
    }

    ListModel {
        id: warnModelSM
    }

    ListModel {
        id: errorModelSM
    }
}
