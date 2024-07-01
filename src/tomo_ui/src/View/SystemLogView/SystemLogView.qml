import QtQml 2.0
import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2

// My import
import "../../Component"
//end

Item {
    id: rootItem
    visible: false
	enabled: loginVM.systemLogEnabled || loginVM.superUserActive
	opacity: _enabled ? 1.0 : 0.5
    clip: true
	signal closing()

	property bool _enabled : true
    property int tabHeight: 34
    property int mouseAreaHeight: 6
    property int marginTitle_Tab: 0
    property int lastest_height_before_up: 0
    property int lastest_y_before_up: 0
    property int lastest_main_view_layout_height: 0
    property int lastest_main_view_layout_y: 0
    property int orginal_main_view_layout_height: 0
    property bool initSystemlog: false
    property bool isMaxSystemLog: false
    property QtObject main_view_layout_object: QtObject{}
    property string moduleName
    property string typeSignal
    property int serverTerIndex: -1

	function hide(){visible = false}
	function show(){visible = true}
    function returnOrgHeightMainView(){
        rootItem.lastest_main_view_layout_height = main_view_layout_object.height
        rootItem.lastest_main_view_layout_y = main_view_layout_object.y
        main_view_layout_object.y = 0
        main_view_layout_object.height = orginal_main_view_layout_height
    }
    function returnLastestHeightMainView(){
        main_view_layout_object.y = rootItem.lastest_main_view_layout_y
        main_view_layout_object.height = rootItem.lastest_main_view_layout_height
    }
    function returnLastestHeight(){
        rootItem.y = rootItem.lastest_y_before_up
        rootItem.height = rootItem.lastest_height_before_up
        isMaxSystemLog = false
    }

	function maxSystemLog(){
		rootItem.lastest_y_before_up = rootItem.y
		rootItem.lastest_height_before_up = rootItem.height
		rootItem.height = Math.round(rootItem.orginal_main_view_layout_height)
		rootItem.y = rootItem.main_view_layout_object.y
        isMaxSystemLog = true
	}

	function normalSystemLog(){
		rootItem.returnLastestHeight()
	}
    // signal changed
    onVisibleChanged:{
        systemLogVM.visibleChanged(visible)
        if(y>0){
            dashboardVM.isCovered = false
        }
        else{
            dashboardVM.isCovered = visible
        }
    }
    onYChanged: {
        systemLogVM.getCurrentY(y)
        systemLogVM.getCurrentHeight(1080-y)
        height = 1080-y
        if(y>0){
            dashboardVM.isCovered = false
        }
        else{
            dashboardVM.isCovered = true
        }
    }
    onFocusChanged: {
        if(isMaxSystemLog && rootItem.visible){
            if(main_view_layout_object.height === 1)
                rootItem.main_view_layout_object.height = rootItem.orginal_main_view_layout_height - rootItem.lastest_height_before_up
            upDownArrow._isClicked = false
            rootItem.normalSystemLog()
        }
    }
    Connections{
        target: controlBarVM
        ignoreUnknownSignals: true
        onSetCurrentIndexTab:{
            if(isMaxSystemLog && rootItem.visible){
                if(main_view_layout_object.height === 1)
                    rootItem.main_view_layout_object.height = rootItem.orginal_main_view_layout_height - rootItem.lastest_height_before_up
				upDownArrow._isClicked = false
                rootItem.normalSystemLog()
            }
        }
    }
	// Component default
    Component.onCompleted:{
        rootItem.orginal_main_view_layout_height = main_view_layout_object.height
        rootItem.lastest_main_view_layout_y = main_view_layout_object.y
        rootItem.lastest_main_view_layout_height = main_view_layout_object.height - rootItem.height
        systemLogVM.setTitleOfTab()
    }
    Component.onDestruction: {
		rootItem.closing()
    }

    //Background
    Rectangle{
        width: parent.width
        height: parent.height
		color: "black"
    }

    // Area for resize window
    Rectangle{
        id: mouse_area_title
        property var dragYArray : []
        property bool mouseDrag : false

        clip: true
        width: parent.width
        height: rootItem.mouseAreaHeight
        color: "#414141"
        radius: height / 2

        Behavior on width {NumberAnimation {duration: 1000 }}
        Timer{
            interval: 100 //ms
            repeat: true
            running: true
            triggeredOnStart: false
            onTriggered: {
                if(!mouse_area_title.mouseDrag) return;
                mouse_area_title.mouseDrag = false
                var avg = 0;
                for(var i=0; i < mouse_area_title.dragYArray.length; i++){avg += mouse_area_title.dragYArray[i]}
                avg = Math.round(avg / mouse_area_title.dragYArray.length)

                if(avg < 0) // Move up
                {
                    if(rootItem.height <  Math.round(rootItem.orginal_main_view_layout_height/2))
                    {
                        rootItem.height += Math.abs(avg)
                        rootItem.y -= Math.abs(avg)
                        rootItem.main_view_layout_object.height -= Math.abs(avg)
                    }
                    else
                    {
                        rootItem.height = Math.round(rootItem.orginal_main_view_layout_height/2)
                        rootItem.y = rootItem.main_view_layout_object.y + Math.round(rootItem.orginal_main_view_layout_height/2)
                        rootItem.main_view_layout_object.height = Math.round(rootItem.orginal_main_view_layout_height/2)
                    }
                }
                else // Move down
                {
                    var margin = mouse_area_title.height + rootItem.tabHeight
                    if(rootItem.height > margin)
                    {
                        rootItem.height -=  Math.abs(avg)
                        rootItem.y +=  Math.abs(avg)
                        rootItem.main_view_layout_object.height += Math.abs(avg)
                    }
                    else
                    {
                        rootItem.height = margin
                        rootItem.y = rootItem.main_view_layout_object.y + Math.round(rootItem.orginal_main_view_layout_height) - margin
                        rootItem.main_view_layout_object.height = rootItem.orginal_main_view_layout_height  - margin
                    }
                }

                mouse_area_title.dragYArray = []
            }

        }
        MouseArea{
            property bool pressAndHold: false
            property variant previousPosition
            enabled: !isMaxSystemLog
            anchors.fill: parent
            hoverEnabled: true
            pressAndHoldInterval: 100 // duration of 100ms
            drag.axis: Drag.YAxis
            drag.smoothed: false
            drag.filterChildren: true

            onEntered: {
                cursorShape = Qt.SizeVerCursor
                mouse_area_title.color = "#007ACC"
            }
            onExited: {
                if(pressAndHold) return;
                mouse_area_title.color = "#414141"
            }
            onPressAndHold: {
                pressAndHold = true
            }
            onReleased: {
                loginVM.updateIdleTimer()
                pressAndHold = false
                mouse_area_title.color = "#414141"
            }
            onPressed: {
                loginVM.updateIdleTimer()
                mouse_area_title.color = "#007ACC"
                previousPosition = mouseY
            }

            onMouseYChanged: {
                if(pressAndHold && pressedButtons == Qt.LeftButton)
                {
                    var dy = mouseY - previousPosition
                    mouse_area_title.dragYArray.push(dy)
                    mouse_area_title.mouseDrag = true
                }
            }
        }
    }

    //Tab switch
    Row{
        width: parent.width
        height: rootItem.tabHeight - rootItem.marginTitle_Tab
        y: mouse_area_title.height + rootItem.marginTitle_Tab
        spacing: 0

        TabBar {
            id: bar
            width: parent.width*0.96 - parent.spacing / 2
            height: parent.height
			contentHeight: parent.height
            background: Rectangle {
                anchors.fill: parent
                color: "#303030"
            }

            property string _fontFamily:  "Arial"
            property int _fontPointSize: 16

            Repeater{
                id: repeater_tab_cmd
                TabButton {
                    text: modelData
                    height: parent.height
                    property int _index: index
                    contentItem: Text {
                        text: parent.text
                        width: parent.width - moduleCbb.width - typeSignalCbb.width
                        font.capitalization: Font.Capitalize
                        font.family: bar._fontFamily
                        font.pointSize: bar._fontPointSize
                        font.bold: false
                        color: bar.currentIndex === parent._index ? "#81D4FA" : "#FFFFFF"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight
                    }
                    ComboBox{
                        id: moduleCbb
                        model:["ALL","WM","DM","FM","CM","IM","OM","TM","GM","LM","SM"]
                        height: parent.height * 1.25
                        anchors.left: parent.left
                        anchors.verticalCenter: parent.verticalCenter
                        visible: index === 1 ? true : false
                        down: true
                        onCurrentTextChanged: {
                            loginVM.updateIdleTimer()
                            moduleName = currentText
                            var ter = xavier_repeater.itemAt(_index);
                            if(ter === null) return
                            ter.changeTypeLogText(currentText,typeSignalCbb.currentText)
                            bar.currentIndex = index
                        }
                        popup: Popup {
                            y: 0 //moduleCbb.height - 1
                            width: moduleCbb.width
                            implicitHeight: Math.min(system_log_layout.height,moduleCbb.height * 12.5)
                            padding: 1
                            contentItem: ListView {
                                clip: true
                                implicitHeight: contentHeight
                                model: moduleCbb.popup.visible ? moduleCbb.delegateModel : null
                                currentIndex: moduleCbb.highlightedIndex
                                ScrollIndicator.vertical: ScrollIndicator { }
                            }
                        }
                    }
                    ComboBox{
                        id: typeSignalCbb
                        model:["ALL","INFO","WARN","ERROR"]
                        height: parent.height * 1.25
                        anchors.right: parent.right
                        anchors.verticalCenter: parent.verticalCenter
                        visible: false
                        onCurrentTextChanged: {
                            loginVM.updateIdleTimer()
                            typeSignal = currentText
                            bar.currentIndex = index
                            var ter = xavier_repeater.itemAt(_index);
                            if(ter === null) return
                            if(serverTerIndex != _index)
                                ter.changeTypeLogText("ALL",currentText)
                            else
                                ter.changeTypeLogText(moduleCbb.currentText,currentText)
                        }
                    }
                    onClicked: {
                        loginVM.checkEasterEgg(modelData)
                    }
                }
            }
            onCurrentIndexChanged: {
                systemLogVM.getCurrentTab(currentIndex)
				var ter = xavier_repeater.itemAt(currentIndex);
				if(ter === null) return
				if(ter.checkFirstTime){
					ter.scrollToEnd()
                    ter.checkFirstTime = false
				}
            }
        }

        Rectangle{
            width: parent.width*0.04 - parent.spacing / 2
            height: parent.height
            color: "transparent"
            Row{
                anchors.right: parent.right
                anchors.rightMargin: 2
                width: parent.width - anchors.rightMargin
                height: parent.height
                layoutDirection: Qt.RightToLeft
                spacing: 0
                Rectangle{
                    width: (parent.width - parent.spacing) /2
                    height: parent.height
                    color: "transparent"

                    ButtonMaterial{
                        anchors.centerIn: parent
                        _width: parent.width
                        _iconSourceOn: "close"
                        _colorDefault: "transparent"
                        _colorOverlayLow: "#A9A9A9"
                        _colorOverlayHigh: _colorOverlayLow
                        _radius:0
                        _size: width
                        _height: parent.height
                        _enableEffect: false
                        onHoveredChanged: {hovered ? parent.color = "#696969" : parent.color = "transparent"}

						ToolTip{
							visible: parent.hovered
							x: parent.width
							y: parent.height / 2
							delay: 100 // ms
							timeout: 2000 // ms
							contentItem: Text{
								color: "#B2C8DF"
								font.pointSize:10
								font.weight: Font.DemiBold
								font.family: "Adobe Gothic Std B"
								text: qsTr("Close")
							}
							background: Rectangle {
								border.color: "#e0e0e0"
								radius: 10
								gradient: Gradient {
									GradientStop { position: 0.0; color: "#232526" }
									GradientStop { position: 1.0; color: "#414345" }
								}
							}
						}
                        onClicked: {
                            loginVM.updateIdleTimer()
                            if(rootItem.visible){
                                rootItem.hide()
                                rootItem.returnOrgHeightMainView()
                            }
                        }
                    }
                }
                Rectangle{
                    width: (parent.width - parent.spacing) /2
                    height: parent.height
                    color: "transparent"
                    ButtonMaterial{
						id: upDownArrow
                        anchors.centerIn: parent
                        _width: parent.width
                        _iconSourceOn: "arrow-down"
                        _iconSourceOff: "arrow-up"
                        _colorDefault: "transparent"
                        _colorOverlayLow: "#A9A9A9"
                        _colorOverlayHigh: _colorOverlayLow
                        _radius:0
                        _size: width
                        _height: parent.height
                        _enableEffect: false
                        _isToggleButton: true
                        onHoveredChanged: {hovered ? parent.color = "#696969" : parent.color = "transparent"}

						ToolTip{
							visible: parent.hovered
							x: parent.width
							y: parent.height / 2
							delay: 100 // ms
							timeout: 1000 // ms
							contentItem: Text{
								color: "#B2C8DF"
								font.pointSize:10
								font.weight: Font.DemiBold
								font.family: "Adobe Gothic Std B"
                                text: upDownArrow._isClicked ? qsTr("Minimize") : qsTr("Maximize")
							}
							background: Rectangle {
								border.color: "#e0e0e0"
								radius: 10
								gradient: Gradient {
									GradientStop { position: 0.0; color: "#232526" }
									GradientStop { position: 1.0; color: "#414345" }
								}
							}
						}
                        onClicked: {
                            loginVM.updateIdleTimer()
                            _isClicked = !_isClicked
                            if(_isClicked){
                                rootItem.maxSystemLog()
                            }
                            else{
								if(main_view_layout_object.height === 1)
                                    rootItem.main_view_layout_object.height = rootItem.orginal_main_view_layout_height - rootItem.lastest_height_before_up
                                rootItem.normalSystemLog()
                            }
                        }
                    }
                }
            }
        }
    }

    // Content
    StackLayout{
        id: system_log_layout
		width:parent.width
        y: bar.height  + mouse_area_title.height
        height: parent.height - y
        currentIndex: (bar.count > 0 && repeater_tab_cmd.count > 0) ? bar.currentIndex : -1
        property var createDone: []
        onCurrentIndexChanged: {
            if(createDone.indexOf(currentIndex) < 0){
                createDone.push(currentIndex)
            }
        }
        FontLoader{
            id: myFontContent
            source: "qrc:/Fonts/FragmentMono-Regular.ttf"
        }

        Repeater{
			id: xavier_repeater
			BaseTerminal{
				_enabled: rootItem._enabled
				Component.onCompleted: {
					systemLogVM.callSetConfigTerminal(modelData)
				}
				onSendCurrentState: {
					systemLogVM.buttonCmdClicked(index, state, true)
				}
			}
        }

        Connections{
            target: systemLogVM
            ignoreUnknownSignals: true
            onSetContentTerminal:{
                var ter = xavier_repeater.itemAt(index - 1);
                if(ter === null) return
                ter.appendLine(logText)
                ter.separateSignal(logText)
            }
            onSetStateButtonTerminal:{
                var ter = xavier_repeater.itemAt(index - 1);
                if(ter === null) return
                ter.setStateButton(text)
            }
            onResetTerminal:{
                var ter = xavier_repeater.itemAt(index - 1);
                if(ter === null) return
                ter.removeAll()
            }
        }
    }
	Connections{
		target: main_view_layout_object
        ignoreUnknownSignals: true
		onWidthChanged:{
			rootItem.width = main_view_layout_object.width + 12
		}
	}
    Connections{
        target: systemLogVM
        ignoreUnknownSignals: true

        onSetTitleTerminal:{
            var array = []
            for(var i=0; i < list_title.length; i++)
                array.push(String(list_title[i]))
            repeater_tab_cmd.model = array
            xavier_repeater.model = array
        }
        onSetConfigTerminal:{
            var js_index = index - 1
            var js_bg_color = bg_color
            var js_text_font = text_font
            var js_text_size = text_size
            var js_line_spacing = line_spacing
            var js_limit_line = limit_line
            var js_line_scroll = scroll_line

            if(xavier_repeater.itemAt(js_index) === null)return
            xavier_repeater.itemAt(js_index).setConfigCmd(index, js_bg_color, js_text_font,
                                                          js_text_size,
                                                          js_line_spacing, js_limit_line,js_line_scroll)
        }
        onRequestHidden:{
            rootItem.hide()
            rootItem.returnOrgHeightMainView()
        }
        onRequestShow:{
            rootItem.height = lastHeight
            rootItem.show()
            rootItem.returnLastestHeightMainView()
        }
        onInitSystemlog:{
            rootItem.y = initY
            rootItem.height = 1080 - rootItem.y
			var temp = main_view_layout_object.height - initHeight
            if(temp === 0)
                main_view_layout_object.height = temp + 1
            else
                main_view_layout_object.height = temp

            main_view_layout_object.y = 0
            rootItem.hide()
            rootItem.returnOrgHeightMainView()
            bar.currentIndex = initIndex
			if((initY ===0) && (temp === 0)){
				upDownArrow._isClicked = true
				rootItem.lastest_y_before_up = 810
				rootItem.lastest_height_before_up = 270
			}
        }
    }
}
