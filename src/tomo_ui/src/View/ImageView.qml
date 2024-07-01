import QtQml 2.2
import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtGraphicalEffects 1.0

// My import
import ImageWriter 1.0
import "../Component"
import "FeatureView"
import "DashboardView"
// end

Item {
    id: image_view
    signal doubleClickImageView()

    property QtObject _viewModel: QtObject{}
    property QtObject _trackViewModel: QtObject{}

    property int _imageSize: 100
    property int _fontPointSize: 10
    property double _scaleTabbar: 0.0
    property color _textActivateNameDocColor: "aqua"
    property color _textDefaultNameDocColor: "#a9a9a9"
    property string  _name: ""
    property string _fontFamily:  "Adobe Gothic Std B"
    property string _private_control_button: ""
    property string _private_control_name: ""
    property string _tabControllerModelName: ""
    property alias _numberTab: bar.count
    property alias _currentIndexTab: bar.currentIndex

    focus: true
    clip: true

    Column{
        width: parent.width
        height:parent.height
        Column{
            width: parent.width
            height: 40
            spacing: 0
            Rectangle{
                //up line
                width: parent.width
                height: parent.height*0.07
                gradient: Gradient {
                    GradientStop { position: 0.0; color: "#232526" }
                    GradientStop { position: 1.0; color: "#414345" }
                }
            }
            Rectangle{
                //header
                id: header
                width: parent.width
                height: parent.height*0.86
                color: "transparent"
                clip : true

                property int sizeButton: 30
                property int spacingButton: 5
                property var tabBtnIndexArray: []
                property var tabBtnWidthArray: []

                function checkShowScrollBtn(){
                    if(tabBarView.visible){
                        if(bar.contentWidthTabbar<rectTabbar.width){
                            bar.x = 0
                        }else if (bar.x+bar.contentWidthTabbar<rectTabbar.width){
                            bar.x = rectTabbar.width-bar.contentWidthTabbar
                        }
                        if(bar.x < 0){
                            leftBtn.visible = true
                        }
                        else{
                            leftBtn.visible = false
                        }
                        if(bar.x+bar.contentWidthTabbar > rectTabbar.width){
                            rightBtn.visible = true
                        }
                        else{
                            rightBtn.visible = false
                        }
                    }
                }
                onWidthChanged: {
                    if(width < 0) return;
                    header.checkShowScrollBtn()
                }


                Item{
                    id: tabBarArea
                    anchors.left: parent.left
                    anchors.right: buttonListView.left
                    height: parent.height
                    clip:true
                    Flickable {
                        anchors.fill: parent
                        contentWidth: tomoModeView.width
                        boundsBehavior: Flickable.StopAtBounds
                        Row{
                            id: tomoModeView
                            visible: camName === 'TomoPose'?true:false
                            height: parent.height
                            spacing: 10
                            function settingChange(){
                                    var config = "";
                                    for(var i=0; i < tomoModeRP.count ; i++){
                                        var data = tomoModeRP.itemAt(i)
                                        if(data === undefined) continue;
                                        if(data.children[0] === undefined) continue;
                                        //config += data.children[0].text + ";" + data.children[1].children[0].textAt(data.children[1].children[0].currentIndex) + "|"
                                        config += data.children[1].currentIndex + "|"
                                    }
                                    modeTomoView.modeUserChoiceSlots(config)
                                }
                            Item {
                                height: parent.height
                                width: 5
                            }
                            Repeater{
                                id: tomoModeRP
                                model:modeTomoView === null ? null : modeTomoView.model
                                delegate: RowLayout{
                                    id: tomoModeRPDelegate
                                    height: parent.height
                                    spacing: 1
                                    Item{
                                        Layout.preferredHeight: parent.height
                                        Layout.preferredWidth: modeBoxName.implicitWidth
                                        Layout.alignment : Qt.AlignVCenter
                                        Text{
                                            id: modeBoxName
                                            anchors.centerIn: parent
                                            font.pointSize: 10
                                            font.family: "Meiryo UI Regular"
                                            text: qsTr(alias_name)+":"
                                            color: "#ffffff"
                                        }
                                    }
                                    ComboBoxCustom{
                                        id:combobox_custom
                                        Layout.preferredWidth: {
                                            switch(index) {
                                            case 0:
                                              return 140
                                            case 1:
                                             return 50
                                            case 2:
                                              return 50
                                            default:
                                                return 90
                                            }
                                        }

                                        Layout.preferredHeight: 35
                                        Layout.alignment : Qt.AlignVCenter
                                        maxWidthHPopup:{
                                            switch(index) {
                                              case 0:
                                                return 750
                                              case 1:
                                                return 600
                                              case 2:
                                                return 600
                                              default:
                                                return 300
                                            }
                                        }
                                        _model: combobox_custom.splitString(values_name, ";")
                                        _bgCombobox: "#303030"
                                        _bgPopup: "#141414"
                                        _opacityPopup: 0.6
                                        modelDataColor:"white"
                                        isVerticalPopup: false
                                        function splitString(text, character){
                                            return text.split(character)
                                        }
                                        onActivated: {
                                            if(!tomoModeRPDelegate.visible) return;
                                            tomoModeView.settingChange()
                                        }
                                    }
                                }
//                                Connections{
//                                    target: typeof(modeTomoView) === "undefine" ? null :modeTomoView
//                                    ignoreUnknownSignals: true
//                                    onApplyMode:{
//                                        if(acmd !== String(cmd)) return
//                                        if(avalue < 0)
//                                            avalue = combobox_custom.find(String(-avalue))
//                                        console.log("onApplyMode: Command=", acmd, "; Value=", avalue)
//                                        combobox_custom.currentIndex = avalue
//                                    }
//                                }
                            }
                        }
                    }


                    Row{
                        id: tabBarView
                        visible: camName === 'TomoPose'?false:true
                        height: parent.height
                        width: parent.width
                        spacing: header.spacingButton
                        Item{
                            width: header.sizeButton
                            height: width
                            anchors.verticalCenter: parent.verticalCenter
                            ButtonMaterial{
                                id: leftBtn
                                visible: false
                                _width: parent.width
                                _height:_width
                                _size: _width
                                _iconSourceOn: "chevron-left"
                                _iconSourceOff: ""
                                _durationAnimation: 100
                                _colorOverlayLow:"yellow"
                                _colorMouseOver:"#696969"
                                onClicked: {
                                    if(bar.contentWidthTabbar<rectTabbar.width){
                                        bar.x = 0
                                    }
                                    else if (bar.x + 30<0 && bar.x+bar.contentWidthTabbar<rectTabbar.width){
                                        bar.x = rectTabbar.width-bar.contentWidthTabbar
                                    }
                                    else if(bar.x + 30 >= 0){
                                        bar.x = 0
                                    }
                                    else{
                                        bar.x += 30
                                    }
                                }
                            }
                        }
                        Item{
                            id:rectTabbar
                            width: parent.width - 2*header.sizeButton - 2*parent.spacing
                            height:parent.height
                            clip: true
                            signal pressMouse(var mousex)
                            TabBar {
                                id: bar
                                y:2
                                height:parent.height
                                position: TabBar.Footer
                                clip:true
                                spacing: 0
                                property int numTabbtn: 0
                                property var _listWidthTabBtn: []
                                property int contentWidthTabbar: 0

                                signal setDocIndex(int docIndex)

                                onCurrentIndexChanged: {
                                    if(image_view._tabControllerModelName === "dashboardVM") return
                                    var modelControl = buttonListView.viewModel2id(image_view._tabControllerModelName)
                                    if(modelControl === null || modelControl === undefined) return
                                    modelControl.tabIndexChange(currentIndex)
                                }
                                onXChanged: {
                                    header.checkShowScrollBtn()
                                }

                                function changeView(docIndex){
                                    if(image_view._tabControllerModelName === "dashboardVM") return
                                    var modelControl = buttonListView.viewModel2id(image_view._tabControllerModelName)
                                    if(modelControl === null || modelControl === undefined) return
                                    modelControl.tabViewChange(docIndex)
                                }
                                function checkIndex(mousex){
                                    var total = 0
                                    var index = 0
                                    for(var i=0;i<bar.count;i++){
                                        if(total < mousex){
                                            total += _listWidthTabBtn[i]
                                            total += bar.spacing
                                            index = i
                                        }
                                    }
									currentIndex = index
                                    bar.setDocIndex(index)
                                }
                                Repeater {
                                    model: _trackViewModel != null ? _trackViewModel.imageModel : null
                                    TabButton {
                                        id:tabbtn
                                        text: qsTr(name)
                                        y:-5
                                        width: Math.round(implicitWidth)
                                        contentItem: Text {
                                            text: parent.text
                                            font.family: image_view._fontFamily
                                            font.pointSize: image_view._fontPointSize
                                            font.capitalization: Font.MixedCase
                                            horizontalAlignment: Text.AlignHCenter
                                            verticalAlignment: Text.AlignVCenter
                                            opacity: enabled ? 1.0 : 0.3
                                            color: parent.down ? "#B6E0DE" : "#F8F8FA"
                                            elide: Text.ElideNone
                                         }
                                        function setDoc(){
                                            if(model.imageViewVM === null || model.imageViewVM === undefined) return
                                            image_view._viewModel = model.imageViewVM
                                            image_view._name = model.name
                                        }

                                        onClicked: {
                                            bar.changeView(index)
                                            setDoc()
                                        }

                                        Component.onCompleted: {
                                            if (index === 0) setDoc()
                                            bar._listWidthTabBtn[bar.numTabbtn] = width
                                            bar.numTabbtn += 1
                                            header.tabBtnIndexArray.push(index)
                                            header.tabBtnWidthArray.push(Math.round(tabbtn.implicitWidth))
                                        }

                                        Connections{
                                            target: bar
                                            ignoreUnknownSignals: true
                                            onSetDocIndex:{
												if(index === docIndex){
													bar.changeView(docIndex)
													tabbtn.setDoc()
												}
                                            }
                                        }

                                        MouseArea{
                                            anchors.fill: parent
                                            hoverEnabled: true
                                            propagateComposedEvents: true
                                            property int step: 30
                                            onWheel: {
                                                if(leftBtn.visible || rightBtn.visible){
                                                    if(wheel.angleDelta.y>0){
                                                        if(bar.x - step > rectTabbar.width - bar.contentWidthTabbar){
                                                            bar.x -= step
                                                            leftBtn.visible = true
                                                        }
                                                        else{
                                                            bar.x = -(bar.contentWidthTabbar - rectTabbar.width)
                                                            rightBtn.visible = false
                                                        }
                                                    }
                                                    else if (wheel.angleDelta.y<0){
                                                        if(bar.x + step > 0){
                                                            bar.x = 0
                                                            leftBtn.visible = false
                                                        }
                                                        else{
                                                            bar.x+= step
                                                            rightBtn.visible = true
                                                        }
                                                    }
                                                }
                                            }
                                            onPressed: mouse.accepted = false
                                        }
                                    }
                                }

                                Connections{
                                    target: buttonListView.viewModel2id(image_view._tabControllerModelName)
                                    ignoreUnknownSignals: true
                                    onSetTabViewIndex:{
                                        if(bar.currentIndex !== docIndex){
											bar.currentIndex = docIndex
                                            bar.changeView(docIndex)
                                            bar.setDocIndex(docIndex)
                                        }
                                    }
                                }
                                Connections{
                                    target: rectTabbar
                                    ignoreUnknownSignals:  true
                                    onPressMouse: bar.checkIndex(mousex)
                                }
                                Component.onCompleted: {
                                    for(var i=0;i<bar._listWidthTabBtn.length;i++){
                                        bar.contentWidthTabbar += bar._listWidthTabBtn[i]
                                    }
                                    bar.contentWidthTabbar += bar.spacing * bar.count
                                }
                            }
                            MouseArea{
                                anchors.fill: parent
                                hoverEnabled: true
                                propagateComposedEvents: true
                                drag.target: bar
                                drag.axis: "XAxis"
                                drag.minimumX:  bar.contentWidthTabbar <= rectTabbar.width ? 0 : rectTabbar.width-bar.contentWidthTabbar
                                drag.maximumX: 0
                                drag.filterChildren: true
                                onClicked: rectTabbar.pressMouse(mouseX - bar.x)
                            }
                        }
                        Item{
                            width: header.sizeButton
                            height: width
                            anchors.verticalCenter: parent.verticalCenter
                            ButtonMaterial{
                                id:rightBtn
                                visible: false
                                _width: parent.width
                                _height:_width
                                _size: _width
                                _iconSourceOn: "chevron-right"
                                _iconSourceOff: ""
                                _durationAnimation: 100
                                _colorOverlayLow: "yellow"
                                _colorMouseOver:"#696969"
                                onClicked: {
                                    if(bar.contentWidthTabbar<rectTabbar.width){
                                        bar.x = 0
                                    }
                                    else if(bar.x - 30 > rectTabbar.width - bar.contentWidthTabbar){
                                        bar.x -= 30
                                    }
                                    else{
                                        bar.x = -(bar.contentWidthTabbar - rectTabbar.width)
                                    }
                                }
                            }
                        }
                    }
                }
                ListView{
                    // Tool in doc title bar
                    id: buttonListView
                    clip: true
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    width:  60*count + 10
                    height: parent.height*0.9
                    orientation: ListView.Horizontal
                    layoutDirection: Qt.RightToLeft
                    model: common_Button
                    property bool isRvizFull: false
                    delegate: Item{
                        width: 60
                        height: buttonListView.height
                        ButtonMaterial{
                            id: button
                            hoverEnabled: true
                            anchors.centerIn: parent
                            _width: parent.width*0.9
                            _height: parent.height*0.9
                            _size: _height*0.8
                            _iconSourceOn: typeof bIconOn === 'undefined' ? "": bIconOn //=== "cast"?"cast":bIconOn === "arrow-collapse-all"? "arrow-expand-all" : bIconOn
                            _iconSourceOff: typeof bIconOff === 'undefined' ? "": bIconOff //=== "cast"? "cast-off":bIconOn === "arrow-collapse-all"? "arrow-collapse-all" : bIconOff
                            _isToggleButton: isToogleButton
                            _durationAnimation: 10
                            enabled: bName === "hide" ? !buttonListView.isRvizFull : true
                            opacity: enabled? 1 : 0.5
                            property var tomoDropComponent: null
                            property var tomoDropWindow: null
                            ToolTip{
                                id: buttonTooltip
                                text: bName.replace("_"," ")
                                font.capitalization: Font.Capitalize
                            }

                            onHoveredChanged: {
                                if(button.hovered){
                                // var point = mapFromGlobal(0, 0)
                                // buttonTooltip.x = -point.x
                                // buttonTooltip.y = -point.y+button.height
                                }
                                buttonTooltip.visible = button.hovered
                            }
                            onClicked: {
                                if(button._isToggleButton) button._isClicked = !button._isClicked
                                buttonListView.viewModel2id(image_view._tabControllerModelName).clickToolButton(track.camName, bName, button._isClicked)
                                master_app.broadcastPressButton(track.camName, bName, button._isClicked)
                                // if(bName === "open-file") obligatory_FileDialogView.open()
                                if(bName === "auto_switch") {
                                }
                                if(bName === "hide"){
                                    rootDashBoard.hideTomoPose()
                                    for(var k = 0 ; k < vision_rpt.count; k++)
                                    {
                                        var itemreset = vision_rpt.itemAt(k)
                                        itemreset.visible = true
                                        itemreset.Layout.columnSpan = 1
                                        if(k === 0)
                                            itemreset.Layout.rowSpan = 2
                                        else
                                            itemreset.Layout.rowSpan = 1
                                    }
                                }
                                if(bName === "fullscreen"){
                                    rootDashBoard.tomoPoseIsMax = !rootDashBoard.tomoPoseIsMax
                                    if(rootDashBoard.tomoPoseIsMax == true){
                                        hideTomoEye()
                                        buttonListView.isRvizFull = true
                                    }
                                    else if(rootDashBoard.tomoPoseIsMax == false){
                                        showNormalView()
                                        buttonListView.isRvizFull = false
                                    }
                                    // if(button._isClicked){
                                    //     rootDashBoard.hideTomoEye()
                                    // }
                                    // else{
                                    //     rootDashBoard.showNormalView()
                                    // }
                                }
                                if(bName === "export"){
                                    rootDashBoard.listDirVisible = true
                                }
                            }

                            Connections{
                                target: buttonListView.viewModel2id(image_view._tabControllerModelName)
                                ignoreUnknownSignals: true
                                onButtonDocStatusChanged:{
                                    if(camName === track.camName && docName === bName && currentIsClick !== button._isClicked){
                                        button._isClicked = currentIsClick
                                    }
                                }
                                // onSetFinishInspectBtn:{
                                //     if(camName === track.camName && bName === "inspect"){
                                //         button._isClicked = false
                                //     }
                                // }
                                // onSetEnableLoadBtn:{
                                //     if(camName === track.camName && bName === "open-file"){
                                //         button.enabled = value
                                //     }
                                // }
                                // onSetEnableInspectBtn:{
                                //     if(camName === track.camName && bName === "inspect"){
                                //         button.enabled = value
                                //     }
                                // }
                                onSetEnableLiveBtn:{
                                    if(camName === track.camName && bName === "live"){
                                        button.enabled = value
                                    }
                                }
								onSetEnableRvizBtn:{
									if(camName === track.camName && bName === "rviz"){
                                        button.enabled = value
                                    }
								}
                                onAutoSwitchChanged:{
                                    if(camName === track.camName && bName === "auto_switch"){
                                        button._isClicked = value
                                    }
                                }
                                onLiveSwitchChanged:{
                                    if(camName === track.camName && bName === "live"){
                                        button._isClicked = value
                                    }
                                }
                            }
							Connections{
								target: productionVM
								ignoreUnknownSignals: true
								onEnabledForRunLotChanged:{
									if(camName === track.camName && bName === "live"){
										button.enabled  = value
									}
								}
							}
                            Component.onCompleted: {
                                if(camName === track.camName && bName === "auto_switch"){
                                    button._isClicked = true
                                }
                            }
                            Connections{
                                target: master_app
                                ignoreUnknownSignals: true
                                onPowerUpStateChanged:{
                                    if(bName === "export")
                                        button.enabled = !state
                                }
                            }
                        }
                    }

                    function viewModel2id(vmName){
                        if (vmName === "tomoVM") return tomoVM
                        if (vmName === "headVM") return headVM
                        if (vmName === "shipperVM") return shipperVM
                        if (vmName === "pimVM") return pimVM
                    }
                }
                ListModel {
                    id: common_Button
                }
                Timer{
                    id:timerGetWidth
                    interval: 2000
                    onTriggered: {
                        bar.width = bar.contentWidthTabbar
                        if(bar.width < rectTabbar.width) rectTabbar.width = bar.width
                        if(bar.x === 0) leftBtn.visible = false
                    }
                }
                Component.onCompleted: {
                    timerGetWidth.running = true
                }
            }
            Rectangle{
                //down line
                width: parent.width
                height: parent.height*0.07
                gradient: Gradient {
                    GradientStop { position: 0.0; color: "#000000" }
                    GradientStop { position: 1.0; color: "#434343" }
                }
            }
        }

        Rectangle{
            id: image_container
            width: parent.width
            height:parent.height - 40
            y: parent.y + 40
            color: "#000000"
            clip: true

            onWidthChanged: {
               zoomTool.anchorZoomAtRankedSector = -1
               zoomTool.sectorViewMode = false
            }
            onHeightChanged: {
               zoomTool.anchorZoomAtRankedSector = -1
               zoomTool.sectorViewMode = false
            }

            Flickable{
                id: flick
                width: parent.width
                height: parent.height*(zoomTool.sectorViewMode ? 0.7 : 1)
                Layout.fillWidth: true
                Layout.fillHeight: true
                contentHeight: image_writer.height
                contentWidth: image_writer.width
                interactive: false

                property int xRepeater
                property int oldxRepeater
                property int yRepeater
                property int oldyRepeater
                property bool firstLoop: true

                ImageWriter{
                    id: image_writer
                    width: flick.width
                    height: flick.height
                    ui_image: typeof _viewModel.image === "undefined" ? null : _viewModel.image
                    roiY: 0
                    roiWidth: _imageSize*width/height
                    roiHeight: _imageSize
                    property int backgroundSizeQml: roiX<0?-1*roiX*height/roiHeight:0
                    property double zoomRatio: roiHeight/ui_height // < 1
                    property double pixImage2pixUI: height/ui_height
                    property bool lockRtBound: false

                    onRequestZoomFit: zoomTool.doZoomFit()
                    onUpdateSourceImage: {
                        var oldimageSize =_imageSize
                        if (_viewModel.viewMode === 2){
                            image_writer.requestStretch(image_container.width,image_container.height)
                            _imageSize = ui_height
                        } else {
                            if ((width*ui_height - ui_width*height > 0) ^ (_viewModel.viewMode === 0)){
                                _imageSize = ui_width*height/width
                            } else {
                                _imageSize = ui_height
                            }
                        }

                        if (oldimageSize !==_imageSize){
                            image_writer.return2bound()
                            zoomTool.doZoomFit()
                            image_writer.update()
                        }
                    }
                    onWidthChanged: updateSourceImage()
                    onHeightChanged: updateSourceImage()
                    onUi_widthChanged: {
                        return2bound()
                        image_writer.update()
                    }
                    onUi_heightChanged: {
                        _imageSize = ui_height
                        return2bound()
                    }

                    function return2bound(){
                        if (roiWidth > ui_width){
                            roiX = (roiWidth - ui_width)/-2
                        }
                        else {
                            if (roiX < 0) roiX = 0
                            if (roiX + roiWidth > ui_width) roiX = ui_width - roiWidth
                        }
                        if (roiHeight > ui_height) roiY = (roiHeight - ui_height)/-2
                        else {
                            if (roiY < 0) roiY = 0
                            if (roiY + roiHeight > ui_height) roiY = ui_height - roiHeight
                        }
                   }

                    MouseArea {
                        id: zoomTool
                        focus: true

                        property int zoomTime: 0
                        property bool pressDoc: false
                        property var flickQueue: []
                        property int anchorZoomAtRankedSector: -1
                        property bool sectorViewMode: false
                        property int waitingMouseSignal: 0
                        property var tmpBuffer : []
                        property bool isCtrlPress: false
                        property bool isKeySToggle: false
                        property bool isKeyWToggle: false
                        property double scaleX4Flick
                        property double scaleY4Flick
                        property int prePressX
                        property int prePressY
                        property double preRoiX
                        property double preRoiY

                        signal sizeChanged()

                        function zoomInAtRectangle(pX, pY){
                            var defectIDinMouse = -1
                            var i = 0

                            if (defectIDinMouse === -1){
                                toprightZoomImaageWriter.close()
                                return
                            }
                            toprightZoomImaageWriter.update()
                        }
                        function activeFlick(scaleX, scaleY,isZoomIn){
                            if (isZoomIn){
                                zoomInFunction(scaleX*flick.contentWidth, scaleY*flick.contentHeight)
                            }
                            else {
                              zoomOutFunction(scaleX*flick.contentWidth, scaleY*flick.contentHeight)
                            }
                        }
                        function zoomRequest(scaleX, scaleY, zoomValue){
                            while (zoomValue !== 0){
                               if (zoomValue > 0){
                                   zoomOutFunction(scaleX*flick.contentWidth,scaleY*flick.contentHeight)
                                   zoomValue--;
                               } else {
                                   zoomInFunction(scaleX*flick.contentWidth,scaleY*flick.contentHeight)
                                   zoomValue++;
                               }
                            }
                        }

                        function processMouseSignal_Move(buttonID){
                            switch(buttonID){
                                case 2:
                                    mouseIndicatorInfomation.x = zoomTool.mouseX - 5
                                    mouseIndicatorInfomation.y = zoomTool.mouseY - 5
                                    break;
                            }
                        }
                        function processMouseSignal_Click(buttonID){
                            switch(buttonID){
                                case 2:
                                    mouseIndicatorInfomation.width = 0
                                    zoomTool.sectorViewMode = true
                                    waitingMouseSignal = 0
                                    zoomTool.cursorShape = Qt.ArrowCursor
                                    break;
                                }
                        }

                        function zoomOutFunction(pointX, pointY){
                            if (zoomTool.zoomTime > 20){ // Maximum 33
                                return
                            }
                            image_writer.roiX += pointX*0.2*image_writer.roiWidth/width
                            image_writer.roiY += pointY*0.2*image_writer.roiHeight/height
                            image_writer.roiWidth/=1.25
                            image_writer.roiHeight/=1.25
                            if (!image_writer.lockRtBound){
                                image_writer.return2bound()
                            }
                            image_writer.update()
                            zoomTool.zoomTime += 1
                        }
                        function zoomInFunction(pointX, pointY){
                            if (image_writer.roiHeight >=_imageSize){
                                return
                            }
                            image_writer.roiX -= pointX*0.25*image_writer.roiWidth/width
                            image_writer.roiY -= pointY*0.25*image_writer.roiHeight/height
                            image_writer.roiWidth*=1.25
                            image_writer.roiHeight*=1.25
                            if (!image_writer.lockRtBound){
                                image_writer.return2bound()
                            }
                            image_writer.update()
                            zoomTool.zoomTime -= 1
                        }
                        function doZoomFit(){
                            zoomTool.zoomTime = 0
                            parent.roiX = -1*((width - height)/2)*_imageSize/height
                            parent.roiY = 0
                            parent.roiWidth = _imageSize*parent.width/parent.height
                            parent.roiHeight= _imageSize
                            image_writer.return2bound()
                            image_writer.update()

                        }


                        width: image_writer.width
                        height: image_writer.height
                        acceptedButtons: Qt.LeftButton | Qt.RightButton
                        hoverEnabled: true
                        onWidthChanged: heightChanged()
                        onHeightChanged: {
                            zoomTool.doZoomFit()
                        }
                        onWheel:{
                            toprightZoomImaageWriter.close()
                            if((wheel.angleDelta.y > 0)){
                                zoomOutFunction(mouseX, mouseY)
                            }
                            if((wheel.angleDelta.y < 0)){
                                zoomInFunction(mouseX, mouseY)
                            }
                        }
                        onClicked:{
                            if(mouse.button == Qt.RightButton){
                                zoomTool.doZoomFit()
                            }
                            else if (mouse.button == Qt.LeftButton && zoomTool.waitingMouseSignal){
                                zoomTool.processMouseSignal_Click(zoomTool.waitingMouseSignal)
                            }
                        }
                        onEntered: {
                                zoomTool.forceActiveFocus(Qt.MouseFocusReason)
                        }
                        onExited: {
                            zoomTool.forceActiveFocus(Qt.MouseFocusReason)
                        }
                        onPositionChanged: {
                            image_view.focus = true
                            if (zoomTool.waitingMouseSignal)
                                circleButton.processMouseSignal_Move(circleButton.waitingMouseSignal)
                            if(containsMouse)
                                // _viewModel.positionMouseHoverInImageUI(image_writer.roiWidth*mouseX/width + image_writer.roiX, image_writer.roiHeight*mouseY/height + image_writer.roiY)
                            if (containsPress){
                                image_writer.roiX = preRoiX - (mouseX - prePressX)*image_writer.roiWidth/width
                                image_writer.roiY = preRoiY - (mouseY - prePressY)*image_writer.roiHeight/height
                                image_writer.return2bound()
                                image_writer.update()
                            }
                       }
                        onPressed: {
                            if(isCtrlPress && (mouse.button === Qt.LeftButton)){
                            }
                            else if (isCtrlPress && (mouse.button === Qt.RightButton)){
                                zoomTool.doZoomFit()
                                zoomTool.anchorZoomAtRankedSector = -1
                                zoomTool.sectorViewMode = false
                            }
                            else if (isKeyWToggle){
                            }
                            prePressX = mouseX
                            prePressY = mouseY
                            preRoiX = image_writer.roiX
                            preRoiY = image_writer.roiY

                            if(mouse.button === Qt.LeftButton){
                            }
                            if((mouse.button === Qt.RightButton) && (_name.search("TomO") === -1) ){
                            }
                            if(mouse.button === Qt.LeftButton){
                                zoomInAtRectangle(mouseX,mouseY)
                            }
                        }
                        onDoubleClicked: {
                            if(mouse.button === Qt.RightButton) return
                            image_view.doubleClickImageView()
                        }
                        onSectorViewModeChanged: {
                            if (sectorViewMode){
                                image_writer.roiHeight*= 0.7
                            }
                            else {
                               image_writer.roiHeight/= 0.7
                            }
                            image_writer.update()
                        }
                    }
               }
               //background image view
               Rectangle{
                   width: image_writer.roiX<0?-1*image_writer.roiX*image_writer.height/image_writer.roiHeight:0
                   height: parent.height
                   color: typeof _viewModel.averageIntensity === "undefined" ? "black" : _viewModel.averageIntensity
                   anchors.left: parent.left
               }
               Rectangle{
                   height: image_writer.roiY<0?-1*image_writer.roiY*image_writer.height/image_writer.roiHeight:0
                   width: parent.width
                   anchors.top: parent.top
                   color: typeof _viewModel.averageIntensity === "undefined" ? "black" : _viewModel.averageIntensity
               }
               Rectangle{
                   height: image_writer.roiY + image_writer.roiHeight > image_writer.ui_height?(image_writer.roiY + image_writer.roiHeight- image_writer.ui_height)*image_writer.height/image_writer.roiHeight:0
                   width: parent.width
                   color: typeof _viewModel.averageIntensity === "undefined" ? "black" : _viewModel.averageIntensity
                   anchors.bottom: parent.bottom
               }
               Rectangle{
                   width: image_writer.roiX + image_writer.roiWidth > image_writer.ui_width?(image_writer.roiX + image_writer.roiWidth- image_writer.ui_width)*image_writer.height/image_writer.roiHeight:0
                   height: parent.height
                   color: typeof _viewModel.averageIntensity === "undefined" ? "black" : _viewModel.averageIntensity
                   anchors.right: parent.right
                   Text{
                       id: textSourceImage
                       anchors.fill: parent
                    //    text: null
                       font.pixelSize: 15
                       font.bold: true
                       font.capitalization: Font.Capitalize
                       wrapMode: Text.WordWrap
                   }
               }
            }

            //this part for FrameCount
            ListView{
                 id: frameCountListView
                 width: parent.width
                 height: parent.height
                 interactive: false
                 property int spacingOf2Text: 5
                 property int maxContentWidth: 0
                 model: frameCountListModel
                 delegate: Rectangle{
                     height: 17
                     FontLoader { id: webFont; source: "../Fonts/BlackOpsOne-Regular.ttf" }
                     Text {
                         id: nameText
                         text: model.name
                         font.family: webFont.name
                         font.pixelSize: 15

                         LinearGradient {
                             anchors.fill: parent
                             source: parent
                             antialiasing: true
                             gradient: Gradient {
                                 GradientStop { position: 0; color: "#000000"}
                                 GradientStop { position: 1; color: "#0f9b0f"}
                             }
                             ShaderEffectSource{
                                 recursive: true
                             }
                         }
                         Component.onCompleted: {
                             if (frameCountListView.maxContentWidth < contentWidth){
                                 frameCountListView.maxContentWidth = contentWidth
                             }
                         }
                     }
                     Text {
                         id: valueText
                         text: model.value
                         x: frameCountListView.maxContentWidth + frameCountListView.spacingOf2Text
                         font.family: webFont.name
                         font.pixelSize: 15

                         LinearGradient  {
                             anchors.fill: parent
                             source: parent
                             gradient: Gradient {
                                 GradientStop { position: 0; color: "#cb2d3e" }
                                 GradientStop { position: 1; color: "#ef473a" }
                             }
                             ShaderEffectSource{
                                 recursive: true
                             }
                         }
                     }
                 }

                 ListModel{
                     id: frameCountListModel
                 }

                 function setFrameCount(colFirst, colSec){
                     frameCountListView.maxContentWidth = 0
                     frameCountListModel.clear()
                     var contentWList = []
                     for(var i=0; i<colFirst.length; i++){
                         frameCountListModel.append({"name": colFirst[i], "value": colSec[i]})
                     }
                 }
            }
            //end part for frame count

            Rectangle{
                id: recIfDocisNull
                anchors.fill: parent
                visible: typeof _viewModel.noSignal === "undefined" ? null : _viewModel.noSignal
                color: "transparent"
                Rectangle{
                    id: bordertextWhenDocisNull
                    width: textWhenDocisNull.contentWidth + 200
                    height: textWhenDocisNull.contentHeight*1.4
                    anchors.centerIn: parent
                    color: "#404040"
                    clip: true
                    visible: false
                    Rectangle{
                        width: parent.width
                        height: 14
                        color: "transparent"
                        anchors.verticalCenter: parent.top
                        clip: true
                        Rectangle{
                            width: parent.width
                            height: 2
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Rectangle{
                            width: 40
                            height: 40
                            rotation: 45
                            anchors.verticalCenter: parent.verticalCenter
                            anchors.horizontalCenter: parent.left
                        }
                        Rectangle{
                            width: 40
                            height: 40
                            rotation: 45
                            anchors.verticalCenter: parent.verticalCenter
                            anchors.horizontalCenter: parent.right
                        }
                    }
                    Rectangle{
                        width: parent.width
                        height: 14
                        color: "transparent"
                        anchors.verticalCenter: parent.bottom
                        clip: true
                        Rectangle{
                            width: parent.width
                            height: 2
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Rectangle{
                            width: 40
                            height: 40
                            rotation: 45
                            anchors.verticalCenter: parent.verticalCenter
                            anchors.horizontalCenter: parent.left
                        }
                        Rectangle{
                            width: 40
                            height: 40
                            rotation: 45
                            anchors.verticalCenter: parent.verticalCenter
                            anchors.horizontalCenter: parent.right
                        }
                    }
                }
                Text {
                    id: textWhenDocisNull
                    anchors.fill: parent
                    text: _name
                    font.family: "Helvetica"
                    font.pointSize: 24
                    color: "white"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                Text {
                    id: textNoSignal
                    anchors.top: bordertextWhenDocisNull.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    height: contentHeight + 20
                    text: ""
                    font.family: "Helvetica"
                    font.pointSize: 15
                    color: "gray"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
            Canvas{
                id: dashRectangle
                anchors.fill: parent
                function drawDashRectangle(dx,dy,dw,dh){
                    deleteDashRectangle()
                    drawDashLine(dx,dy,dx + dw,dy)
                    drawDashLine(dx + dw,dy,dx+dw,dy+dh)
                    drawDashLine(dx + dw,dy+dh,dx,dy+dh)
                    drawDashLine(dx,dy+dh,dx,dy)
                    requestPaint()
                }

                function drawDashLine(x1,y1,x2,y2){
                    var ctx = getContext("2d");
                    ctx.strokeStyle = "purple";
                    ctx.lineWidth = 2;

                    var dashType = [10,5]
                    var dx = x2 - x1
                    var dy = y2 - y1
                    var length = Math.sqrt(dx*dx + dy*dy)
                    var remaningLength = 0
                    var dashListCounter = 0
                    ctx.beginPath();
                    while (remaningLength + dashType[dashListCounter]/length < 1){
                        if (dashListCounter === 0){
                            ctx.moveTo(x1 + remaningLength*dx, y1 + remaningLength*dy)
                            ctx.lineTo(x1 + (remaningLength + dashType[dashListCounter]/length)*dx,y1 + (remaningLength + dashType[dashListCounter]/length)*dy)
                            ctx.stroke()
                        }

                        remaningLength += dashType[dashListCounter]/length
                        dashListCounter = (dashListCounter+1)%(dashType.length)
                    }
                }

                function deleteDashRectangle(){
                    var ctx = getContext("2d");
                    ctx.reset();
                    requestPaint()
                }
            }
            Rectangle{
                color: "transparent"
                border.color: "#302b63"
                border.width: 2
                anchors.right: parent.right
                anchors.top: parent.top
                clip: true
                property int _width: image_view.width*0.25
                property int _height: image_view.width*0.25

                Rectangle{
                    id: toprightZoomImage
                    anchors.centerIn: parent
                    width : parent._width - 4
                    height : parent._height - 4
                    color: "transparent"
                    clip: true

                    ImageWriter{
                        id: toprightZoomImaageWriter
                        anchors.centerIn: parent
                        height: parent.height
                        width: height*image_writer.width/image_writer.height
                        ui_image: typeof _viewModel.image === "undefined" ? null : _viewModel.image
                        property double alpha: 12
                        property int sizeDashWindow: 2*height/Math.pow(1.2,alpha)

                        function setLocation(cx,cy){
                            roiWidth = image_writer.roiWidth/alpha
                            roiHeight = image_writer.roiHeight/alpha
                            roiX = image_writer.roiX + (cx - image_writer.width/2)*image_writer.roiWidth/image_writer.width + (image_writer.roiWidth - roiWidth)/2
                            roiY = image_writer.roiY + (cy - image_writer.height/2)*image_writer.roiHeight/image_writer.height + (image_writer.roiHeight - roiHeight)/2
                            parent.parent.width = parent.parent._width
                            parent.parent.height = parent.parent._height
                            update()
                            dashRectangle.drawDashRectangle(cx-sizeDashWindow/2, cy-sizeDashWindow/2, sizeDashWindow,sizeDashWindow)
                        }
                        function close(){
                            parent.parent.width = 0
                            parent.parent.height = 0
                            dashRectangle.deleteDashRectangle()
                        }
                    }
                }
            }
        }
    }

    // signal changed
    on_ViewModelChanged:{
       image_writer.clearTmpImage()
    }
    on_Private_control_buttonChanged:{
        var listButton4enable = _private_control_button.split(" ")
        var listNameButton = _private_control_name.split(" ")
        for (var i=0; i<listButton4enable.length; i++){
            var in4Button = listButton4enable[i].split(";")
            var in4Checkbox = in4Button[0].split("|")
            common_Button.append({bName: listNameButton[i], bIconOn: in4Checkbox[0] , bIconOff: in4Checkbox[1] , isToogleButton: in4Button[1] === "t"})
        }
    }

    // Connections{
	// 	target: typeof optispecVM === 'undefined' ? null : optispecVM
    //     ignoreUnknownSignals: true
    //     onInitPathVideo:obligatory_FileDialogView._pathInit = path
    // }

    // FileDialogView{
    //     id:obligatory_FileDialogView
    //     function typeFile(){
    //         if(image_view._tabControllerModelName === "optispecVM") return [ "All video file (*.avi *.mp4)", "All file (*.*)"]
	// 		else if(image_view._tabControllerModelName === "bomVM") return [ "All image (*.jpg *.png *.jpeg)", "All file (*.*)"]
	// 		else return ["All file (*.*)"]
    //     }
    //     _extension: typeFile()
    //     onAccepted:buttonListView.viewModel2id(image_view._tabControllerModelName).loadImageFromFile(fileUrl)
    //     onRejected:{}
    //     onVisibleChanged:{}
    // }
}
