import QtQml 2.3
import QtQuick 2.9
import QtQuick.Window 2.3
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2

// My import
import "../../View"
import "../../Component"
import "../../Component/MaterialDesign"
//end

Item {
    id: rootDashBoard
    width: parent.width
    height: parent.height
    clip: true
    opacity: enabled? 1:0.5
    signal closeSettingPage()
    signal visibleSettingPage()

    property QtObject _dashModel: QtObject{}

    property int indexOneDoc: -1
    property bool tomoPoseIsMax: false
    property bool tomoEyeIsMax: false
    property bool isTomoPoseHide: false
    property bool isCovered: false
    property bool listDirVisible: false

    function hideTomoPose(){
        tomoPoseRec.visible = false
        tomoEyeRec.width    = tomoEyeRec.parent.width - tomoEyeRec.parent.spacing /2
        hideArea.width      = hideArea.parent.width*0.01
    }
    function hideTomoEye(){
        tomoEyeRec.visible  = false
        tomoPoseRec.width   = tomoPoseRec.parent.width - tomoPoseRec.parent.spacing /2
    }
    function showNormalView(){
        tomoPoseIsMax = false
        tomoEyeIsMax = false
        hideArea.width      = 0
		tomoPoseRec.visible = true
        tomoPoseRec.width   = tomoPoseRec.parent.width*0.5 - tomoPoseRec.parent.spacing
        tomoEyeRec.visible  = true
        tomoEyeRec.width    = tomoEyeRec.parent.width*0.5 - tomoEyeRec.parent.spacing
        for(var k = 0 ; k < vision_rpt.count; k++)
        {
            var itemreset = vision_rpt.itemAt(k)
            itemreset.visible = true
            itemreset.Layout.columnSpan = model_dashboard_TOMOC2.get(k)._scaleWidth
            itemreset.Layout.rowSpan = model_dashboard_TOMOC2.get(k)._scaleHeight
        }
    }

    onVisibleChanged: {
        visibleSettingPage()
    }

    ListModel {
        id: model_dashboard_TOMOC1
    }
    ListModel {
        id: model_dashboard_TOMOC2
    }

    Rectangle{
        width: parent.width
        height: parent.height
        color: "transparent"
        Row{
            width: parent.width
            height: parent.height
            spacing: 8
            Rectangle{
                id: hideArea
                width: 0
                height: parent.height
                color: "transparent"
                ButtonMaterial{
                    _width:parent.width
                    _height:parent.height
                    _size: _width+4
                    _iconSourceOn: "chevron-right"
                    _iconSourceOff: ""
                    onClicked: {
                        showNormalView()
                    }
                    ToolTip{
                        parent: parent
                        visible: parent.hovered
                        text: "Unhide"
                        x: 10
                        y: parent.height / 2
                        background: Rectangle{
                            color: "black"
                            border{
                                width: 0.5
                                color: "gray"
                            }
                        }
                    }
                }
            }
            Item{
                id: tomoPoseRec
                width: parent.width*0.5 - parent.spacing /2
                height: parent.height
                GridLayout{
                    id: gridDashboard
                    columns: 10
                    rows: 2
                    width: parent.width
                    height: parent.height
                    columnSpacing: 0
                    rowSpacing: 0
                    property int preWidth: -1
                    property int preHeight: -1

                    Repeater{
						model: model_dashboard_TOMOC1
                        delegate: Rectangle{
                            color: "#303030"
                            visible: _normal
                            Layout.columnSpan: _scaleWidth
                            Layout.rowSpan: _scaleHeight
                            Layout.preferredWidth:  gridDashboard.width/gridDashboard.columns* Layout.columnSpan
                            Layout.preferredHeight: gridDashboard.height /gridDashboard.rows* Layout.rowSpan
                            Track{
                                id: track_dashboard
                                camName: trackName
                                width: parent.width
                                height: parent.height
                                _scaleTabbar: scaleTabbar
                                _tabControllerModelName: tabControllerModelName
                                private_control_button: privateButton != undefined ? privateButton:""
                                private_control_name: privateName != undefined ? privateName:""
                                _viewModel: _dashModel != null ? _dashModel.trackListAt(index) : null

								// Single point control to update RViz position, size and visibility
								// based on TomoPose window's state
								// Udupa; 22Aug'23
								function updateDim()
								{
									if(track_dashboard._tabControllerModelName !== "tomoVM") return
									if(typeof tomoVM === 'undefined') return

									var point = mapFromGlobal(0, 0)
									var border = 5;
									var border2 = border * 2
									var tabBarHeight = 37
									tomoVM.updateRVizMetric(!dashboardVM.isCovered && track_dashboard.visible,  -point.x + border, -point.y + tabBarHeight + border, track_dashboard.width - border2, track_dashboard.height - tabBarHeight - border2, tomoPoseIsMax)
								}
								onEnabledChanged:
									if(visible)
										track_dashboard.updateDim()
								onVisibleChanged:
									if(typeof track_dashboard !== "undefined")
										track_dashboard.updateDim()

                                onHeightChanged:{
									if(typeof track_dashboard !== "undefined")
										track_dashboard.updateDim()
                                }

								onWidthChanged:
									if(typeof track_dashboard !== "undefined")
										track_dashboard.updateDim()

                                onDoubleClickTrack: {
									// if(camName === "TomoPose"){
                                    //     rootDashBoard.tomoPoseIsMax = !rootDashBoard.tomoPoseIsMax
                                    //     if(tomoPoseIsMax == true){
                                    //         hideTomoEye()
                                    //     }
                                    //     else if(tomoPoseIsMax == false){
                                    //         showNormalView()
                                    //     }
                                    // }
                                    // _dashModel.doubleClickSlots(camName)
                                }
                                Connections{
                                    target: main_view_layout
                                    ignoreUnknownSignals: true
                                    onYChanged:{
                                        if(track_dashboard.visible)
                                            track_dashboard.updateDim()
                                    }

                                }

								Connections{
									target: typeof tomoVM === 'undefined' ? null : tomoVM
                                    ignoreUnknownSignals: true
									onInitRvizStartState:{
										if(track_dashboard.visible){
											track_dashboard.updateDim()
										}
									}
                                    onSendVisibleRviz:{
                                        track_dashboard.updateDim()
                                    }
								}
                                Connections{
                                    target: dashboardVM
                                    ignoreUnknownSignals: true
                                    onIsCoveredChanged:{
                                        track_dashboard.updateDim()
                                    }
                                }
                                Connections{
                                    id: min_max_dashboard_connections
                                    target:_dashModel
                                    ignoreUnknownSignals: true
                                    onInitMaxView:{
                                        if(tab === "TomoPose"){
                                            rootDashBoard.tomoPoseIsMax = true
                                        }
                                        else if(tab === "TomoEye"){
                                            rootDashBoard.tomoEyeIsMax = true
                                        }
                                    }
                                }
                            }
                        }
                    }

                }
            }
            Item{
                id: tomoEyeRec
                width: parent.width*0.5 - parent.spacing /2
                height: parent.height
                visible: true
                GridLayout{
                    id: gridDashboardC
                    columns: 2
                    rows: 2
                    width: parent.width
                    height: parent.height

                    property int preWidth: -1
                    property int preHeight: -1
                    Repeater{
                        id: vision_rpt
                        model: model_dashboard_TOMOC2
                        delegate: Rectangle{
                            id: vision_rec
                            color: "#303030"
                            visible: _normal
                            Layout.columnSpan:  isTomoPoseHide ? 1 : _scaleWidth
                            Layout.rowSpan:     isTomoPoseHide ? 2 : _scaleHeight
                            Layout.preferredWidth:  gridDashboardC.width/gridDashboardC.columns* Layout.columnSpan
                            Layout.preferredHeight: gridDashboardC.height /gridDashboardC.rows* Layout.rowSpan
                            Track{
                                id: track_dashboardC
								property bool _isFull: false
                                camName: trackName
                                width: parent.width
                                height: parent.height
                                _scaleTabbar: scaleTabbar
                                _tabControllerModelName: tabControllerModelName
                                private_control_button: privateButton != undefined ? privateButton:""
                                private_control_name: privateName != undefined ? privateName:""
                                _viewModel: _dashModel != null ? _dashModel.trackListAt(index+1) : null
                                onDoubleClickTrack: {
                                    _dashModel.doubleClickSlots(camName)
									_isFull = !_isFull
									if(_isFull == true){
										hideTomoPose()
										for(var i = 0 ; i < vision_rpt.count; i++)
										{
											var item = vision_rpt.itemAt(i)
											item.visible = false
										}
										vision_rec.Layout.columnSpan = 2
										vision_rec.Layout.rowSpan = 2
										vision_rec.visible = true
									}
									else if(_isFull == false){
										showNormalView()
									}
                                }
                                Connections{
                                    target: dashboardVM
                                    ignoreUnknownSignals: true
                                    onHideTomoPoseChanged:{
                                        if(value){
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
                                        else if(!value){
                                            showNormalView()
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

    Connections{
        target: _dashModel
        ignoreUnknownSignals: true
        onSetConfigTomOView:{
            model_dashboard_TOMOC1.append({"tabControllerModelName": "tomoVM",
                                            "privateButton":"credit-card;t fullscreen-exit|fullscreen;t chevron-left",
                                            "privateName":"rviz fullscreen hide",
                                            "trackName": "TomoPose",
                                            "_scaleWidth": 10,
                                            "_scaleHeight": 2,
                                            "scaleTabbar": 1.0,
                                            "_normal": true
            })
            model_dashboard_TOMOC2.append({"tabControllerModelName": "headVM",
                                            "trackName": "TomoEye",
                                            "privateButton": "webcam;t checkbox-marked|checkbox-blank;t export",
                                            "privateName": "live auto_switch export",
                                            "_scaleWidth": 2,
                                            "_scaleHeight": 1,
                                            "scaleTabbar": 1.0,
                                            "_normal": true
            })
            model_dashboard_TOMOC2.append({"tabControllerModelName": "shipperVM",
                                            "trackName": "Shipper",
                                            "privateButton": "webcam;t checkbox-marked|checkbox-blank;t export",
                                            "privateName": "live auto_switch export",
                                            "_scaleWidth": 1,
                                            "_scaleHeight": 1,
                                            "scaleTabbar": 1.0,
                                            "_normal": true
            })
            model_dashboard_TOMOC2.append({"tabControllerModelName": "pimVM",
                                            "trackName": "PIM",
                                            "privateButton": "webcam;t checkbox-marked|checkbox-blank;t export",
                                            "privateName": "live auto_switch export",
                                            "_scaleWidth": 1,
                                            "_scaleHeight": 1,
                                            "scaleTabbar": 1.0,
                                            "_normal": true
            })
        }
    }
    ListDir{
        id: listDirPopup
        width: 600
        visible: rootDashBoard.listDirVisible
        x: parent.width / 2 - width / 2
        y: parent.height / 2 - height / 2
        onVisibleChanged: if(!visible) rootDashBoard.listDirVisible = false
    }
}
