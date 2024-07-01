import QtQuick 2.9
import QtQml 2.0
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2

// My import
import "../../View"
import "../../Dialog"
import "../../Component"
import "../../Component/MaterialDesign"
import "FeatureView"
//end
Item {
    id: rootTomoPose
    width: parent.width
    height: parent.height

    // signal visibleTomOPage()

    property int _fontPointSize: 10
    property string _fontFamily:  "Adobe Gothic Std B"
    property QtObject _tomoModel: QtObject{}
    property bool tomoPoseIsMax: false

    // onVisibleChanged:{
    //     visibleTomOPage()
    // }
    enabled: true
    opacity: enabled? 1:0.5
    Row{
        anchors.fill: parent
        spacing: 5
        //track
        Track{
            id: tomo_track
            camName: model_tomo.get(0).trackName
            width: Math.round(parent.width*0.8 - parent.spacing/2)
            height: parent.height
            _scaleTabbar: model_tomo.get(0).scaleTabbar
            _tabControllerModelName: model_tomo.get(0).tabControllerModelName
            private_control_button: model_tomo.get(0).privateButton !== undefined ? model_tomo.get(0).privateButton:""
            private_control_name: model_tomo.get(0).privateName !== undefined ? model_tomo.get(0).privateName:""
            _viewModel: _tomoModel != null ? _tomoModel.trackListAt(0) : null

			// Single point control to update RViz position, size and visibility
			// based on TomoPose window's state
			// Udupa; 22Aug'23
			function updateDim()
			{
				if(tomo_track._tabControllerModelName !== "tomoVM") return
				if(typeof tomoVM === 'undefined') return

				var point = mapFromGlobal(0, 0)
				var border = 5;
				var border2 = border * 2
				var tabBarHeight = 37
				tomoVM.updateRVizMetric(!dashboardVM.isCovered && tomo_track.visible, -point.x + border, -point.y + tabBarHeight + border, tomo_track.width - border2, tomo_track.height - tabBarHeight - border2, false)
            }

			onEnabledChanged:
				if(visible)
					tomo_track.updateDim()

			onVisibleChanged:
				if(typeof tomo_track !== "undefined")
					tomo_track.updateDim()

			onHeightChanged:
				if(typeof tomo_track !== "undefined")
					tomo_track.updateDim()

			onWidthChanged:
				if(typeof tomo_track !== "undefined")
					tomo_track.updateDim()

            onDoubleClickTrack: {
                if(!rootTomoPose.tomoPoseIsMax) {
                    rootTomoPose.tomoPoseIsMax = true
                    return
                }
                _tomoModel.doubleClickSlots(camName)
            }
			Connections{
				target: typeof tomoVM === 'undefined' ? null : tomoVM
				ignoreUnknownSignals: true
				onInitRvizStartState:{
					if(tomo_track.visible){
						tomo_track.updateDim()
					}
				}
			}
            Connections{
                target: dashboardVM
                ignoreUnknownSignals: true
                onIsCoveredChanged:{
                    tomo_track.updateDim()
                }
            }
        }
        //state and mode layout
        Column{
            id: state_mode_layout
            width: Math.round(parent.width*0.2 - parent.spacing / 2)
            height: parent.height
            spacing:0
            padding: 0

            //mode

            //state
        }
    }

	Connections{
        target: mainVM
        ignoreUnknownSignals: true
        onSendEnableAllsLayout:{
            state_mode_layout.enabled = isEnable
        }
	}

    ListModel {
        id: model_tomo
        ListElement {
            tabControllerModelName: "tomoVM"
            privateButton: "" // name_button;toggle
            privateName: ""
            trackName: "TomoPose"
            scaleTabbar: 0.5
        }
    }

	Connections{
		target: main_view_layout
		ignoreUnknownSignals: true
		onVisibleChanged:{
			if(typeof tomo_track !== "undefined")
				tomo_track.visible = main_view_layout.visible
		}
	}
}
