import QtQml 2.3
import QtQuick 2.9


// Carefully when using signal in qml, cause program has been crashed without any error
Item {
    id: track

    property QtObject _viewModel: QtObject{}

    signal doubleClickTrack()

    property string camName: ""
    property string private_control_button: ""
    property string private_control_name: ""
    property string _tabControllerModelName: ""
    property int _numberTabTrack: 0
    property int _currentIndexTrack: 0
    property double _scaleTabbar: 0.0

    Rectangle{
        width: parent.width
        height: parent.height
        color: "transparent"
        border.width: 1
        border.color: "#89868D"
        ImageView{
            anchors.centerIn: parent
            width: Math.round(parent.width - 4)
            height: Math.round(parent.height - 4)
            _scaleTabbar: track._scaleTabbar
            _trackViewModel: track._viewModel != null ? track._viewModel : null
            _tabControllerModelName: track._tabControllerModelName
            _private_control_button: private_control_button
            _private_control_name: private_control_name

            onDoubleClickImageView: {
                track.doubleClickTrack()
            }
            Component.onCompleted: {
                track._numberTabTrack = _numberTab
            }
            on_CurrentIndexTabChanged: {
                if(typeof bomVM === 'undefined') return
                if(typeof headVM === 'undefined') return
                if(typeof shipperVM === 'undefined') return
                if(typeof pimVM === 'undefined') return

                if(
                   bomVM.currentTabViewModel === null ||
                   headVM.currentTabViewModel === null ||
                   shipperVM.currentTabViewModel  === null ||
                   pimVM.currentTabViewModel === null ) return
                // if(camName === 'OptiSpec')
                //     optispecVM.currentTabViewModel = _currentIndexTab
                if(camName === 'TomoEye')
                    headVM.currentTabViewModel = _currentIndexTab
                if(camName === 'Shipper')
                    shipperVM.currentTabViewModel = _currentIndexTab
                if(camName === 'PIM')
                    pimVM.currentTabViewModel = _currentIndexTab
                if(camName === 'TomoPose')
                    tomoVM.currentTabViewModel = _currentIndexTab
                _currentIndexTrack = _currentIndexTab
            }
        }
    }
}
