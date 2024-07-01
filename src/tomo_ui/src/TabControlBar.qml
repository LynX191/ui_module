import QtQml 2.2
import QtQuick 2.9
import QtQuick.Window 2.3
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2

// My import
import "View/TomoView"
//import "View/IntroView"
import "View/DashboardView"
import "View/SettingView"
import "View/IOControl"
// end
Item {
    width: parent.width
    height: parent.height
    property int _fontPointSize: 14
    property string _fontFamily:  "Arial"

    property bool onSetting: false
    property bool allowCurrentIndexTabChange: false
    property int _heightIcon: 48
    property int _heightButton: 60
    property int _spacing: 0
    property int _numComponent: loginVM.superUserActive ? 3 : 1

    property var optispec_view: null
    property var dashboard_view: null
    property var tomo_view: null
    property var production_view: null
    property var station_view: null
    property var io_view: null
    property var setting_view: null


    onOnSettingChanged: {
        var preIndex = stackTabControl.currentIndex
        stackTabControl.currentIndex = onSetting ? stackTabControl._indexSettingPage : preIndex
        preIndex = tabBar.currentIndex
        tabBar.currentIndex = onSetting ? -1 : preIndex
    }

    ListModel{
        id: tab_model
    }
    // create tab
    TabBar {
        id: tabBar
        clip: true
        height: parent.width
        width: parent.height
        enabled: (productionVM.enabledForRunLot && productionVM.fileValid && master_app.xavierReady) || ((( systemBarVM.processState === "Pause" || systemBarVM.processState === "Error" ) && ( systemBarVM.lotState === "Started" || systemBarVM.lotState === "Ending" )) && !productionVM.operationValue) || loginVM.superUserActive
		opacity: enabled? 1:0.5
        y: width
        property bool isCompleted: false
        transform: Rotation {
            origin.x: 0;
            origin.y: 0;
            axis { x: 0; y: 0; z: 1 }
            angle: -90
        }
        background: Rectangle{
            width: parent.width
            height: parent.height
            color: "#4F5052"
        }
        Repeater{
            id: tabRpt
            model: tab_model
            width: parent.width
            height: parent.height
            TabButton {
                id: tabControl
                text: nameTab
                height: parent.height
                visible:{
                    if(nameTab === "Dashboard" )
                        return loginVM.superUserActive
                    else
                        return true
                }
                width: {
                    if(loginVM.superUserActive)
                        return implicitWidth * 1.57
                    else{
                        if(nameTab === "Dashboard" )
                            return 0
                        else
                            return implicitWidth * 2.493
                    }
                }
                Behavior on width {
                    NumberAnimation {
                        duration: 200
                    }
                }
                contentItem: Text {
                    width: parent.width
                    text: parent.text
                    font.capitalization: Font.Capitalize
                    font.family: _fontFamily
                    font.pointSize: _fontPointSize
                    font.bold: true
                    opacity: enabled ? 1.0 : 0.3
                    color: parent.down ? "#B6E0DE" : "#F8F8FA"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    elide: Text.ElideLeft
                }
                onClicked: {
                    loginVM.updateIdleTimer()
                    if(onSetting)
                        tabBar.currentIndex = index
                    onSetting = false
                    stackTabControl.currentIndex =  tabBar.currentIndex
                }
            }
        }
        onCurrentIndexChanged: {
            if(!allowCurrentIndexTabChange) return
            // if(!tabBar.isCompleted){
            //     tabBar.isCompleted = true
            //     return
            // }
            if(currentIndex === -1) return
            controlBarVM.tabCurrentChanged(currentIndex)
        }

        Connections{
            target: controlBarVM
            ignoreUnknownSignals: true
            onSetCurrentIndexTab:{
                tabBar.currentIndex = index
                stackTabControl.currentIndex = index
            }
        }
        Connections{
            target: productionVM
            ignoreUnknownSignals: true
            onSwitchToProduction:{
                tabBar.currentIndex = 1
                stackTabControl.currentIndex = 1
            }
        }
    }

    //tab layout
    StackLayout {
        id:stackTabControl
        x: mainViewItem.x
        y: main_view_layout.y - (_heightIcon + _heightButton*_numComponent + _spacing*(_numComponent+1))
        width: main_view_layout.width
        height: main_view_layout.height
		enabled: master_app.xavierReady || loginVM.superUserActive
		opacity: enabled? 1:0.5

        property int _indexSettingPage: tabBar.count

        function createTomO() {
            var component;

            component = Qt.createComponent("View/TomoView/TomoView.qml");
            if (component.status === Component.Ready){
                tomo_view = component.createObject(stackTabControl, {
                                                    _tomoModel: tomoVM,
                                                    width: stackTabControl.width,
                                                    height: stackTabControl.height,
                                                    tomoPoseIsMax: dashboard_view === null ? false : tomoPoseIsMax
                                                   });
                if (tomo_view === null) {
                    console.log("StackLayout >>>> Error creating object TomoView.qml");
                }
            }
            else if (component.status === Component.Error) {
                console.log("StackLayout >>>> Error loading component:", component.errorString());
            }
        }
        // Connections{
        //     target: tomo_view
        //     ignoreUnknownSignals: true
        //     onVisibleTomOPage:{
        //         tomoVM.visibilityChanged(visible)
        //     }
        // }

        function createDash() {
            var component;

            component = Qt.createComponent("View/DashboardView/DashboardView.qml");
            if (component.status === Component.Ready){
                dashboard_view = component.createObject(stackTabControl, {
                                                        _dashModel: dashboardVM,
                                                        width: stackTabControl.width,
                                                        height: stackTabControl.height,
                                                        });
                if (dashboard_view === null) {
                    console.log("StackLayout >>>> Error creating object DashboardView.qml");
                }
            }
            else if (component.status === Component.Error) {
                console.log("StackLayout >>>> Error loading component:", component.errorString());
            }
        }

        Connections{
            target: dashboard_view
            ignoreUnknownSignals: true
            onCloseSettingPage:{
                onSetting = false
            }
            // onVisibleSettingPage:{
            //     dashboardVM.visibilityChanged(visible)
            // }
        }

        function createSet() {
            var component;

            component = Qt.createComponent("View/SettingView/SettingView.qml");
            if (component.status === Component.Ready){
                setting_view = component.createObject(stackTabControl, {
                                                        _settingModel: settingVM,
                                                        width: stackTabControl.width,
                                                        height: stackTabControl.height,
                                                      	});
                if (setting_view === null) {
                    console.log("StackLayout >>>> Error creating object SettingView.qml");
                }
            }
            else if (component.status === Component.Error) {
                console.log("StackLayout >>>> Error loading component:", component.errorString());
            }
        }

        function createIO() {
            var component;

            component = Qt.createComponent("View/IOControl/IOControl.qml");
            if (component.status === Component.Ready){
                io_view = component.createObject(stackTabControl, {
                                                    width: stackTabControl.width,
                                                    height:stackTabControl.height,
                                                 	});
                if (io_view === null) {
                    console.log("StackLayout >>>> Error creating object IOControl.qml");
                }
            }
            else if (component.status === Component.Error) {
                console.log("StackLayout >>>> Error loading component:", component.errorString());
            }
        }

        function createStation() {
            var component;

            component = Qt.createComponent("View/StationControl/StationControl.qml");
            if (component.status === Component.Ready){
                station_view = component.createObject(stackTabControl, {
                                                          width: stackTabControl.width,
                                                          height: stackTabControl.height,
                                                      	});
                if (station_view === null) {
                    console.log("StackLayout >>>> Error creating object StationControl.qml");
                }
            }
            else if (component.status === Component.Error) {
                console.log("StackLayout >>>> Error loading component:", component.errorString());
            }
        }

        function createProduction() {
            var component;

            component = Qt.createComponent("View/Production/ProductionView.qml");
            if (component.status === Component.Ready){
                production_view = component.createObject(stackTabControl, {
                                                          width: stackTabControl.width,
                                                          height: stackTabControl.height,
                                                        });
                if (production_view === null) {
                    console.log("StackLayout >>>> Error creating object Production.qml");
                }
            }
            else if (component.status === Component.Error) {
                console.log("StackLayout >>>> Error loading component:", component.errorString());
            }
        }
    }

    Connections{
        target: controlBarVM
        ignoreUnknownSignals: true
        onSetConfigTomOView:{
            tab_model.clear()
            stackTabControl.createIO()
            // stackTabControl.createStation()
            stackTabControl.createProduction()
            // stackTabControl.createTomO()
            stackTabControl.createDash()
			tab_model.append({"nameTab" : "I/O Control"})
			// tab_model.append({"nameTab" : "Station"})
            tab_model.append({"nameTab" : "Production & Control"})
			// tab_model.append({"nameTab" : "TomO"})
			tab_model.append({"nameTab" : "Dashboard"})

            stackTabControl.createSet()

            controlBarVM.setComponentCompleted()
            allowCurrentIndexTabChange = true
        }
    }
}
