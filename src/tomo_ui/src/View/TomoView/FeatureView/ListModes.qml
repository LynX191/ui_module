import QtQml 2.0
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
//My import
import "../../../Component/MaterialDesign"
import "../../../Component"
//end

Frame {
    property QtObject _viewModel: QtObject{}
    id: item

    Column{
        width : parent.width
        height: parent.height
        ListView {
            id: listView

            property int delegateHeight: Math.round(listView.pointSizeText*10)
            property int pointSizeText: 10

            signal settingChanged()

            width : parent.width
            height: parent.height
            model: _viewModel === null ? null : _viewModel.model
            clip: true
            focus: true
            ScrollBar.vertical: ScrollBar{
                height:30
                width: 10
                active: true
                policy: ScrollBar.AsNeeded
                snapMode:ScrollBar.SnapOnRelease
                focusPolicy: Qt.WheelFocus
                Behavior on position {
                        NumberAnimation {
                            duration: 1000
                        }
                }
                contentItem: Rectangle {
                    radius: implicitWidth / 2
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: "#00d2ff" }
                        GradientStop { position: 1.0; color: "#3a7bd5" }
                    }
                }
            }
            delegate:RowLayout {
				width: parent.width
				height: 25
				spacing: 2
				Text{
					Layout.fillWidth: true
					Layout.preferredWidth: Math.round(parent.width*0.3)
					Layout.alignment: Qt.AlignHCenter | Qt.AlignLeft
					Layout.leftMargin: 5
					font.pointSize:listView.pointSizeText
					font.family: "Meiryo UI Regular"
					font.bold: true
					text: qsTr(alias_name)
					color: "#ffffff"
				}

				Rectangle{
					Layout.fillWidth: true
					Layout.preferredWidth: Math.round(parent.width*0.35)
					Layout.preferredHeight: parent.height
					Layout.alignment: Qt.AlignHCenter | Qt.AlignLeft
					color: "transparent"

					function splitString(text, character){
						return text.split(character)
					}
					ComboBoxCustom{
						id:combobox_custom
						width: parent.width
						implicitHeight: 34
						anchors.verticalCenter: parent.verticalCenter
						anchors.horizontalCenter:parent.horizontalCenter
						_model: parent.splitString(values_name, ";")
						_bgCombobox: "#141414"
						_bgPopup: "#141414"
						_opacityPopup: 0.6
						modelDataColor:"white"
						//onCurrentIndexChanged: {
						onActivated: {
							if(!listView.visible) return;
							listView.settingChanged()
						}
					}
				}

				Connections{
					target: typeof(_viewModel) === "undefine" ? null :_viewModel
					ignoreUnknownSignals: true
					onApplyMode:{
						if(acmd !== String(cmd)) return
						if(avalue < 0)
							avalue = combobox_custom.find(String(-avalue))
						console.log("onApplyMode: Command=", acmd, "; Value=", avalue)
						combobox_custom.currentIndex = avalue
					}
				}
            }
            Connections{
                target: listView
                ignoreUnknownSignals: true
                onSettingChanged:{
                    var config = "";
                    for(var i=0; i <= listView.count ; i++){
                        var data = listView.contentItem.children[i]
                        if( data === undefined) continue;
                        if(data.children[0] === undefined) continue;
                        //config += data.children[0].text + ";" + data.children[1].children[0].textAt(data.children[1].children[0].currentIndex) + "|"
                        config += data.children[1].children[0].currentIndex + "|"
                    }
                    _viewModel.modeUserChoiceSlots(config)
                }
            }

        }
    }
}
