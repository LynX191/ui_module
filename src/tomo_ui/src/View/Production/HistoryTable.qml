import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

import "../../Component"
import "../../Component/MaterialDesign"

Popup {
    id: historyRoot
    width: 1750
    height: listHistory.contentHeight+padding*2>900?900:listHistory.contentHeight+padding*2
    padding: 0
    modal: true
    background: Rectangle{
        color: "#232526"
        border.color: "gray"
        border.width: 2
    }
    ListModel{
        id: historyHeader
        ListElement{_info:"Pack Size"}
        ListElement{_info:"Lot Start Time"}
        ListElement{_info:"Cycle Time\n(sec)"}
        ListElement{_info:"Carton Per Minute"}
        ListElement{_info:"Average Cycle Time\n(sec)"}
        ListElement{_info:"Average Carton\nPer Minute"}
        ListElement{_info:"No. of Completed Trays"}
        ListElement{_info:"No. of Loaded Cartons"}
    }
    property var valueColName: ["packSize","startTime","cycleTime","cartonPerMin","averageCycleTime","averageCartonPerMin","completedTrays","loadedCartons"]
    Item{
        id: listHistoryFrame
        width: parent.width
        height: parent.height
        clip:true

		ListView{
			id: listHistory
			anchors.fill: parent
			model: productionVM.historyModel
			headerPositioning: ListView.OverlayHeader
			delegate: Item{
				id: historyItem
				width: listHistory.width
				height: 50
				property string indexColor: modelData.color
				Row{
					anchors.fill: parent
					spacing: 0
					Repeater{
						id: lotDataRpt
						property var lotData:[eval("modelData." + valueColName[0]),eval("modelData." + valueColName[1]),
											eval("modelData." + valueColName[2]),eval("modelData." + valueColName[3]),
											eval("modelData." + valueColName[4]),eval("modelData." + valueColName[5]),
											eval("modelData." + valueColName[6]),eval("modelData." + valueColName[7])]
						model: historyHeader.count
						delegate: Rectangle{
							width: parent.width / historyHeader.count
							height: 50
							border.width: 1
							border.color: "black"
							color: historyItem.indexColor.length != 7 ? "#4c5052" : historyItem.indexColor
							Text{
								anchors.fill: parent
								font.pointSize: 13
								horizontalAlignment: Text.AlignHCenter
								verticalAlignment: Text.AlignVCenter
								text: lotDataRpt.lotData[index]
								color: "white"
							}
						}
					}
				}
			}
			header: Row{
				width: listHistory.width
				height: 100
				z:5
				Repeater{
					model: historyHeader
					delegate: Rectangle{
						width: parent.width / historyHeader.count
						height: 100
						color: "black"
						border{
							width: 0.5
							color: "gray"
						}
						Text{
							anchors.fill: parent
							horizontalAlignment: Text.AlignHCenter
							verticalAlignment: Text.AlignVCenter
							text: _info
							wrapMode: Text.Wrap
							font.pointSize: 14
							color: "white"
						}
					}
				}
			}
		}
    }
}
