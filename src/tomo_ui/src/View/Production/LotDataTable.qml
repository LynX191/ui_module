import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

import "../../Component"
import "../../Component/MaterialDesign"

Popup {
    id: historyRoot
    width: 1800
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
        ListElement{_info:"Lot End Time"}
        ListElement{_info:"Total Runtime"}
        ListElement{_info:"Total Downtime"}
        ListElement{_info:"Average Cartons\n Per Minute"}
        ListElement{_info:"Average Cycle Time"}
        ListElement{_info:"No. of Completed Trays"}
        ListElement{_info:"No. of Loaded Cartons"}
    }
    property var valueColName: ["packSize",
                                "startTime", "endTime",
                                "runTime", "downTime",
                                "averageCartonPerMin", "averageCycleTime",
                                "completedTrays", "loadedCartons"]
    Item{
        id: listHistoryFrame
        width: parent.width
        height: parent.height
        clip:true

    ListView{
        id: listHistory
        anchors.fill: parent
        model: productionVM.lotDataModel
        headerPositioning: ListView.OverlayHeader
        delegate: Item{
			id: lotDataItem
            width: listHistory.width
            height: 50
			property int indexColor: index
            Row{
                anchors.fill: parent
                spacing: 0
                Repeater{
                    id: lotDataRpt
                    property var lotData:[eval("modelData." + valueColName[0]),eval("modelData." + valueColName[1]),
                                        eval("modelData." + valueColName[2]),eval("modelData." + valueColName[3]),
                                        eval("modelData." + valueColName[4]),eval("modelData." + valueColName[5]),
                                        eval("modelData." + valueColName[6]),eval("modelData." + valueColName[7]),
                                        eval("modelData." + valueColName[8])]
                    model: historyHeader.count
                    delegate: Rectangle{
                        width: parent.width / historyHeader.count
                        height: 50
                        border.width: 1
                        border.color: "black"
                        color: lotDataItem.indexColor % 2 === 0 ? "#4c5052" : "#838b8f"
                        Text{
                            anchors.fill: parent
                            font.pointSize: 12
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
