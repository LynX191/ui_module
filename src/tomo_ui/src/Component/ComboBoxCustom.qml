import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Window 2.3
import "MaterialDesign"

ComboBox {
    id: combobox
    property var _model: []
    property int _fontSize: 12
    property string _fontFamily: "Arial"
    property color _bgCombobox: "#000000"
    property color _bgPopup: "#000000"
    property double _opacityPopup: 1.0
    property var modelDataColor: "white"
    property bool isVerticalPopup: true
    property var maxWidthHPopup: 300
    property var _viewText:""

    model: _model
    delegate: ItemDelegate {
        width: combobox.width
        height: 30
        contentItem: Text {
            text: modelData
            color: modelDataColor
            font.pixelSize: combobox._fontSize
            font.family: combobox._fontFamily
            verticalAlignment: Text.AlignVCenter
            leftPadding: 0
//            rightPadding: combobox.indicator.width + combobox.spacing
        }
        highlighted: combobox.highlightedIndex === index
    }
    indicator: MaterialDesignIcon {
        id: name
        visible: isVerticalPopup
        name: "chevron-down"
        x: combobox.width - width - combobox.rightPadding/4
        y: combobox.topPadding + (combobox.availableHeight - height) / 2
        color: "white"
    }

    contentItem: Text {
        leftPadding: 10
        rightPadding: isVerticalPopup?combobox.indicator.width + combobox.spacing:0
        text: _viewText!==""?_viewText:combobox.displayText
        font.pixelSize: combobox._fontSize
        font.family: combobox._fontFamily
        color: "white"
		verticalAlignment: Text.AlignVCenter
		horizontalAlignment: Text.AlignHCenter
        elide: Text.ElideRight
    }


    popup: Popup {
        y: isVerticalPopup?combobox.height:-4
        width: isVerticalPopup?combobox.width:contentItem.implicitWidth>=maxWidthHPopup?maxWidthHPopup:contentItem.implicitWidth
        height: isVerticalPopup?contentItem.implicitHeight:40
        opacity: combobox.pressed ? 1.0 : combobox._opacityPopup
        padding: 0

        contentItem: ListView {
            clip: true
            implicitHeight: contentHeight
            implicitWidth: contentWidth
            model: combobox.popup.visible ? combobox.delegateModel : null
            currentIndex: combobox.highlightedIndex
            orientation: isVerticalPopup?ListView.Vertical:ListView.Horizontal
            ScrollIndicator.vertical: ScrollIndicator {}
            ScrollIndicator.horizontal: ScrollIndicator {}
        }
        background: Rectangle {
            radius: 2
            color: combobox._bgPopup
            border.width: 1
            border.color: "gray"
        }
    }
}


